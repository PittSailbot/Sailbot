import glob
import json
import os
import time
import base64
import socket
from dotenv import load_dotenv

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import String

load_dotenv()

GPS_BAUD = int(os.environ.get("RTK_GPS_BAUD", "460800"))
GPS_PORT = os.environ.get("RTK_GPS_PORT")
GPS_READ_TIMEOUT = float(os.environ.get("RTK_GPS_READ_TIMEOUT", "0.1"))
GGA_PREFIXES = ("$GNGGA", "$GPGGA")
RMC_PREFIXES = ("$GNRMC", "$GPRMC")


# Update these in the .env to change stations
NTRIP_HOST = os.getenv("RTK_NTRIP_HOST")
NTRIP_PORT = int(os.getenv("RTK_NTRIP_PORT", 0))
MOUNTPOINT = os.getenv("RTK_NTRIP_MOUNTPOINT", "RTCM33GRCEJ")
NTRIP_USER = os.getenv("RTK_NTRIP_USER")
NTRIP_PASS = os.getenv("RTK_NTRIP_PASS")

FIX_TYPE_TEXT = {
    0: "No Fix",
    1: "GPS SPS Mode",
    2: "DGNSS (SBAS/DGPS)",
    3: "GPS PPS Fix",
    4: "Fixed RTK",
    5: "Float RTK",
}

def convert_to_decimal_degree(value, direction):
    if not value:
        return 0.0
    degree_digits = 2 if direction in ("N", "S") else 3 if direction in ("E", "W") else 0
    if degree_digits == 0 or len(value) <= degree_digits:
        return 0.0

    degrees = float(value[:degree_digits])
    minutes = float(value[degree_digits:])
    decimal_degrees = degrees + minutes / 60.0
    if direction in ("S", "W"):
        decimal_degrees = -decimal_degrees
    return decimal_degrees

def split_nmea(sentence):
    return sentence.strip().split(",")

def parse_gga(sentence):
    parts = split_nmea(sentence)
    if len(parts) < 15:
        return None
    try:
        fix_type = int(parts[6])
        satellites = int(parts[7])
        hdop = float(parts[8])
        latitude = convert_to_decimal_degree(parts[2], parts[3])
        longitude = convert_to_decimal_degree(parts[4], parts[5])
    except (ValueError, IndexError):
        return None

    return {
        "fix": fix_type > 0,
        "fix_type": fix_type,
        "fix_text": FIX_TYPE_TEXT.get(fix_type, f"Unknown ({fix_type})"),
        "satellites": satellites,
        "hdop": hdop,
        "lat": latitude,
        "lon": longitude,
        "rtk_data": "RECEIVED" if fix_type in (4, 5) else "NOT RECEIVED",
    }

def parse_rmc(sentence):
    parts = split_nmea(sentence)
    if len(parts) < 12:
        return None
    rmc_valid = parts[2] == "A"
    try:
        speed = float(parts[7]) if rmc_valid and parts[7] else 0.0
        heading = float(parts[8]) if rmc_valid and parts[8] else 0.0
    except (ValueError, IndexError):
        speed = 0.0
        heading = 0.0
    return {"rmc_valid": rmc_valid, "speed": speed, "heading": heading}

def scan_gps_ports():
    patterns = ["/dev/ttyUSB*", "/dev/ttyAMA*", "/dev/ttyACM*", "/dev/serial0"]
    ports = []
    for pattern in patterns:
        ports.extend(glob.glob(pattern))
    return sorted(set(ports))

def find_gps_port(baudrate=GPS_BAUD, timeout=0.5, logger=None):
    if GPS_PORT:
        return GPS_PORT
    for port in scan_gps_ports():
        try:
            with serial.Serial(port, baudrate=baudrate, timeout=timeout) as candidate:
                time.sleep(1)
                data = candidate.read(500).decode(errors="ignore")
                if any(prefix in data for prefix in (*GGA_PREFIXES, *RMC_PREFIXES)):
                    if logger:
                        logger.info(f"GPS device found on {port}")
                    return port
        except (OSError, serial.SerialException):
            continue
    return None


class RTK(Node):
    def __init__(self):
        super().__init__("rtk")
        self.logging = self.get_logger()

        port = find_gps_port(logger=self.logging)
        if port is None:
            raise RuntimeError("Failed to find a GPS serial port emitting NMEA data.")

        self.serial_port = serial.Serial(port, baudrate=GPS_BAUD, timeout=GPS_READ_TIMEOUT)
        self.logging.info(f"Reading/Writing GPS NMEA data from {port} at {GPS_BAUD} baud")

        self.pub = self.create_publisher(String, "/GPS", 1)
        self.gps_timer = self.create_timer(0.1, self.gps_thread)
        self.rtk_timer = self.create_timer(0.1, self.ntrip_client_thread)

        self.gps_data = {
            "fix": False,
            "fix_type": 0,
            "fix_text": "No Fix",
            "satellites": 0,
            "hdop": 0.0,
            "lat": 0.0,
            "lon": 0.0,
            "speed": 0.0,
            "heading": 0.0,
            "rtk_data": "NOT RECEIVED",
        }
        self.has_gga = False
        self.has_rmc = False
        self.nmea_buffer = ""
        self.last_gga_sentence = "" # Stored to send back to NTRIP Caster
        self.last_fix = time.time()

        # Connect to RTK
        self.logging.info(f"Connecting to NTRIP Caster {NTRIP_HOST}:{NTRIP_PORT}...")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(10)
        self.sock.connect((NTRIP_HOST, NTRIP_PORT))

        auth_str = f"{NTRIP_USER}:{NTRIP_PASS}"
        auth_enc = base64.b64encode(auth_str.encode()).decode()
        request = (
            f"GET /{MOUNTPOINT} HTTP/1.1\r\n"
            f"User-Agent: NTRIP ROS2 Client\r\n"
            f"Accept: */*\r\n"
            f"Connection: keep-alive\r\n"
            f"Authorization: Basic {auth_enc}\r\n"
            f"\r\n"
        )
        self.sock.send(request.encode())
        self.sock.settimeout(0.1)
        self.logging.info("NTRIP connected successfully! Streaming differential corrections...")

        self.last_gga_sent_time = 0.0

    def gps_thread(self):
        try:
            character = self.serial_port.read().decode(errors="ignore")
            
            if character == "\n":
                line = self.nmea_buffer.strip()
                self.nmea_buffer = ""
                self.process_nmea_line(line)
            elif character != "\r":
                self.nmea_buffer += character

        except (OSError, serial.SerialException) as error:
            self.logging.error(f"GPS read error: {error}")
            time.sleep(0.1)

    def process_nmea_line(self, line):
        if not line:
            return

        gps_data = None
        if line.startswith(GGA_PREFIXES):
            self.last_gga_sentence = line # Keep track of latest position for NTRIP
            gga_data = parse_gga(line)
            if gga_data is None:
                return

            self.gps_data.update(gga_data)
            self.has_gga = True
            if self.has_gga and self.has_rmc:
                gps_data = dict(self.gps_data)
                self.has_gga = False
                self.has_rmc = False

        elif line.startswith(RMC_PREFIXES):
            rmc_data = parse_rmc(line)
            if rmc_data is None:
                return

            self.gps_data.update(rmc_data)
            self.has_rmc = True
            if self.has_gga and self.has_rmc:
                gps_data = dict(self.gps_data)
                self.has_gga = False
                self.has_rmc = False

        if gps_data is not None:
            self.publish_gps_data(gps_data)

    def ntrip_client_thread(self):
        """Connects to NTRIP caster, receives RTCM chunks, and injects them into the GPS serial."""
        # 1. Read RTCM from server and push down to serial
        try:
            data = self.sock.recv(1024)
            if not data:
                self.logging.warn("NTRIP server closed connection abruptly.")
            else:
                self.serial_port.write(data)
        except socket.timeout:
            pass

        # 2. Push GGA to caster every 5 seconds so VRS (Virtual Reference Station) works
        if self.last_gga_sentence and (time.time() - self.last_gga_sent_time > 5.0):
            send_line = self.last_gga_sentence + "\r\n"
            try:
                self.sock.send(send_line.encode())
                last_gga_sent_time = time.time()
            except Exception as e:
                self.logging.error(f"Failed to transmit GGA to NTRIP: {e}")

        time.sleep(0.01)

    def publish_gps_data(self, gps_data):
        if not gps_data["fix"]:
            self.logging.warning(f"Waiting for fix ({round(time.time() - self.last_fix)}s)", throttle_duration_sec=5)
            return

        msg = String()
        msg.data = json.dumps(
            {
                "lat": gps_data["lat"],
                "lon": gps_data["lon"],
                "track_angle": gps_data["heading"],
                "velocity": gps_data["speed"],
                "fix_type": gps_data["fix_text"],
                "rtk_data": gps_data["rtk_data"],
                "satellites": gps_data["satellites"],
                "hdop": gps_data["hdop"],
            }
        )

        self.pub.publish(msg)
        self.logging.info(str(msg.data))
        self.last_fix = time.time()


def main(args=None):
    if "ROS_LOG_DIR_BASE" in os.environ:
        os.environ["ROS_LOG_DIR"] = os.environ["ROS_LOG_DIR_BASE"] + "/rtk"

    rclpy.init(args=args)
    rtk = RTK()
    rclpy.spin(rtk)

if __name__ == "__main__":
    main()