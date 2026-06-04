# RTK demo file from Waveshare for your reference. Delete this after you can read from the sensor with rtk.py

import serial
import socket
import base64
import time
import threading
import glob

# --- Configuration ---
GPS_BAUD = 460800

NTRIP_HOST = "114.111.30.88"
NTRIP_PORT = 8003
MOUNTPOINT = "RTCM33GRCEJ"
NTRIP_USER = "qx5946"
NTRIP_PASS = "23832134"

# --- Global Variables ---
last_gga = ""
last_gga_time = 0

gps_lock = threading.Lock()  # Protect GPS data
gps_data = {
    "fix": False,
    "satellites": 0,
    "hdop": 0.0,
    "latitude": 0.0,
    "longitude": 0.0,
    "speed": 0.0,
    "heading": 0.0,
    "fix_type": 0,
}

has_gga = False
has_rmc = False

nmea_buffer = ""

# --- Auto-detect GPS port ---
def find_gps_port(baudrate=GPS_BAUD, timeout=0.5):
    candidates = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyAMA*") + glob.glob("/dev/serial*") + glob.glob("/dev/ttyACM*")
    for port in candidates:
        try:
            with serial.Serial(port, baudrate=baudrate, timeout=timeout) as ser:
                time.sleep(1)  # Wait for serial output
                data = ser.read(500).decode(errors="ignore")
                if "$GNGGA" in data or "$GNRMC" in data:
                    print(f"[Auto Detect] GPS device found on {port}")
                    return port
        except Exception:
            continue
    # If auto-detection fails, try GPIO serial port like /dev/serial0
    gpio_port = "/dev/serial0"
    try:
        with serial.Serial(gpio_port, baudrate=baudrate, timeout=timeout) as ser:
            time.sleep(1)
            data = ser.read(500).decode(errors="ignore")
            if "$GNGGA" in data or "$GNRMC" in data:
                print(f"[Fallback] GPS device found on GPIO port {gpio_port}")
                return gpio_port
            else:
                print(f"❌ No GPS data found on fallback GPIO port {gpio_port}.")
    except Exception as e:
        print(f"❌ Could not open fallback GPIO port {gpio_port}: {e}")

    print("❌ No GPS device found automatically or on GPIO fallback.")
    return None

# --- Utility Functions ---
def base64_encode(data: str) -> str:
    return base64.b64encode(data.encode()).decode()

def convert_to_decimal_degree(val: str, direction: str) -> float:
    if len(val) < 6:
        return 0.0
    if direction in ("N", "S"):
        deg = float(val[0:2])
        minutes = float(val[2:])
    elif direction in ("E", "W"):
        deg = float(val[0:3])
        minutes = float(val[3:])
    else:
        return 0.0
    dec = deg + minutes / 60.0
    if direction in ("S", "W"):
        dec = -dec
    return dec

def split_nmea(sentence: str):
    return sentence.strip().split(',')

# --- NMEA Parsing ---
def parse_gga(sentence):
    global last_gga, gps_data, has_gga
    parts = split_nmea(sentence)
    if len(parts) < 15:
        return
    try:
        fix_type = int(parts[6])
        satellites = int(parts[7])
        hdop = float(parts[8])
        latitude = convert_to_decimal_degree(parts[2], parts[3])
        longitude = convert_to_decimal_degree(parts[4], parts[5])
    except (ValueError, IndexError):
        return
    with gps_lock:
        gps_data.update({
            "fix_type": fix_type,
            "fix": fix_type > 0,
            "satellites": satellites,
            "hdop": hdop,
            "latitude": latitude,
            "longitude": longitude
        })
    last_gga = sentence
    has_gga = True

def parse_rmc(sentence):
    global gps_data, has_rmc
    parts = split_nmea(sentence)
    if len(parts) < 12:
        return
    status = parts[2]
    if status == 'A':  # valid
        try:
            speed = float(parts[7])
            heading = float(parts[8])
        except ValueError:
            speed = 0.0
            heading = 0.0
    else:
        speed = 0.0
        heading = 0.0
    with gps_lock:
        gps_data.update({"speed": speed, "heading": heading})
    has_rmc = True

# --- Print GPS Data ---
def print_gps_data():
    with gps_lock:
        fix = gps_data["fix"]
        fix_type = gps_data["fix_type"]
        satellites = gps_data["satellites"]
        hdop = gps_data["hdop"]
        latitude = gps_data["latitude"]
        longitude = gps_data["longitude"]
        speed = gps_data["speed"]
        heading = gps_data["heading"]

    print("-------------")
    print(f"Positioning Status: {'Yes' if fix else 'No'}")
    print("RTK Status: ", end='')
    if fix_type == 0:
        print("No Fix")
    elif fix_type == 1:
        print("GPS SPS Mode")
    elif fix_type == 2:
        print("DGNSS (SBAS/DGPS)")
    elif fix_type == 3:
        print("GPS PPS Fix")
    elif fix_type == 4:
        print("Fixed RTK")
    elif fix_type == 5:
        print("Float RTK")
    else:
        print(f"Unknown ({fix_type})")

    print("RTK Data: " + ("RECEIVED" if fix_type in (4,5) else "NOT RECEIVED"))
    print(f"Satellites Number: {satellites}")
    print(f"HDOP: {hdop:.2f}")
    print(f"Latitude: {latitude:.6f}")
    print(f"Longitude: {longitude:.6f}")
    print(f"Speed (knots): {speed:.3f}")
    print(f"Heading (degrees): {heading:.2f}")
    print("-------------")

# --- NTRIP Connection Thread ---
def ntrip_thread(serial_port):
    global last_gga, last_gga_time

    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((NTRIP_HOST, NTRIP_PORT))
            auth_str = f"{NTRIP_USER}:{NTRIP_PASS}"
            auth_enc = base64_encode(auth_str)
            request = (
                f"GET /{MOUNTPOINT} HTTP/1.1\r\n"
                f"User-Agent: NTRIP client\r\n"
                f"Accept: */*\r\n"
                f"Connection: keep-alive\r\n"
                f"Authorization: Basic {auth_enc}\r\n"
                f"\r\n"
            )
            sock.send(request.encode())
            print(f"Connected to NTRIP {NTRIP_HOST}:{NTRIP_PORT}")

            sock.settimeout(0.1)

            while True:
                # Receive RTCM data and write to GPS serial port
                try:
                    data = sock.recv(1024)
                    if not data:
                        print("NTRIP server closed connection")
                        break
                    serial_port.write(data)
                except socket.timeout:
                    pass

                # Send GGA every 5 seconds
                if last_gga and (time.time() - last_gga_time > 5):
                    send_line = last_gga + "\r\n"
                    try:
                        sock.send(send_line.encode())
                        print(f"[GGA sent to NTRIP] {last_gga}")
                        last_gga_time = time.time()
                    except Exception as e:
                        print(f"Send GGA failed: {e}")
                        break

                time.sleep(0.01)

        except Exception as e:
            print(f"NTRIP connection error: {e}")
            try:
                sock.close()
            except:
                pass
            print("Reconnecting NTRIP in 1 second...")
            time.sleep(1)

# --- GPS Reading Thread ---
def gps_thread():
    global nmea_buffer, has_gga, has_rmc

    ser = serial.Serial(GPS_PORT, GPS_BAUD, timeout=0.1)

    while True:
        try:
            c = ser.read().decode(errors='ignore')
            if c == '\n':
                line = nmea_buffer.strip()
                if line.startswith("$GNGGA"):
                    parse_gga(line)
                elif line.startswith("$GNRMC"):
                    parse_rmc(line)
                nmea_buffer = ""
            elif c != '\r':
                nmea_buffer += c

            if has_gga and has_rmc:
                print_gps_data()
                has_gga = False
                has_rmc = False

        except Exception as e:
            print(f"GPS read error: {e}")
            time.sleep(0.1)

# --- Main Program ---
def main():
    global GPS_PORT

    GPS_PORT = find_gps_port(GPS_BAUD)

    if GPS_PORT is None:
        print("Exiting due to missing GPS port.")
        return

    try:
        ser = serial.Serial(GPS_PORT, GPS_BAUD, timeout=0.1)
    except Exception as e:
        print(f"Failed to open GPS serial port: {e}")
        return

    # Start NTRIP thread
    threading.Thread(target=ntrip_thread, args=(ser,), daemon=True).start()

    # Start GPS reading thread (blocks main thread)
    gps_thread()

if __name__ == "__main__":
    main()