import argparse
import os
import platform
import subprocess
import sys
import time
from threading import Thread

RUN_COMMAND = ""
# Script assumes chat server is launched with the command:
#     ./chat_server -id [id]
# where [id] is the ID of the server (an integer from 1-5)

NETWORK_NAME = "SailNet"
IP_BASE = "172.30.100."

OS_PI = "Raspberry Pi"
OS_LINUX = "Not Pi"
OS_WINDOWS = "Windows"

WARNING = "\033[93m"
FAIL = "\033[91m"
ENDC = "\033[0m"


def get_os():
    system = platform.system()
    if system == "Linux":
        # Check if it's a Raspberry Pi or an Ubuntu desktop
        release = platform.release()
        if "raspi" in release or "raspberry" in release or "rpi" in release:
            return OS_PI
        else:
            return OS_LINUX
    else:
        return OS_WINDOWS


def line_exists_in_file(file_path, text_to_find):
    try:
        with open(file_path, "r") as file:
            for line in file:
                if text_to_find in line:
                    return True
        return False
    except FileNotFoundError:
        print(f"{WARNING}Warning: File: {file_path} not found.{ENDC}")
        return False


def get_args(argv):
    parser = argparse.ArgumentParser(description="Script for testing sailbot code")
    subparsers = parser.add_subparsers(required=True, help="available commands")

    parser_create = subparsers.add_parser("create", description="Initialize Docker image")
    parser_create.add_argument("--name", help="name the docker image")
    parser_create.set_defaults(func=init_image)

    parser_net = subparsers.add_parser("initNet", description="Initialize Docker network")
    parser_net.set_defaults(func=init_network)

    parser_init = subparsers.add_parser("run", description="Initialize Docker container")
    parser_init.add_argument("-nd", action="store_true", help="prevent the docker container from being deleted when closed")
    parser_init.add_argument("--copy", action="store_true", help="copy the sailbot folder rather than mount it")
    parser_init.add_argument("--no-connect", action="store_true", help="Start the container without connecting to it via terminal")
    # parser_init.add_argument('--name', help="name the docker container")
    # parser_init.add_argument('--id', help="id of the docker container, used for ip address assignment")
    parser_init.add_argument("--image", help="name the docker image to use")

    parser_init.set_defaults(func=init_container)

    parser_connect = subparsers.add_parser("connect", description="connect to Docker container")
    # parser_connect.add_argument('--name', help="name the docker container")
    # parser_connect.add_argument('--id', help="id of the docker container, used for ip address assignment")
    parser_connect.set_defaults(func=connect_container)

    parser_vnc = subparsers.add_parser("runVNC", description="Initialize Docker container with VNC")
    parser_vnc.set_defaults(func=init_vnc)

    parser_rm = subparsers.add_parser("rm", description="Remove Docker containers")
    parser_rm.add_argument("--name", help="name the docker container")
    parser_rm.add_argument("--id", help="id of the docker container, used for ip address assignment")
    parser_rm.set_defaults(func=cleanup)

    return parser.parse_args()


def container_ip(container_id):
    return IP_BASE + str(100 + container_id)


def container_ports(container_id):
    return str(5000 + container_id) + ":" + str(5000 + container_id)


# Do all initialization
def init_image(args):
    name = args.name if args.name else "sailbot"
    cmd_str = f'docker build . -t "{name}"'
    subprocess.run(cmd_str, shell=True)

    init_network(args)


# Create bridge network
def init_network(args):
    cmd_str = "docker network create --driver bridge --subnet {sub} --gateway {gate} {net}".format(sub=IP_BASE + "0/24", gate=IP_BASE + "1", net=NETWORK_NAME)

    subprocess.run(cmd_str, shell=True)


# Create server containers and attach them to bridge network
def init_container(args):
    name = "sailbot_client"  # args.name if args.name else 'sailbot_client'
    image = args.image if args.image else "sailbot"
    id = 1  # int(args.id) if args.id else 1
    ports = container_ports(id)

    name = f'"{name}_{id}"'

    if get_os() != OS_WINDOWS and not line_exists_in_file("/etc/udev/rules.d/99-serial.rules", 'KERNEL=="ttyACM[0-9]*",MODE="0666"'):
        print(
            f"{WARNING}You do not appear to have rules configured for allowing access to the USB, instructions for setting this up can be found here: https://www.losant.com/blog/how-to-access-serial-devices-in-docker{ENDC}"
        )

    # cmd_str = F"docker create -p 5000:5000 -t -it --name {name} sailbot"
    privileged = "--privileged"  # if get_os() == OS_PI or use_privileged_desktop else ""
    # cmd_str = F"docker create -t -it  --network {NETWORK_NAME} --ip {container_ip(id)} -p {ports} --name {name} {image}"
    enviroment_var = ("-e IS_PI_DOCKER=true" if get_os() == OS_PI else "-e IS_DOCKER=true") + f" -e PORTS={ports}"
    volume = "" if args.copy else f"-v {os.getcwd()}:/workspace"
    volume += "" if get_os() == OS_WINDOWS else " -v /dev:/dev"
    network = "" if get_os() == OS_WINDOWS else "-t --network host"
    cmd_str = f"docker create {enviroment_var} {privileged} {volume} -it {network} -p {ports} --name {name} {image} "

    subprocess.run(cmd_str, shell=True)
    # subprocess.Popen(cmd_str, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    cmd_str = f"docker start {name}"
    subprocess.run(cmd_str, shell=True)

    if args.copy:
        cmd_str = f"docker cp ./ {name}:/workspace/"
        subprocess.run(cmd_str, shell=True)
        if get_os() == OS_WINDOWS:
            cmd_str = f"docker exec {name} dos2unix utils/dos2unixConvert.bash"
            subprocess.run(cmd_str, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

            cmd_str = f"docker exec {name} bash /workspace/utils/dos2unixConvert.bash"
            subprocess.run(cmd_str, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    elif get_os() == OS_WINDOWS:
        print(f"{WARNING}You appear to be using windows, it is recommended to use the --copy option so that files can be converted from windows to linux format. Without this you may get errors{ENDC}")

    print("type: 'exit' to close connection")

    if not args.no_connect:
        cmd_str = f"docker exec -e PORTS={ports} -it {name} bash"
        subprocess.run(cmd_str, shell=True)

        if not args.nd:
            cleanup(args)


def connect_container(args):
    name = "sailbot_client"  # args.name if args.name else 'sailbot_client'
    id = 1  # int(args.id) if args.id else 1

    name = f'"{name}_{id}"'

    print("type: 'exit' to close connection")

    cmd_str = f"docker exec -it {name} bash"
    subprocess.run(cmd_str, shell=True)


def init_vnc(args):
    cmd_str = "docker run -it -p 6080:80 -v /new_folder --name sailbot_client sailbot"
    subprocess.run(cmd_str, shell=True)
    print("vnc available at: http://127.0.0.1:6080/")


# Stop all containers, then remove containers and network
def cleanup(args):
    name = "sailbot_client"  # args.name if args.name else 'sailbot_client'
    id = 1  # int(args.id) if args.id else 1

    name = f'"{name}_{id}"'

    cmd_str = f"docker kill {name}"
    subprocess.run(cmd_str, shell=True)

    cmd_str = f"docker rm {name}"
    subprocess.run(cmd_str, shell=True)


def main(argv):
    args = get_args(argv)
    args.func(args)


if __name__ == "__main__":
    main(sys.argv[1:])
