import argparse
import subprocess
import sys
import time
from threading import Thread

RUN_COMMAND   = ""
# Script assumes chat server is launched with the command:
#     ./chat_server -id [id]
# where [id] is the ID of the server (an integer from 1-5)

IMAGE_NAME = "sailbot"

def get_args(argv):
    parser = argparse.ArgumentParser(description="Script for testing sailbot code")
    subparsers = parser.add_subparsers(required=True, help='available commands')

    parser_create = subparsers.add_parser('create', description='Initialize Docker image')
    parser_create.set_defaults(func=init_image)

    parser_init = subparsers.add_parser('run', description='Initialize Docker container')
    parser_init.add_argument('-nd', action='store_true', help="prevent the docker container from being deleted when closed")
    parser_init.add_argument('--name', help="name the docker container")
    parser_init.set_defaults(func=init_container)

    parser_connect = subparsers.add_parser('connect', description='connect to Docker container')
    parser_connect.set_defaults(func=connect_container)

    parser_vnc = subparsers.add_parser('runVNC', description='Initialize Docker container with VNC')
    parser_vnc.set_defaults(func=init_vnc)

    parser_rm = subparsers.add_parser('rm', description='Remove Docker containers')
    parser_rm.add_argument('--name', help="name the docker container")
    parser_rm.set_defaults(func=cleanup)

    return parser.parse_args()

# Do all initialization
def init_image(args):
    cmd_str = 'docker build . -t "sailbot"'
    subprocess.run(cmd_str, shell=True)

# Create server containers and attach them to bridge network
def init_container(args):
    name = args.name if args.name else 'sailbot_client'

    cmd_str = F"docker create -p 5000:5000 -t -it --name {name} sailbot"
    subprocess.run(cmd_str, shell=True)
    #subprocess.Popen(cmd_str, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    cmd_str = F"docker start {name}"
    subprocess.run(cmd_str, shell=True)

    cmd_str = F"docker cp ./ {name}:/workspace/"
    subprocess.run(cmd_str, shell=True)

    print("type: 'exit' to close connection")

    cmd_str = F"docker exec -it {name} bash"
    subprocess.run(cmd_str, shell=True)

    if not args.nd:
        cleanup(args)

def connect_container(args):
    print("type: 'exit' to close connection")

    cmd_str = "docker exec -it sailbot_client bash"
    subprocess.run(cmd_str, shell=True)

def init_vnc(args):
    cmd_str = "docker run -it -p 6080:80 -v /new_folder --name sailbot_client sailbot"
    subprocess.run(cmd_str, shell=True)
    print("vnc available at: http://127.0.0.1:6080/")

# Stop all containers, then remove containers and network
def cleanup(args):
    name = args.name if args.name else 'sailbot_client'

    cmd_str = F"docker kill {name}"
    subprocess.run(cmd_str, shell=True)

    cmd_str = F"docker rm {name}"
    subprocess.run(cmd_str, shell=True)

def main(argv):
    args = get_args(argv)
    args.func(args)

if __name__ == "__main__":
    main(sys.argv[1:])
