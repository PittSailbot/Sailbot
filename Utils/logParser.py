import argparse
import subprocess
import sys
import time
from threading import Thread

RUN_COMMAND   = ""
# Script assumes chat server is launched with the command:
#     ./chat_server -id [id]
# where [id] is the ID of the server (an integer from 1-5)


def get_args(argv):
    parser = argparse.ArgumentParser(description="Script for filtering log files")
    
    parser.add_argument('filename', help="the path to the log file")
    parser.add_argument('-s', '--source', help="the source of the message")
    parser.add_argument('-ss', '--sourceSubstring', help="the arg is a substring of the source")
    parser.add_argument('-ps', "--printSources", action="store_true", help="prints all of the available sources")

    parser.add_argument('-d', '--debug', action='store_true', help="print the debug messages")
    parser.add_argument('-i', '--info', action='store_true', help="print the info messages")
    parser.add_argument('-w', '--warning', action='store_true', help="print the warning messages")
    parser.add_argument('-e', '--error', action='store_true', help="print the error messages")
    parser.add_argument('-f', '--fatal', action='store_true', help="print the fatal messages")
    parser.add_argument('-u', '--unset', action='store_true', help="print the unset messages")
    parser.add_argument('-a', '--all', action='store_true', help="print all the messages")

    return parser.parse_args()

def printFile(args):

    with open(args.filename) as f:
        lines = f.readlines()
        
        sources = set()

        for line in lines:
            source = "None"
            if len(line.split(' ')) > 2:
                source = line.split(" ")[2][1:-2]

            if args.printSources and args.none:
                sources.add(source)
                continue

            if args.source and (args.source.upper() != source.upper()):
                continue

            if args.sourceSubstring and (args.sourceSubstring.upper() not in source.upper()):
                continue

            found = False
            if line.upper().startswith('[DEBUG') and args.debug or args.all:
                print(line)
                found = True
            elif line.upper().startswith('[INFO') and args.info or args.all:
                print(line)
                found = True
            elif line.upper().startswith('[WARN') and args.warning or args.all:
                print(line)
                found = True
            elif line.upper().startswith('[ERROR') and args.error or args.all:
                print(line)
                found = True

            if args.printSources and found == True:
                sources.add(source)

        if args.printSources:
            if len(sources) == 0:
                print("NONE FOUND")
            else:
                print(sources)

def main(argv):
    args = get_args(argv)
    args.none = False
    if (not args.debug and
            not args.info and 
            not args.warning and
            not args.error and 
            not args.fatal and 
            not args.unset and
            not args.all):
        if args.printSources:
            args.none = True
        else:
            args.all = True

    printFile(args)

if __name__ == "__main__":
    main(sys.argv[1:])