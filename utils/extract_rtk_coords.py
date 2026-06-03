#!/usr/bin/env python3
import argparse
import json
import re
import sys


def read_source(source):
    """读取输入源：'-' 表示 stdin，否则按命令行输出文本文件读取。"""
    if source == "-":
        return sys.stdin.read()

    with open(source, encoding="utf-8") as file:
        return file.read()


def parse_float_field(text, label):
    """从命令行输出中按标签提取一个浮点数字段。"""
    match = re.search(rf"^{re.escape(label)}\s*:\s*([-+]?\d+(?:\.\d+)?)\s*$", text, re.MULTILINE)
    return float(match.group(1)) if match else None


def parse_int_field(text, label):
    """从命令行输出中按标签提取一个整数字段。"""
    match = re.search(rf"^{re.escape(label)}\s*:\s*([-+]?\d+)\s*$", text, re.MULTILINE)
    return int(match.group(1)) if match else None


def parse_text_field(text, label):
    """从命令行输出中按标签提取一个文本字段。"""
    match = re.search(rf"^{re.escape(label)}\s*:\s*(.+?)\s*$", text, re.MULTILINE)
    return match.group(1).strip() if match else None


def extract_from_cli_text(text):
    """从 Waveshare RTK_Rover/main.py 的终端输出中提取最新一组 Sailbot GPS JSON。"""
    blocks = [block for block in re.split(r"^-{5,}\s*$", text, flags=re.MULTILINE) if block.strip()]
    candidates = blocks if blocks else [text]

    for block in reversed(candidates):
        lat = parse_float_field(block, "Latitude")
        lon = parse_float_field(block, "Longitude")
        if lat is None or lon is None:
            continue

        speed = parse_float_field(block, "Speed (knots)")
        heading = parse_float_field(block, "Heading (degrees)")
        hdop = parse_float_field(block, "HDOP")
        fix_text = parse_text_field(block, "RTK Status")

        return {
            "lat": lat,
            "lon": lon,
            "track_angle": heading if heading is not None else 0.0,
            "velocity": speed if speed is not None else 0.0,
            "fix_type": fix_text,
            "positioning_status": parse_text_field(block, "Positioning Status"),
            "rtk_data": parse_text_field(block, "RTK Data"),
            "satellites": parse_int_field(block, "Satellites Number"),
            "hdop": hdop,
        }

    raise ValueError("Could not find Latitude/Longitude in command output.")


def extract_from_cli_output(source):
    """读取命令行输出文件或 stdin，并解析成 Sailbot GPS JSON。"""
    return extract_from_cli_text(read_source(source))


def stream_cli_output(input_stream):
    """持续读取 main.py 输出，每遇到完整坐标块就打印一行 JSON。"""
    buffer = []
    in_block = False

    for line in input_stream:
        if re.match(r"^-{5,}\s*$", line):
            if in_block and buffer:
                try:
                    print(json.dumps(extract_from_cli_text("".join(buffer))), flush=True)
                except ValueError:
                    pass
                buffer = []
            in_block = True
            continue

        if in_block:
            buffer.append(line)


def main():
    """解析 main.py 的终端输出，并输出一行 Sailbot GPS JSON。"""
    parser = argparse.ArgumentParser(description="Extract RTK/GNSS coordinates as Sailbot JSON.")
    parser.add_argument("source", help="Waveshare RTK_Rover/main.py output file path, or '-' for stdin")
    parser.add_argument("--stream", action="store_true", help="With SOURCE='-', emit JSON for each complete output block")
    args = parser.parse_args()

    if args.stream:
        if args.source != "-":
            parser.error("--stream expects SOURCE to be '-'")
        stream_cli_output(sys.stdin)
        return

    print(json.dumps(extract_from_cli_output(args.source)))


if __name__ == "__main__":
    main()
