#!/usr/bin/env python3
import argparse
import json
import re
import urllib.request
from html.parser import HTMLParser


class RtkStatusParser(HTMLParser):
    def __init__(self):
        super().__init__()
        self._capture_id = None
        self._current_id = None
        self.values = {}

    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs)
        element_id = attrs.get("id")
        if element_id in {"lat_text", "lon_text", "fix_text", "gga_time"}:
            self._capture_id = element_id
            self._current_id = element_id

    def handle_data(self, data):
        if self._capture_id:
            self.values[self._capture_id] = data.strip()

    def handle_endtag(self, tag):
        if self._current_id:
            self._capture_id = None
            self._current_id = None


def read_source(source):
    if source.startswith(("http://", "https://")):
        with urllib.request.urlopen(source, timeout=2.0) as response:
            return response.read().decode("utf-8")

    with open(source, encoding="utf-8") as file:
        return file.read()


def extract_from_status_api(base_url):
    url = base_url.rstrip("/") + "/api/status"
    with urllib.request.urlopen(url, timeout=2.0) as response:
        status = json.loads(response.read().decode("utf-8"))

    return {
        "lat": float(status["lat"]),
        "lon": float(status["lon"]),
        "track_angle": 0.0,
        "velocity": 0.0,
        "fix_type": status.get("fix_text"),
        "last_gga_time": status.get("last_gga_time"),
    }


def extract_from_html(source):
    html = read_source(source)
    parser = RtkStatusParser()
    parser.feed(html)

    lat = parser.values.get("lat_text")
    lon = parser.values.get("lon_text")

    if lat is None or lon is None:
        lat_match = re.search(r'id=["\']lat_text["\'][^>]*>\s*([^<]+)', html)
        lon_match = re.search(r'id=["\']lon_text["\'][^>]*>\s*([^<]+)', html)
        if not lat_match or not lon_match:
            raise ValueError("Could not find lat_text/lon_text in HTML.")
        lat = lat_match.group(1).strip()
        lon = lon_match.group(1).strip()

    return {
        "lat": float(lat),
        "lon": float(lon),
        "track_angle": 0.0,
        "velocity": 0.0,
        "fix_type": parser.values.get("fix_text"),
        "last_gga_time": parser.values.get("gga_time"),
    }


def main():
    parser = argparse.ArgumentParser(description="Extract RTK/GNSS coordinates as Sailbot JSON.")
    parser.add_argument("source", help="HTML file path, page URL, or RTK tool base URL")
    parser.add_argument("--api", action="store_true", help="Read live JSON from SOURCE/api/status")
    args = parser.parse_args()

    data = extract_from_status_api(args.source) if args.api else extract_from_html(args.source)
    print(json.dumps(data))


if __name__ == "__main__":
    main()
