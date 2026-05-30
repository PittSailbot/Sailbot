#!/usr/bin/env bash
set -Eeuo pipefail

# Ensures the Waveshare LG290P RTK rover demo exists, then runs web_rtk.py
# in the foreground so ROS launch/systemd can supervise it.

RTK_URL="${RTK_URL:-https://files.waveshare.com/wiki/LG290P-GNSS-RTK-Module/Demo/LG290P-GNSS-RTK-Module-Demo.zip}"
RTK_INSTALL_PARENT="${RTK_INSTALL_PARENT:-/opt}"
RTK_DEMO_DIR="${RTK_DEMO_DIR:-${RTK_INSTALL_PARENT}/LG290P-GNSS-RTK-Module-Demo}"
RTK_WORKDIR="${RTK_WORKDIR:-${RTK_DEMO_DIR}/Raspberry_Pi/Python/RTK_Rover}"
RTK_ENTRY="${RTK_ENTRY:-web_rtk.py}"
RTK_USE_SUDO="${RTK_USE_SUDO:-true}"

sudo_cmd() {
    if [[ "${EUID}" -eq 0 ]] || [[ "${RTK_USE_SUDO}" != "true" ]]; then
        "$@"
    else
        sudo "$@"
    fi
}

ensure_command() {
    if ! command -v "$1" >/dev/null 2>&1; then
        echo "Missing required command: $1" >&2
        exit 1
    fi
}

ensure_rtk_demo() {
    if [[ -f "${RTK_WORKDIR}/${RTK_ENTRY}" ]]; then
        return
    fi

    ensure_command wget
    ensure_command unzip

    tmp_zip="/tmp/LG290P-GNSS-RTK-Module-Demo.zip"
    echo "RTK demo not found. Downloading Waveshare LG290P demo..."
    wget -O "${tmp_zip}" "${RTK_URL}"

    echo "Installing RTK demo into ${RTK_INSTALL_PARENT}..."
    sudo_cmd mkdir -p "${RTK_INSTALL_PARENT}"
    sudo_cmd unzip -o "${tmp_zip}" -d "${RTK_INSTALL_PARENT}"

    if [[ ! -f "${RTK_WORKDIR}/${RTK_ENTRY}" ]]; then
        echo "Could not find ${RTK_WORKDIR}/${RTK_ENTRY} after install." >&2
        exit 1
    fi
}

main() {
    ensure_rtk_demo

    echo "Starting Waveshare RTK rover server..."
    cd "${RTK_WORKDIR}"
    if [[ "${EUID}" -eq 0 ]] || [[ "${RTK_USE_SUDO}" != "true" ]]; then
        exec python3 "${RTK_ENTRY}"
    fi

    sudo -v
    exec sudo python3 "${RTK_ENTRY}"
}

main "$@"
