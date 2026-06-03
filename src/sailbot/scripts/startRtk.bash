#!/usr/bin/env bash
set -Eeuo pipefail

# Ensures the Waveshare LG290P RTK rover demo exists, then runs main.py
# in the foreground so ROS launch/systemd can supervise it.

RTK_URL="${RTK_URL:-https://files.waveshare.com/wiki/LG290P-GNSS-RTK-Module/Demo/LG290P-GNSS-RTK-Module-Demo.zip}"
RTK_INSTALL_PARENT="${RTK_INSTALL_PARENT:-/opt}"
RTK_DEMO_DIR="${RTK_DEMO_DIR:-${RTK_INSTALL_PARENT}/LG290P-GNSS-RTK-Module-Demo}"
RTK_WORKDIR="${RTK_WORKDIR:-${RTK_DEMO_DIR}/Raspberry_Pi/Python/RTK_Rover}"
RTK_ENTRY="${RTK_ENTRY:-main.py}"
RTK_USE_SUDO="${RTK_USE_SUDO:-true}"

# 按需用 sudo 执行命令；root 或关闭 RTK_USE_SUDO 时直接执行。
sudo_cmd() {
    if [[ "${EUID}" -eq 0 ]] || [[ "${RTK_USE_SUDO}" != "true" ]]; then
        "$@"
    else
        sudo "$@"
    fi
}

# 检查某个系统命令是否存在，不存在就中止脚本。
ensure_command() {
    if ! command -v "$1" >/dev/null 2>&1; then
        echo "Missing required command: $1" >&2
        exit 1
    fi
}

# 确保 Waveshare RTK demo 已下载并解压到目标目录。
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

# 脚本主流程：准备 RTK demo，然后以前台进程运行 main.py。
main() {
    ensure_rtk_demo

    echo "Starting Waveshare RTK rover command-line program..."
    cd "${RTK_WORKDIR}"
    if [[ "${EUID}" -eq 0 ]] || [[ "${RTK_USE_SUDO}" != "true" ]]; then
        exec python3 "${RTK_ENTRY}"
    fi

    sudo -v
    exec sudo python3 "${RTK_ENTRY}"
}

main "$@"
