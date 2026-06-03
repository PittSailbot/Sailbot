#!/usr/bin/env bash
set -Eeuo pipefail

# Starts the Waveshare LG290P RTK rover command-line program, then runs Sailbot.
#
# First run:
#   - downloads and unzips the Waveshare demo if it is not already installed
# Later runs:
#   - reuses the existing install and starts main.py directly
#
# Common usage from the ROS workspace/repo root:
#   bash utils/runWithRtk.bash
#   bash utils/runWithRtk.bash -- ros2 launch sailbot websiteTest.launch.py

RTK_URL="${RTK_URL:-https://files.waveshare.com/wiki/LG290P-GNSS-RTK-Module/Demo/LG290P-GNSS-RTK-Module-Demo.zip}"
RTK_INSTALL_PARENT="${RTK_INSTALL_PARENT:-/opt}"
RTK_DEMO_DIR="${RTK_DEMO_DIR:-${RTK_INSTALL_PARENT}/LG290P-GNSS-RTK-Module-Demo}"
RTK_WORKDIR="${RTK_WORKDIR:-${RTK_DEMO_DIR}/Raspberry_Pi/Python/RTK_Rover}"
RTK_ENTRY="${RTK_ENTRY:-main.py}"
RTK_USE_SUDO="${RTK_USE_SUDO:-true}"

ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO}/setup.bash}"
SAILBOT_WS="${SAILBOT_WS:-$(pwd)}"
SAILBOT_SETUP="${SAILBOT_SETUP:-${SAILBOT_WS}/install/local_setup.bash}"
SAILBOT_DEFAULT_LAUNCH="${SAILBOT_DEFAULT_LAUNCH:-boat_all.launch.py}"

RTK_PID=""

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

# 后台启动 Waveshare RTK rover 命令行程序，并记录进程号方便退出时关闭。
start_rtk() {
    echo "Starting Waveshare RTK rover command-line program..."
    pushd "${RTK_WORKDIR}" >/dev/null
    if [[ "${EUID}" -eq 0 ]] || [[ "${RTK_USE_SUDO}" != "true" ]]; then
        python3 "${RTK_ENTRY}" &
    else
        sudo -v
        sudo python3 "${RTK_ENTRY}" &
    fi
    RTK_PID="$!"
    popd >/dev/null
}

# 如果本脚本启动过 RTK 服务，退出时把它停掉。
stop_rtk() {
    if [[ -n "${RTK_PID}" ]] && kill -0 "${RTK_PID}" >/dev/null 2>&1; then
        echo "Stopping Waveshare RTK rover server..."
        if [[ "${EUID}" -eq 0 ]] || [[ "${RTK_USE_SUDO}" != "true" ]]; then
            kill "${RTK_PID}" >/dev/null 2>&1 || true
        else
            sudo kill "${RTK_PID}" >/dev/null 2>&1 || true
        fi
    fi
}

# 加载 ROS/Sailbot 环境，然后运行传入命令或默认 boat_all launch。
run_sailbot() {
    if [[ ! -f "${ROS_SETUP}" ]]; then
        echo "ROS setup file not found: ${ROS_SETUP}" >&2
        exit 1
    fi

    if [[ ! -f "${SAILBOT_SETUP}" ]]; then
        echo "Sailbot setup file not found: ${SAILBOT_SETUP}" >&2
        echo "Build first, for example: bash compileDocker.sh" >&2
        exit 1
    fi

    source "${ROS_SETUP}"
    source "${SAILBOT_SETUP}"

    if [[ "$#" -gt 0 ]]; then
        "$@"
    else
        ros2 launch sailbot "${SAILBOT_DEFAULT_LAUNCH}" start_rtk:=false
    fi
}

# 脚本主流程：处理参数、安装/启动 RTK、再启动 Sailbot。
main() {
    if [[ "${1:-}" == "--" ]]; then
        shift
    fi

    trap stop_rtk EXIT INT TERM

    ensure_rtk_demo
    start_rtk
    run_sailbot "$@"
}

main "$@"
