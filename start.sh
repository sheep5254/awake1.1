#!/usr/bin/env bash
#
# 一键编译 + 启动自瞄
#
#   ./start.sh                 # 编译（增量）后启动：standard omni true
#   ./start.sh omni false      # 自定义参数（robot / debug）
#   ./start.sh --run-only      # 不编译，直接启动已编好的程序
#   ./start.sh --help
#
# 说明：
#   - 编译目录是 build-local（和 run.sh 一致），产物在 build-local/bin/standard
#   - 会自动 source 含 auto_aim_interfaces 的工作空间，保证 tracker/target 能发布
#   - 会自动导出 Fast DDS 的 UDP-only profile，避免 sudo/普通用户跨用户收不到话题
#   - 不要用 sudo 运行本脚本
#
set -uo pipefail

WORK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${AWAKENING_BUILD_DIR:-$WORK_DIR/build-local}"
CONFIG_DIR="$WORK_DIR/config"
RUN_PROGRAM="$BUILD_DIR/bin/standard"

RUN_ONLY=0
ARGS=()
for arg in "$@"; do
    case "$arg" in
        --run-only|-r) RUN_ONLY=1 ;;
        --help|-h)
            cat <<'USAGE'
用法: ./start.sh [--run-only] [robot] [debug]

  ./start.sh                 编译（增量）后启动: standard omni true
  ./start.sh omni false      自定义参数
  ./start.sh --run-only      不编译，直接启动已编好的程序
  ./start.sh --help          显示本帮助

编译目录: build-local（可用 AWAKENING_BUILD_DIR 覆盖）
可选环境变量: AWAKENING_ROS_WS=/path/to/ws（含 auto_aim_interfaces 的工作空间）
不要用 sudo 运行。
USAGE
            exit 0
            ;;
        *) ARGS+=("$arg") ;;
    esac
done
# 默认：omni 配置 + debug 模式
if [ ${#ARGS[@]} -eq 0 ]; then
    ARGS=(omni true)
fi

if [ "$EUID" -eq 0 ]; then
    echo "请不要用 sudo 运行本脚本（会产生 root 属主的编译产物，并影响 ROS 跨用户通信）" >&2
    exit 1
fi

# ---------------- 运行环境 ----------------
export VISION_ROOT="$WORK_DIR"
export MVCAM_SDK_PATH=/opt/MVS
export MVCAM_COMMON_RUNENV=/opt/MVS/lib
export MVCAM_GENICAM_CLPROTOCOL=/opt/MVS/lib/CLProtocol
export ALLUSERSPROFILE=/opt/MVS/MVFG
export LD_LIBRARY_PATH="/opt/MVS/lib/64:/opt/MVS/lib/32:$WORK_DIR/lib:${LD_LIBRARY_PATH:-}"

# Fast DDS 强制走 UDP：本程序如果以 root 跑，普通用户的 ros2 CLI / PlotJuggler 收不到数据
if [ -f "$CONFIG_DIR/fastdds_udp.xml" ] && [ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$CONFIG_DIR/fastdds_udp.xml"
fi

# ---------------- ROS 环境 + auto_aim_interfaces ----------------
set +u
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash

AUTO_AIM_PREFIX=""
if [ -n "${AWAKENING_ROS_WS:-}" ] && [ -f "$AWAKENING_ROS_WS/install/setup.bash" ]; then
    source "$AWAKENING_ROS_WS/install/setup.bash"
    AUTO_AIM_PREFIX="$AWAKENING_ROS_WS/install/auto_aim_interfaces"
else
    for base in "$HOME/Lin/pb2025_sentry_ws" "$HOME/Lin" "$HOME"; do
        if [ -f "$base/install/setup.bash" ] \
            && [ -d "$base/install/auto_aim_interfaces" ]; then
            source "$base/install/setup.bash"
            AUTO_AIM_PREFIX="$base/install/auto_aim_interfaces"
            break
        fi
    done
fi
set -u

if [ -n "$AUTO_AIM_PREFIX" ] && [ -d "$AUTO_AIM_PREFIX" ]; then
    echo "auto_aim_interfaces: $AUTO_AIM_PREFIX"
else
    AUTO_AIM_PREFIX=""
    echo "警告：没找到 auto_aim_interfaces，tracker/target 不会被发布。" >&2
    echo "      可先 source 你的工作空间，或 AWAKENING_ROS_WS=/path/to/ws ./start.sh" >&2
fi

# ---------------- 编译目录权限 ----------------
mkdir -p "$BUILD_DIR"
if [ ! -w "$BUILD_DIR" ]; then
    echo "build-local 不可写（之前用 sudo 编译过），正在改回当前用户..."
    local_user="$(id -un)"
    sudo chown -R "$local_user:$local_user" "$BUILD_DIR" || {
        echo "chown 失败，请手动执行：sudo chown -R \$USER:\$USER $BUILD_DIR" >&2
        exit 1
    }
fi

# ---------------- 编译 ----------------
if [ "$RUN_ONLY" -eq 0 ]; then
    CMAKE_ARGS=(
        -S "$WORK_DIR"
        -B "$BUILD_DIR"
        -G Ninja
        -DCMAKE_C_COMPILER=clang
        -DCMAKE_CXX_COMPILER=clang++
    )
    if [ -n "$AUTO_AIM_PREFIX" ]; then
        CMAKE_ARGS+=(-DCMAKE_PREFIX_PATH="$AUTO_AIM_PREFIX${CMAKE_PREFIX_PATH:+;$CMAKE_PREFIX_PATH}")
    fi

    echo "=== 配置 (cmake) ==="
    if ! cmake "${CMAKE_ARGS[@]}"; then
        echo "cmake 配置失败" >&2
        exit 1
    fi

    echo "=== 编译 (ninja: standard) ==="
    if ! ninja -C "$BUILD_DIR" standard -j"$(( $(nproc) - 1 ))"; then
        echo "编译失败" >&2
        exit 1
    fi
fi

# ---------------- 启动 ----------------
if [ ! -x "$RUN_PROGRAM" ]; then
    echo "找不到可执行文件：$RUN_PROGRAM" >&2
    echo "去掉 --run-only 再运行一次以触发编译。" >&2
    exit 1
fi

echo "=== 启动：$RUN_PROGRAM ${ARGS[*]}（Ctrl-C 退出）==="
exec "$RUN_PROGRAM" "${ARGS[@]}"
