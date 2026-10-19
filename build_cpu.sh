#!/usr/bin/env bash
#
# Build awakening on a machine WITHOUT an NVIDIA GPU.
#
# Why not `./run.sh build`?
#   run.sh forces clang/clang++, but this system only ships clang 14, which does
#   not accept `-std=c++23`. gcc-12 does, so the CPU build uses gcc-12 here.
#   TensorRT/CUDA is disabled and OpenVINO (CPU) is used for inference.
#
# ROS2 (Humble) is enabled because the vision node publishes the three topics
# (cmd_gimbal_joint / cmd_shoot / tracker/target) for the navigation stack.
# Open3D - required only by the optional radar target - is not installed, so
# radar/sentry targets are disabled instead.
#
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD="$ROOT/build"

export MVCAM_SDK_PATH=/opt/MVS
export MVCAM_COMMON_RUNENV=/opt/MVS/lib
export MVCAM_GENICAM_CLPROTOCOL=/opt/MVS/lib/CLProtocol
export ALLUSERSPROFILE=/opt/MVS/MVFG

# ---- ROS2 环境：auto_aim_interfaces 是发布 tracker/target 必需的 ----
has_auto_aim_interfaces() {
    [ -n "${AMENT_PREFIX_PATH:-}" ] \
        && printf '%s\n' "$AMENT_PREFIX_PATH" | tr ':' '\n' | grep -q '/auto_aim_interfaces$'
}

find_auto_aim_ws() {
    local base hit
    for base in "$HOME" "$HOME/Lin" "$HOME/Misc" "$HOME/github" "$HOME/Downloads"; do
        [ -d "$base" ] || continue
        hit=$(find "$base" -maxdepth 5 -type d -name auto_aim_interfaces \
                  -path '*/install/auto_aim_interfaces' 2>/dev/null | head -1)
        [ -n "$hit" ] || continue
        (cd "$hit/../.." && pwd)
        return 0
    done
    return 1
}

if ! has_auto_aim_interfaces; then
    # ROS 的 setup.bash 会引用未绑定变量，和 set -u 冲突 —— source 时临时关掉
    set +u
    [ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
    # 只需要能 find_package 到 auto_aim_interfaces 即可（自瞄与导航是独立进程）。
    if [ -n "${AWAKENING_ROS_WS:-}" ] && [ -f "$AWAKENING_ROS_WS/install/setup.bash" ]; then
        echo "source 指定工作空间: $AWAKENING_ROS_WS/install/setup.bash"
        # shellcheck disable=SC1090
        source "$AWAKENING_ROS_WS/install/setup.bash"
    elif ws="$(find_auto_aim_ws)"; then
        echo "自动 source ROS 工作空间: $ws/install/setup.bash"
        # shellcheck disable=SC1090
        source "$ws/install/setup.bash"
    else
        echo "错误：找不到 auto_aim_interfaces（发布 tracker/target 需要）。" >&2
        echo "      请先 source 含该接口包的工作空间再构建。" >&2
        exit 1
    fi
    set -u
fi

cmake -S "$ROOT" -B "$BUILD" -G Ninja \
    -DCMAKE_C_COMPILER=gcc-12 \
    -DCMAKE_CXX_COMPILER=g++-12 \
    -DBUILD_WITH_TRT=OFF \
    -DBUILD_WITH_OPENVINO=ON \
    -DBUILD_WITH_RERUN=OFF \
    -DBUILD_RUNTIME_TESTS=OFF \
    -DBUILD_RADAR=OFF \
    -DBUILD_SENTRY=OFF

ninja -C "$BUILD" standard -j"$(( $(nproc) - 1 ))"

echo "built: $ROOT/bin/standard"
