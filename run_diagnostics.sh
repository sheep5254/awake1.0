#!/usr/bin/env bash
#
# 单独运行自瞄（standard 可执行程序）。
#
#   ./run_autoaim.sh [配置文件] [debug]
#
# 参数：
#   配置文件   默认 config/omni.yaml（海康相机，标准自瞄）
#              视频测试可用 config/video_test.yaml
#    debug      true/false。不填时自动判断：
#               - 检测到 Web 调试器(wust_vision_web)在跑 -> true，输出可视化数据
#               - 否则 -> false，纯算法运行，不产生调试开销
#
# 停止：Ctrl-C
#
# 注意：配置里 publish_to_nav: true 时要发布 ROS2 话题，
#       必须能加载 libauto_aim_interfaces__rosidl_typesupport_fastrtps_cpp.so，
#       也就是要 source 含 auto_aim_interfaces 的工作空间 —— 本脚本会自动找并 source。
#
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export MVCAM_SDK_PATH=/opt/MVS
export MVCAM_COMMON_RUNENV=/opt/MVS/lib
export MVCAM_GENICAM_CLPROTOCOL=/opt/MVS/lib/CLProtocol
export ALLUSERSPROFILE=/opt/MVS/MVFG

# ---- 确保 ROS2 接口可用（auto_aim_interfaces）----
has_auto_aim_interfaces() {
    [ -n "${AMENT_PREFIX_PATH:-}" ] \
        && printf '%s\n' "$AMENT_PREFIX_PATH" | tr ':' '\n' | grep -q '/auto_aim_interfaces$'
}

find_auto_aim_ws() {
    # 在常见位置找 <ws>/install/auto_aim_interfaces，返回 <ws>
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
    # 只需要能加载 auto_aim_interfaces 的类型支持库即可；
    # 自瞄与导航是独立进程，通过 ROS 话题通信，不必用同一个工作空间。
    # 若消息定义与导航不一致，可用 AWAKENING_ROS_WS=/path/to/ws 指定。
    if [ -n "${AWAKENING_ROS_WS:-}" ] && [ -f "$AWAKENING_ROS_WS/install/setup.bash" ]; then
        echo "source 指定工作空间: $AWAKENING_ROS_WS/install/setup.bash"
        # shellcheck disable=SC1090
        source "$AWAKENING_ROS_WS/install/setup.bash"
    elif ws="$(find_auto_aim_ws)"; then
        echo "自动 source ROS 工作空间: $ws/install/setup.bash"
        # shellcheck disable=SC1090
        source "$ws/install/setup.bash"
    else
        echo "警告：找不到 auto_aim_interfaces 工作空间。" >&2
        echo "      若配置里 publish_to_nav: true，启动时会因加载不到类型支持库而崩溃。" >&2
        echo "      请先 source 含该接口包的工作空间，或把 publish_to_nav 设为 false。" >&2
    fi
    set -u
fi

export LD_LIBRARY_PATH="/opt/MVS/lib/64:/opt/MVS/lib/32:$ROOT/build-codex-diagnostics:${LD_LIBRARY_PATH:-}"

CONFIG="${1:-$ROOT/config/omni.yaml}"

if [ "$#" -ge 2 ]; then
    DEBUG="$2"
elif pgrep -x wust_vision_web >/dev/null 2>&1; then
    DEBUG=true
else
    DEBUG=false
fi

if [ ! -x "$ROOT/build-codex-diagnostics/bin/standard" ]; then
    echo "错误：找不到 $ROOT/build-codex-diagnostics/bin/standard，请先编译 build-codex-diagnostics 诊断版本" >&2
    exit 1
fi
if [ ! -f "$CONFIG" ]; then
    echo "错误：配置文件不存在：$CONFIG" >&2
    exit 1
fi

echo "诊断版自瞄启动中：config=$CONFIG  debug=$DEBUG"
exec "$ROOT/build-codex-diagnostics/bin/standard" "$CONFIG" "$DEBUG"
