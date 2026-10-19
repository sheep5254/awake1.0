#!/usr/bin/env bash
#
# 启动 PlotJuggler 查看自瞄/导航数据。
#
#   ./plotjuggler.sh
#
# ── 背景 1：snap(core20) 的 glibc ────────────────────────────────────────────
# snap(core20) 自带 glibc 2.31，被注入后会顶掉系统 glibc 2.35，典型报错：
#   /opt/ros/humble/lib/plotjuggler/plotjuggler: symbol lookup error:
#   /snap/core20/current/lib/x86_64-linux-gnu/libpthread.so.0:
#   undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE
# 本脚本用 `env -i` 起一个**完全隔离的环境**（只显式传入必需的几个变量），
# 这样 LD_LIBRARY_PATH / LD_PRELOAD / LD_AUDIT 等一切注入途径都被切断；
# 并直接用系统 loader 指定库搜索路径。
#
# ── 背景 2：必须能找到 auto_aim_interfaces（否则勾完话题会闪退）────────────
# PlotJuggler 订阅 /tracker/target 时，要用 rosidl introspection 类型支持库
# 现场生成消息 schema；它是靠 AMENT_PREFIX_PATH 找包的（找不到就抛异常）。
# ros-humble-plotjuggler-ros 的 DataStreamROS2 在 subscribeToTopic() 里
# 没有 try/catch，所以只要解析不到 auto_aim_interfaces，勾完话题点 OK 就会
# 直接闪退（std::terminate / abort），而不是弹错误框。
# 因此这里先 source 含 auto_aim_interfaces 的工作空间，再把它传进隔离环境。
# 可用 AWAKENING_ROS_WS=/path/to/ws 指定其它工作空间（与 run_autoaim.sh 一致）。
#
# ── 背景 3：绝对不能用 ld.so 显式加载 plotjuggler（否则也会闪退）─────────────
# PlotJuggler 是用 QCoreApplication::applicationDirPath() 找**自带插件**的
# （libParserROS2.so 提供 ros2msg 解析器、还有 CSV/FFT/Lua 等）。
# 如果用 `/lib64/ld-linux-x86-64.so.2 --library-path ... plotjuggler` 启动，
# /proc/self/exe 指向的是 ld.so，applicationDirPath() 就变成了
# /usr/lib/x86_64-linux-gnu —— 自带的 lib/plotjuggler 目录根本不会被扫描，
# ros2msg 解析器没注册，订阅话题时 factories.at("ros2msg") 抛
# std::out_of_range(map::at) → std::terminate → 闪退。
# 所以这里直接 exec 可执行文件（LD_LIBRARY_PATH 已由 env -i 完全控制，
# 不需要再借 ld.so），并把插件目录显式传给 --plugin_folders 做双保险。
#
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PJ=/opt/ros/humble/lib/plotjuggler/plotjuggler
PJ_DIR="$(dirname "$PJ")"

if [ ! -x "$PJ" ]; then
    echo "错误：找不到 PlotJuggler，请先安装：" >&2
    echo "      sudo apt install ros-humble-plotjuggler ros-humble-plotjuggler-ros" >&2
    exit 1
fi

# ---- 找并 source 含 auto_aim_interfaces 的工作空间 ----
has_auto_aim_interfaces() {
    [ -n "${AMENT_PREFIX_PATH:-}" ] \
        && printf '%s\n' "$AMENT_PREFIX_PATH" | tr ':' '\n' | grep -q '/auto_aim_interfaces$'
}

find_auto_aim_ws() {
    # 在常见位置找 <ws>/install/auto_aim_interfaces，返回 <ws>
    local base hit
    for base in "$HOME/Lin/pb2025_sentry_ws" "$HOME/Lin" "$HOME" "$HOME/Misc" "$HOME/github" "$HOME/Downloads"; do
        [ -d "$base" ] || continue
        hit=$(find "$base" -maxdepth 5 -type d -name auto_aim_interfaces \
                  -path '*/install/auto_aim_interfaces' 2>/dev/null | head -1)
        [ -n "$hit" ] || continue
        (cd "$hit/../.." && pwd)
        return 0
    done
    return 1
}

# ROS 的 setup.bash 会引用未绑定变量，和 set -u 冲突 —— source 时临时关掉
set +u
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
if ! has_auto_aim_interfaces; then
    if [ -n "${AWAKENING_ROS_WS:-}" ] && [ -f "$AWAKENING_ROS_WS/install/setup.bash" ]; then
        echo "source 指定工作空间: $AWAKENING_ROS_WS/install/setup.bash"
        source "$AWAKENING_ROS_WS/install/setup.bash"
    elif ws="$(find_auto_aim_ws)"; then
        echo "自动 source ROS 工作空间: $ws/install/setup.bash"
        source "$ws/install/setup.bash"
    fi
fi
set -u

# ---- 库搜索路径：系统目录排最前，绝不落到 snap 上 ----
LIBS="/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu:/usr/lib:/lib"
LIBS="$LIBS:/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu"
LIBS="$LIBS:/opt/ros/humble/opt/rviz_ogre_vendor/lib:/opt/MVS/lib/64:/opt/MVS/lib/32:/usr/local/lib"
# 关键：AMENT_PREFIX_PATH 里每个包（含 auto_aim_interfaces）的 lib 目录
for prefix in $(printf '%s' "${AMENT_PREFIX_PATH:-/opt/ros/humble}" | tr ':' ' '); do
    [ -n "$prefix" ] && [ -d "$prefix/lib" ] && LIBS="$LIBS:$prefix/lib"
done
# 兜底：把调用者 LD_LIBRARY_PATH 里的条目剔除 /snap/ 后补进来，避免漏包
while IFS= read -r entry; do
    case "$entry" in
        *"/snap/"*|"") continue ;;
    esac
    case ":$LIBS:" in *":$entry:"*) ;; *) LIBS="$LIBS:$entry" ;; esac
done < <(printf '%s\n' "${LD_LIBRARY_PATH:-}" | tr ':' '\n')

# ---- 诊断输出 ----
echo "启动 PlotJuggler（隔离环境）..."
echo "  原 LD_PRELOAD = [${LD_PRELOAD:-}]"
echo "  原 LD_AUDIT   = [${LD_AUDIT:-}]"
echo "  原 LD_LIBRARY_PATH 中 /snap/ 条目数: $(printf '%s' "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | grep -c '/snap/' || true)"
echo "  订阅话题: /cmd_gimbal_joint  /cmd_shoot  /tracker/target"
echo "  （/cmd_shoot 为 example_interfaces/UInt8；/tracker/target 是 SensorDataQoS）"
if has_auto_aim_interfaces; then
    echo "  auto_aim_interfaces 类型支持: OK（/tracker/target 可解析）"
else
    echo "  警告：AMENT_PREFIX_PATH 里没有 auto_aim_interfaces！" >&2
    echo "        订阅 /tracker/target 时 PlotJuggler 会直接闪退。" >&2
    echo "        请先 source 含该接口包的工作空间，或设置 AWAKENING_ROS_WS=/path/to/ws" >&2
fi
if [ -f "$PJ_DIR/libParserROS2.so" ]; then
    echo "  自带插件目录: $PJ_DIR（ros2msg 解析器 OK）"
else
    echo "  警告：$PJ_DIR/libParserROS2.so 不存在！" >&2
    echo "        订阅任何话题都会因 factories.at(\"ros2msg\") 闪退。" >&2
    echo "        请安装/修复：sudo apt install --reinstall ros-humble-plotjuggler" >&2
fi

UID_NUM="$(id -u)"
ENV_ARGS=(
    HOME="$HOME"
    USER="${USER:-$(id -un)}"
    LOGNAME="${LOGNAME:-$(id -un)}"
    DISPLAY="${DISPLAY:-:0}"
    XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
    XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$UID_NUM}"
    DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-unix:path=/run/user/$UID_NUM/bus}"
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
    LANG="${LANG:-C.UTF-8}"
    ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
    AMENT_PREFIX_PATH="${AMENT_PREFIX_PATH:-/opt/ros/humble}"
    LD_LIBRARY_PATH="$LIBS"
)
# 便于无头测试 / Wayland 用户
[ -n "${QT_QPA_PLATFORM:-}" ] && ENV_ARGS+=(QT_QPA_PLATFORM="$QT_QPA_PLATFORM")
# 与自瞄/导航进程保持一致，否则 DDS 发现不到话题
[ -n "${RMW_IMPLEMENTATION:-}" ] && ENV_ARGS+=(RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION")
[ -n "${ROS_LOCALHOST_ONLY:-}" ] && ENV_ARGS+=(ROS_LOCALHOST_ONLY="$ROS_LOCALHOST_ONLY")
[ -n "${ROS_LOG_DIR:-}" ] && ENV_ARGS+=(ROS_LOG_DIR="$ROS_LOG_DIR")
[ -n "${ROS_HOME:-}" ] && ENV_ARGS+=(ROS_HOME="$ROS_HOME")

# 不要用 ld.so 显式加载（见文件头「背景 3」），否则自带插件目录会被认错。
# 额外显式传一次插件目录做双保险；用户自己传了 --plugin_folders 就不重复加。
PLUGIN_ARGS=()
case " $* " in
    *" --plugin_folders "*|*" --plugin_folders="*) ;;
    *) PLUGIN_ARGS=(--plugin_folders "$PJ_DIR") ;;
esac

exec env -i "${ENV_ARGS[@]}" "$PJ" "${PLUGIN_ARGS[@]}" "$@"
