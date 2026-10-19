#!/usr/bin/env bash
#
# Start the Web debugger (http://<host>:9000), then run ./run_autoaim.sh in
# another terminal. The Web UI shows the annotated camera stream, target state
# and logs.
#
# Notes:
#   * web.py is a browser UI - it does NOT open a native desktop window.
#   * The active Python virtualenv may not have Flask, so this script picks a
#     Python that actually can import Flask (normally /usr/bin/python3).
#   * setproctitle is resolved from ./.pydeps (workspace-local). It makes the
#     process appear as `wust_vision_web`, which is how the C++ side
#     (`pgrep -x wust_vision_web`) detects that the debugger is running.
#
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="$ROOT/.pydeps:${PYTHONPATH:-}"

PY=""
for cand in /usr/bin/python3 python3; do
    if command -v "$cand" >/dev/null 2>&1 && "$cand" -c "import flask" >/dev/null 2>&1; then
        PY="$cand"
        break
    fi
done

if [ -z "$PY" ]; then
    echo "错误：找不到能 import flask 的 Python。" >&2
    echo "  当前 python3 = $(command -v python3 || echo 无)" >&2
    echo "  修复： python3 -m pip install --target '$ROOT/.pydeps' flask setproctitle" >&2
    exit 1
fi

echo "Web 调试器使用 Python: $PY ($($PY -V 2>&1))"
exec "$PY" "$ROOT/web.py"
