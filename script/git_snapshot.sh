#!/usr/bin/env bash
# 一键存档：把当前改动提交成一个带时间戳的 commit，并打 tag，随时可以回档。
#
#   ./script/git_snapshot.sh "改了什么"           # 只存本地
#   ./script/git_snapshot.sh "改了什么" --push    # 存档并推送到 GitHub
#
# 回档方法见 docs/git-guide.md
set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")/.."

msg="${1:-}"
push=""
for a in "$@"; do
    [ "$a" = "--push" ] && push=1
done
if [ -z "$msg" ] || [ "$msg" = "--push" ]; then
    echo "用法: $0 \"这次改了什么\" [--push]"
    exit 1
fi

if [ -z "$(git status --porcelain)" ]; then
    echo "工作区没有改动，只打标签。"
else
    git add -A
    git commit -m "$msg"
fi

stamp="$(date +%Y%m%d_%H%M%S)"
tag="snap-$stamp"
git tag "$tag"

echo
echo "已存档: $(git log --oneline -1)"
echo "标签:   $tag"
echo "回档:   git reset --hard $tag"

if [ -n "$push" ]; then
    branch="$(git rev-parse --abbrev-ref HEAD)"
    git push origin "$branch"
    git push origin "$tag"
    echo "已推送到 origin/$branch（含标签 $tag）。"
fi
