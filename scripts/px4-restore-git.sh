#!/usr/bin/env bash
# ============================================================================
# px4 git 元数据重建 —— 全新环境跑一次即可 (克隆本仓库后、编译 PX4 前)
# 作用: PX4 的 make 强制要求 .git 存在且版本号可生成, 本仓库为瘦身把 px4 的
#       git 历史剔除了, 此脚本按 REPRODUCE.md 第三节自动补齐
# ============================================================================
set -e
PXD="$(cd "$(dirname "$0")/../px4.1.15.2" && pwd)"
cd "$PXD"

if [ -d .git ]; then
    echo "[px4-git] .git 已存在, 无需处理"
    exit 0
fi

echo "[px4-git] 1/4 重建主仓库..."
git init -q
git config user.name "${USER:-repro}" && git config user.email "repro@local"
git add -A && git commit -qm "restore px4 v1.15.2 from project"
git tag -f v1.15.2

echo "[px4-git] 2/4 重建子模块 .git (构建依赖存在性检查 + 让 IDE/状态显示干净)..."
git config -f .gitmodules --get-regexp '^submodule\..*\.path$' | awk '{print $2}' |
while read p; do
    if [ -d "$p" ] && [ ! -e "$p/.git" ]; then
        (cd "$p" && git init -q && git config user.name repro && git config user.email repro@local \
         && git add -A && git commit -qm "restore submodule")
        echo "  init: $p"
    fi
done

echo "[px4-git] 3/4 补 NuttX 版本 tag..."
(cd platforms/nuttx/NuttX/nuttx && git tag -f nuttx-10.3.0 2>/dev/null || true)
(cd platforms/nuttx/NuttX/apps  && git tag -f nuttx-10.3.0 2>/dev/null || true)

echo "[px4-git] 4/4 恢复脚本执行位..."
find . -path ./build -prune -o -name "*.sh" -exec chmod +x {} +

echo "[px4-git] 完成"
