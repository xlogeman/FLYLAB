#!/usr/bin/env bash
# ============================================================================
# px4 git 元数据自检/重建 —— 幂等脚本, 启动脚本每次运行前调用, 缺啥补啥
#
# 背景: PX4 的 make 强制要求 .git 存在且版本号可生成, 本仓库为瘦身把 px4 的
#       git 历史剔除了。常见残缺状态:
#       a) 主仓库无 .git          → px_update_git_header.py 拿不到版本号
#       b) 子模块无 .git          → cmake 构建目标检查失败
#       c) NuttX 子模块有 .git 但无 tag → 版本头脚本 IndexError (实测踩坑)
#       全部通过则秒退, 不影响启动速度
# ============================================================================
set -e
PXD="$(cd "$(dirname "$0")/../px4.1.15.2" && pwd)"
cd "$PXD"

NUTTX_SUBS="platforms/nuttx/NuttX/nuttx platforms/nuttx/NuttX/apps"

# ---- 自检: 判断是否需要修复 ------------------------------------------------
need_fix=0
[ -d .git ] || need_fix=1
for sub in $NUTTX_SUBS; do
    # 有 .git 但没有任何 commit/tag → 版本头脚本会崩
    if [ -d "$sub/.git" ] && ! git -C "$sub" describe --tags >/dev/null 2>&1; then
        need_fix=1
    fi
done
if [ "$need_fix" = "0" ]; then
    echo "[px4-git] git 元数据完整, 无需处理"
    exit 0
fi

# ---- 修复 a) 主仓库 --------------------------------------------------------
echo "[px4-git] 1/4 检查主仓库..."
if [ ! -d .git ]; then
    git init -q
    git config user.name "${USER:-repro}" && git config user.email "repro@local"
    git add -A && git commit -qm "restore px4 v1.15.2 from project"
    git tag -f v1.15.2
    echo "  init: 主仓库 + tag v1.15.2"
fi

# ---- 修复 b) 子模块 .git ---------------------------------------------------
echo "[px4-git] 2/4 检查子模块 .git..."
git config -f .gitmodules --get-regexp '^submodule\..*\.path$' | awk '{print $2}' |
while read p; do
    if [ -d "$p" ] && [ ! -e "$p/.git" ]; then
        (cd "$p" && git init -q && git config user.name repro && git config user.email repro@local \
         && git add -A && git commit -qm "restore submodule")
        echo "  init: $p"
    fi
done

# ---- 修复 c) NuttX tag (含"有.git但无commit"的残缺态) ----------------------
echo "[px4-git] 3/4 检查 NuttX 版本 tag..."
for sub in $NUTTX_SUBS; do
    [ -d "$sub/.git" ] || continue
    if ! git -C "$sub" rev-parse -q --verify HEAD >/dev/null 2>&1; then
        (cd "$sub" && git add -A && git commit -qm "restore submodule")
    fi
    git -C "$sub" tag -f nuttx-10.3.0 2>/dev/null || true
done

# ---- 修复 d) 脚本执行位 ----------------------------------------------------
echo "[px4-git] 4/4 恢复脚本执行位..."
find . -path ./build -prune -o -name "*.sh" -exec chmod +x {} +

echo "[px4-git] 完成"
