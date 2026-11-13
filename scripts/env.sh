#!/usr/bin/env bash
# ============================================================================
# 环境总入口 —— 编译和运行前都先 source 本文件
#   source /root/flylab/scripts/env.sh
# ============================================================================

# ROS2 Humble
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash

# colcon 工作空间 (build 之后自动生效)
_ws=/root/flylab/ros2_ws
if [ -f "$_ws/install/setup.bash" ]; then
    source "$_ws/install/setup.bash"
fi

# ORB_SLAM3 运行时库 (orbslam3 wrapper 找 libORB_SLAM3.so / libDBoW2 / libg2o)
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:\
/root/flylab/ORB_SLAM3_FIXED/lib:\
/root/flylab/ORB_SLAM3_FIXED/Thirdparty/DBoW2/lib:\
/root/flylab/ORB_SLAM3_FIXED/Thirdparty/g2o/lib

# onnxruntime CUDA EP 依赖: providers 插件在 pip onnxruntime-gpu 的 capi 目录,
# CUDA/cuDNN 运行库在 pip nvidia-* 包里 (yolo_detect_node device:=cuda 用)
_ort_capi=/usr/local/lib/python3.10/dist-packages/onnxruntime/capi
if [ -d "$_ort_capi" ]; then
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$_ort_capi
    for _d in /usr/local/lib/python3.10/dist-packages/nvidia/*/lib; do
        [ -d "$_d" ] && export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$_d
    done
    unset _ort_capi _d
fi

# AirSim 所在的 Windows 宿主机:
# - NAT 网络 (WSL 默认 172.16/12): 网关即宿主机 → PX4_SIM_HOST_ADDR=网关
# - mirrored 网络 (.wslconfig networkingMode=mirrored, 与宿主共享局域网 IP):
#   网关是路由器而非宿主机 → 宿主即本机回环 127.0.0.1 (实测 RPC 可达)
_wsl_ip=$(ip -4 addr show eth0 2>/dev/null | grep -oP 'inet \K[\d.]+' | head -1)
case $_wsl_ip in
    172.1[6-9].*|172.2[0-9].*|172.3[01].*)
        export PX4_SIM_HOST_ADDR=$(ip route | grep default | awk '{print $3}') ;;
    *)
        export PX4_SIM_HOST_ADDR=127.0.0.1 ;;
esac
unset _wsl_ip

# Fast DDS: 同机大图 (900KB 显示帧转发等) 走 SHM 零拷贝, 避开 UDP 分片
# 背压 (实测 30Hz 原始图转发 publish 阻塞 ~80ms/帧, 见 fastrtps_shm.xml 头注)
export FASTRTPS_DEFAULT_PROFILES_FILE=/root/flylab/scripts/fastrtps_shm.xml

# SHM 遗留段清理: 节点被强杀时其 SHM 段不会自动释放, 日积月累吃满 /dev/shm
# (满了新节点报 "Failed to create segment" 并回退 UDP)。脚本逐段 fuser 检测
# 占用, 只删无主段, 任何时机运行都不会打断正在跑的节点。静默调用, 只在
# 实际清理到东西时输出一行
bash /root/flylab/scripts/clean-fastrtps-shm.sh

# rviz/ORB 走软件渲染 (WSL 无独显穿透)
export LIBGL_ALWAYS_SOFTWARE=1
export DISPLAY=${DISPLAY:-:0}
