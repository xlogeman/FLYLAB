#!/usr/bin/env python3
"""ULG 飞行日志解析层 —— 纯 pyulog/numpy, 不碰 Qt, 可单独 CLI 测试。

职责:
  扫描 PX4 SITL 日志目录 / 解析 .ulg / 话题字段提取 (相对时间轴)
  预置曲线模板 (高度/姿态/油门/角速度/电池/EKF)
  悬停诊断: 悬停段油门统计 + z 振荡 FFT 频谱 (服务 X300 0.15Hz 振荡定位)

时间轴约定: 所有曲线时间 = (timestamp - start_timestamp)/1e6, 秒。
"""
import glob
import os

import numpy as np
from pyulog import ULog

# 不作为可勾选曲线的字段 (时间戳元数据/填充/设备 id 等噪音字段)
_META_FIELDS = {
    'timestamp', 'timestamp_sample', 'ref_timestamp', 'nav_state_timestamp',
    'time_utc_usec', 'time_last_fuse', 'arming_time', 'takeoff_time',
}

# ---------- 曲线预置模板 ----------
# 结构: {模板名: [(子图标题, [(话题, 字段, 曲线标签)...])...]}
# 字段缺失自动跳过 (机型/固件版本差异), 至少一条画出来即算成功
PRESETS = {
    '高度 / 垂直速度': [
        ('位置 (NED)', [
            ('vehicle_local_position[0]', 'z', '实际 z (下+)'),
            ('vehicle_local_position_setpoint[0]', 'z', '设定 z'),
        ]),
        ('垂直速度', [
            ('vehicle_local_position[0]', 'vz', '实际 vz (下+)'),
            ('vehicle_local_position_setpoint[0]', 'vz', '设定 vz'),
        ]),
        ('水平速度', [
            ('vehicle_local_position[0]', 'vx', '实际 vx (北)'),
            ('vehicle_local_position[0]', 'vy', '实际 vy (东)'),
        ]),
    ],
    '悬停振荡诊断 (FFT)': [],   # 特殊模板: ulg_window 调 hover_diagnosis() 定制绘图
    '姿态 (欧拉角)': [
        ('姿态实际 (deg)', []),  # 特殊: 四元数转欧拉, ulg_window 定制
        ('姿态设定 (deg)', []),  # 特殊: q_d 转欧拉
    ],
    '油门 / 执行器': [
        ('执行器归一化输出 (actuator_motors)', [
            ('actuator_motors[0]', 'control[0]', 'motor 0'),
            ('actuator_motors[0]', 'control[1]', 'motor 1'),
            ('actuator_motors[0]', 'control[2]', 'motor 2'),
            ('actuator_motors[0]', 'control[3]', 'motor 3'),
        ]),
        ('机体推力设定 (z)', [
            ('vehicle_thrust_setpoint[0]', 'xyz[2]', 'thrust z (NED 下+)'),
        ]),
    ],
    '角速度 / 速率环': [
        ('角速度实际 (rad/s)', [
            ('vehicle_angular_velocity[0]', 'xyz[0]', 'p (roll rate)'),
            ('vehicle_angular_velocity[0]', 'xyz[1]', 'q (pitch rate)'),
            ('vehicle_angular_velocity[0]', 'xyz[2]', 'r (yaw rate)'),
        ]),
        ('速率设定 (rad/s)', [
            ('vehicle_rates_setpoint[0]', 'roll', 'roll sp'),
            ('vehicle_rates_setpoint[0]', 'pitch', 'pitch sp'),
            ('vehicle_rates_setpoint[0]', 'yaw', 'yaw sp'),
        ]),
    ],
    '电池 / 状态': [
        ('电池电压 (V)', [
            ('battery_status[0]', 'voltage_v', 'voltage'),
            ('battery_status[0]', 'voltage_filtered_v', 'filtered'),
        ]),
        ('电池电流 (A) / 剩余', [
            ('battery_status[0]', 'current_a', 'current (A)'),
            ('battery_status[0]', 'remaining', 'remaining (0~1)'),
        ]),
    ],
    'EKF 高度融合': [
        ('气压计高度 innovation', [
            ('estimator_aid_src_baro_hgt[0]', 'innovation', 'baro inn'),
            ('estimator_aid_src_baro_hgt[0]', 'innovation_variance', 'var'),
        ]),
        ('GNSS 高度 innovation', [
            ('estimator_aid_src_gnss_hgt[0]', 'innovation', 'gnss inn'),
            ('estimator_aid_src_gnss_hgt[0]', 'innovation_variance', 'var'),
        ]),
    ],
}

FFT_FREQ_MIN = 0.05   # FFT 主频搜索下限 (Hz), 排除直流/超低频漂移
FFT_FREQ_MAX = 3.0    # 搜索上限 (Hz), 多旋翼姿态/高度振荡关注低频段
HOVER_VZ_MAX = 0.3    # 悬停判定: |vz| 上限 (m/s)
HOVER_VXY_MAX = 0.5   # 悬停判定: 水平速度上限 (m/s)


# ---------- 文件扫描与解析 ----------

def scan_log_dir(root):
    """扫描日志目录 (root/日期/时刻.ulg), 按 mtime 新→旧返回文件路径列表。
    目录不存在返回空列表。"""
    if not os.path.isdir(root):
        return []
    files = glob.glob(os.path.join(root, '*', '*.ulg'))
    files.sort(key=os.path.getmtime, reverse=True)
    return files


def display_name(path, root):
    """文件显示名: 相对日志根目录的 '日期/时刻.ulg'。"""
    try:
        return os.path.relpath(path, root)
    except ValueError:
        return path


def load_ulog(path):
    """解析 ulg 文件 (后台线程调用, 大文件 1~2s)。"""
    return ULog(path)


def ulog_info(ul, path):
    """基本信息汇总 (单行展示用)。"""
    dur = (ul.last_timestamp - ul.start_timestamp) / 1e6
    size_mb = os.path.getsize(path) / 1e6
    return (f'时长 {dur/60:.1f} min | 大小 {size_mb:.1f} MB | '
            f'话题 {len(ul.data_list)} | 飞控消息 {len(ul.logged_messages)} | '
            f'丢段 {len(ul.dropouts)} | 参数 {len(ul.initial_parameters)}')


# ---------- 话题树 ----------

def topic_tree(ul):
    """话题 → 可勾选字段列表, 树形展示用。
    返回有序 dict: 'topic[multi_id]' -> [field, ...] (剔除元数据/填充字段)。"""
    tree = {}
    for ds in ul.data_list:
        fields = [f for f in ds.data
                  if f not in _META_FIELDS and not f.startswith('_padding')
                  and not f.startswith('_')]
        if fields:
            tree[f'{ds.name}[{ds.multi_id}]'] = sorted(fields)
    return dict(sorted(tree.items()))


def get_series(ul, topic_id, field):
    """提取单条曲线: (t[秒相对起点], y)。话题不存在/字段缺失返回 None。"""
    ds = ul.data_list.get(topic_id) if hasattr(ul.data_list, 'get') else None
    if ds is None:
        ds = next((d for d in ul.data_list if f'{d.name}[{d.multi_id}]' == topic_id),
                  None)
    if ds is None:
        return None
    y = ds.data.get(field)
    if y is None or len(y) == 0:
        return None
    t = (ds.data['timestamp'] - ul.start_timestamp) / 1e6
    return t, np.asarray(y)


# ---------- 悬停诊断 ----------

def _airborne_mask(ul, t_target):
    """目标时间轴上的"离地"布尔掩码: 已解锁 且 未落地。
    armed/landed 按各自时间轴插值对齐到 t_target。"""
    armed = _bool_series(ul, 'actuator_armed[0]', 'armed', t_target)
    landed = _bool_series(ul, 'vehicle_land_detected[0]', 'landed', t_target)
    if armed is None:
        return None
    mask = armed.astype(bool)
    if landed is not None:
        mask &= ~landed.astype(bool)
    return mask


def _bool_series(ul, topic_id, field, t_target):
    """离散布尔量插值到目标时间轴 (ffill: 取时间轴上最近一次采样)。"""
    s = get_series(ul, topic_id, field)
    if s is None:
        return None
    t, y = s
    idx = np.searchsorted(t, t_target, side='right') - 1
    idx = np.clip(idx, 0, len(y) - 1)
    return np.asarray(y)[idx]


def hover_stats(ul):
    """悬停段油门统计。悬停 = 离地 且 |vz|<HOVER_VZ_MAX 且 vxy<HOVER_VXY_MAX。
    返回 dict (缺执行器话题时 throttle 相关为 None), keys:
      hover_time 悬停时长[s], throttle_mean/std, vz_rms 悬停段 vz 标准差[m/s]
    注意: actuator_motors.control 为归一化输出, SITL 下模拟器侧还有
    c' = 0.8c + 0.2 映射 (MavLinkMultirotorApi), 与 MPC_THR_HOVER 对比时须换算。"""
    s = get_series(ul, 'vehicle_local_position[0]', 'z')
    if s is None:
        return None
    t, _ = s
    mask_air = _airborne_mask(ul, t)
    if mask_air is None or not mask_air.any():
        return {'hover_time': 0.0, 'throttle_mean': None, 'throttle_std': None,
                'vz_rms': None}
    vz = get_series(ul, 'vehicle_local_position[0]', 'vz')
    vxy = None
    svx = get_series(ul, 'vehicle_local_position[0]', 'vx')
    svy = get_series(ul, 'vehicle_local_position[0]', 'vy')
    if svx is not None and svy is not None:
        vxy = np.sqrt(svx[1] ** 2 + svy[1] ** 2)
    mask_hover = mask_air.copy()
    if vz is not None:
        mask_hover &= np.abs(np.interp(t, vz[0], vz[1])) < HOVER_VZ_MAX
    if vxy is not None:
        mask_hover &= np.interp(t, t, vxy) < HOVER_VXY_MAX   # vxy 已是 t 同轴
    dt = np.median(np.diff(t)) if len(t) > 1 else 1.0
    hover_time = float(mask_hover.sum() * dt)
    out = {'hover_time': hover_time, 'throttle_mean': None,
           'throttle_std': None,
           'vz_rms': None}
    if vz is not None and mask_hover.any():
        out['vz_rms'] = float(np.std(np.interp(t, vz[0], vz[1])[mask_hover]))
    # 自动选有有效数据的电机通道求均值 (四轴 0..3 / 共轴 0..1, 未用通道全 NaN)
    thr = None
    for i in range(12):
        m = get_series(ul, 'actuator_motors[0]', f'control[{i}]')
        if m is None or not np.isfinite(m[1]).any():
            continue
        c = np.interp(t, m[0], m[1])
        thr = c if thr is None else thr + c
        if thr is c:
            thr = c.copy()
    if thr is not None and mask_hover.any():
        seg = thr[mask_hover]   # 未解锁段 control 为 NaN, 只统计有效值
        if np.isfinite(seg).any():
            out['throttle_mean'] = float(np.nanmean(seg))
            out['throttle_std'] = float(np.nanstd(seg))
    return out


def hover_diagnosis(ul):
    """悬停振荡诊断: 离地最长连续段的 z/vz 曲线 + FFT 频谱。
    返回 dict:
      t/z, t_vz/vz        曲线 (离地段)
      f/amp_z, amp_vz     FFT 频率轴与幅值谱 (单边)
      f_dom_z/f_dom_vz    主频 (Hz), 找不到离地段时为 None
      stats               hover_stats() 结果
      note                统计摘要文本
    """
    out = {'t': None, 'z': None, 't_vz': None, 'vz': None,
           'f': None, 'amp_z': None, 'amp_vz': None,
           'f_dom_z': None, 'f_dom_vz': None,
           'stats': hover_stats(ul), 'note': ''}
    s = get_series(ul, 'vehicle_local_position[0]', 'z')
    if s is None:
        return out
    t, z = s
    mask_air = _airborne_mask(ul, t)
    if mask_air is None or not mask_air.any():
        out['note'] = '日志中没有离地飞行段'
        return out
    # 最长连续离地段 (FFT 要等间隔连续采样)
    seg = _longest_run(mask_air)
    i0, i1 = seg
    t, z = t[i0:i1], np.asarray(z[i0:i1])
    out['t'], out['z'] = t, z
    svz = get_series(ul, 'vehicle_local_position[0]', 'vz')
    if svz is not None:
        out['t_vz'] = svz[0][i0:i1]
        out['vz'] = np.asarray(svz[1][i0:i1])
    freqs, amp_z = _fft_amplitude(t, z)
    out['f'], out['amp_z'] = freqs, amp_z
    out['f_dom_z'] = _dominant_freq(freqs, amp_z)
    if out['vz'] is not None:
        _, amp_vz = _fft_amplitude(t, out['vz'])
        out['amp_vz'] = amp_vz
        out['f_dom_vz'] = _dominant_freq(freqs, amp_vz)
    out['note'] = _diagnosis_note(out)
    return out


def _longest_run(mask):
    """布尔掩码里最长连续 True 段的 (起,止) 索引 (左闭右开)。"""
    best, cur = (0, 0), None
    for i, v in enumerate(mask):
        if v and cur is None:
            cur = i
        elif not v and cur is not None:
            if best[1] - best[0] < i - cur:
                best = (cur, i)
            cur = None
    if cur is not None and best[1] - best[0] < len(mask) - cur:
        best = (cur, len(mask))
    return best


def _fft_amplitude(t, y):
    """重采样到等间隔 → Hann 加窗 → 单边幅值谱 (幅值 2|X|/n, 物理单位)。"""
    dt = float(np.median(np.diff(t)))
    tu = np.arange(t[0], t[-1], dt)
    yu = np.interp(tu, t, y)
    n = len(tu)
    win = np.hanning(n)
    spec = np.fft.rfft((yu - yu.mean()) * win)
    freqs = np.fft.rfftfreq(n, dt)
    amp = 2.0 * np.abs(spec) / n
    return freqs, amp


def _dominant_freq(freqs, amp):
    """带宽 [FFT_FREQ_MIN, FFT_FREQ_MAX] 内幅值峰对应的频率 (Hz)。"""
    band = (freqs >= FFT_FREQ_MIN) & (freqs <= FFT_FREQ_MAX)
    if not band.any():
        return None
    i = np.argmax(amp[band])
    return float(freqs[band][i])


def _diagnosis_note(d):
    st = d.get('stats') or {}
    lines = []
    if st.get('hover_time'):
        lines.append(f'悬停时长 {st["hover_time"]:.0f}s')
    if st.get('throttle_mean') is not None:
        lines.append(f'悬停油门 {st["throttle_mean"]:.3f}±{st["throttle_std"]:.3f} '
                     f'(归一化输出, SITL 侧另有 c\'=0.8c+0.2 映射)')
    if d.get('f_dom_z') is not None:
        lines.append(f'z 主频 {d["f_dom_z"]:.2f} Hz')
    if d.get('f_dom_vz') is not None:
        lines.append(f'vz 主频 {d["f_dom_vz"]:.2f} Hz')
    if st.get('vz_rms') is not None:
        lines.append(f'悬停 vz RMS {st["vz_rms"]:.3f} m/s')
    return ' | '.join(lines)


# ---------- 姿态四元数 → 欧拉角 ----------

def _quat_to_euler_deg(q):
    """四元数组 [n,4] (w,x,y,z, pyulog 顺序) → roll/pitch/yaw (deg)。"""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    roll = np.degrees(np.arctan2(2 * (w * x + y * z),
                                 1 - 2 * (x * x + y * y)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))
    yaw = np.degrees(np.arctan2(2 * (w * z + x * y),
                                1 - 2 * (y * y + z * z)))
    return roll, pitch, yaw


def euler_series(ul, topic_id, quat_field_prefix):
    """从四元数话题出三条欧拉角曲线 (deg)。
    quat_field_prefix: 'q' (vehicle_attitude) 或 'q_d' (attitude_setpoint)。
    返回 [(t, roll, 'roll'), (t, pitch, 'pitch'), (t, yaw, 'yaw')] 或 None。"""
    s = get_series(ul, topic_id, f'{quat_field_prefix}[0]')
    if s is None:
        return None
    t, q0_arr = s
    cols = [get_series(ul, topic_id, f'{quat_field_prefix}[{i}]')
            for i in range(1, 4)]
    if any(c is None for c in cols):
        return None
    q = np.stack([q0_arr, cols[0][1], cols[1][1], cols[2][1]], axis=1)
    roll, pitch, yaw = _quat_to_euler_deg(q)
    return [(t, roll, 'roll'), (t, pitch, 'pitch'), (t, yaw, 'yaw')]


# ---------- 消息 / 参数 ----------

def logged_messages(ul, limit=5000):
    """飞控日志消息 [(t秒, 级别, 文本)], 时间正序, 超长取最新 limit 条。"""
    msgs = [(m.timestamp / 1e6, m.log_level_str(), m.message)
            for m in ul.logged_messages]
    return msgs[-limit:]


def filter_params(ul, keyword=''):
    """初始参数 [(名字, 值)], 名字含关键字 (大小写不敏感) 过滤。"""
    kw = keyword.lower()
    items = ul.initial_parameters.items()
    if kw:
        items = [(k, v) for k, v in items if kw in k.lower()]
    return sorted(items)


# ---------- CLI 自测 ----------

def _main():
    import sys
    root = '/root/flylab/px4.1.15.2/build/px4_sitl_default/rootfs/log'
    path = sys.argv[1] if len(sys.argv) > 1 else (scan_log_dir(root) or [None])[0]
    if not path or not os.path.isfile(path):
        print('用法: python3 ulg_analyzer.py <file.ulg>')
        return
    ul = load_ulog(path)
    print(ulog_info(ul, path))
    d = hover_diagnosis(ul)
    print('悬停诊断:', d['note'])
    for k in ('f_dom_z', 'f_dom_vz'):
        if d[k] is not None:
            print(f'  {k} = {d[k]:.3f} Hz')


if __name__ == '__main__':
    _main()
