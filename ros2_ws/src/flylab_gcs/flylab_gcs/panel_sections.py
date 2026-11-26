#!/usr/bin/env python3
"""算法信息面板 —— 分组注册表 (各算法的数据提取/格式化, 纯逻辑无 Qt 控件)。

通用面板框架 (分组控件/表格渲染/超时置灰) 在 algo_output_panel.py,
本文件只负责"算法 JSON -> 显示文本"这一层: 新算法接入改这里, 不碰框架。

数据 schema 由各算法自定义, 只要求顶层:
  {"src": "<分组key>", ...字段...}

---------- 接入新算法 (三步) ----------
1. 算法节点往 /flylab/algo_info 发 {"src": "<key>", ...} (建议 10Hz 限频;
   参数名 info_topic, 参照 aruco_node.publishInfo)
2. 本文件 SECTIONS 注册一项: title + fields 字段表
   (label, extract[, opts]) —— extract(data) 返回显示文本 (可为 HTML 富
   文本, QLabel 自动识别), None 显示 '—', 格式化逻辑完全自定义;
   opts 可选 {'wrap': N} 让该行换行并固定 N 行高
   (长度逐帧变化的长文本字段必加, 否则宽高随内容撑动右栏布局)
3. config.ALGORITHMS 该算法条目加 'panel': '<key>' —— 运行时面板自动
   显示该分组, 停止自动移除

内置分组:
  aruco    —— aruco_node 最大码摘要
  landing  —— marker_landing_node 降落状态 (视觉反控按钮按下时显示)
  depth    —— yolo_depth_node 深度估计 (伪彩深度图 + 摘要)
  semantic —— yolo_semantic_node 语义分割 (配色类别图 + 类别图例)
  seg      —— yolo_segment_node 实例分割 (前 3 实例并列表格, 按置信度降序)
  pose     —— yolo_pose_node 姿态估计 (前 3 人体并列表格, 按置信度降序)
  detect   —— yolo_detector_node 目标检测 (前 3 目标并列表格, 按置信度降序)
  face     —— face_detect_node 人脸检测 (前 3 张脸并列表格, 按置信度降序;
              标题显示 FaceXFormer 下游任务, "结果"行仅最高置信度脸有值)
  obb      —— yolo_obb_node 旋转框检测 (前 3 旋转框并列表格, 按置信度降序)
  cls      —— yolo_cls_node 图像分类 (top-3 类别并列表格, 按置信度降序)
  track    —— yolo_track_node/ortrack_node 目标追踪 (状态字段卡片, 框选在画面完成)

分组 spec 可带 'image': True —— 分组顶部多一个固定高度 (IMG_ROW_HEIGHT,
常量在 algo_output_panel) 的图片行, 数据里可选字段 "img" (base64 JPEG,
参照 overlay.py image 图元的编码方式) 等比缩放完整放入并居中留边,
任意分辨率/长宽比都自适应, 无 "img" 或超时即清空。
"""
from . import config

# 字段 extract 的返回哨兵: 整行隐藏 (按数据源动态显隐, 如 ortrack 无
# 检测候选时隐藏"检出"行); None 仍显示 '—'
HIDE_ROW = object()

# 目标追踪状态机 -> 界面文案
_TRACK_STATE_TEXT = {
    'IDLE': '未选目标',
    'SELECTED': '已框选',
    'TRACK': '追踪中',
    'LOST': '目标丢失',
}

# 降落状态机 -> 界面文案
_LANDING_STATE_TEXT = {
    'IDLE': '待机',
    'TRACKING': '对齐下降',
    'LANDING': 'PX4 自动降落',
}


def _landing_state_text(data):
    """降落状态 -> 界面文案 (分组标题用)。"""
    if data.get('lost'):
        return f"目标丢失 (悬停 {data.get('lost_t', 0.0):.1f}s)"
    return _LANDING_STATE_TEXT.get(data.get('state', 'IDLE'), '待机')


def _detect_fields(data):
    """aruco_node 最大码摘要 -> {字段名: 文本|None}。"""
    out = {'码号': None, '尺寸': None, '占比': None, '偏移': None,
           '实宽': None, '位置': None,
           '检出': f"{data.get('count', 0)} 个"}
    largest = data.get('largest')
    if largest:
        dx, dy = largest.get('dx', 0.0), largest.get('dy', 0.0)
        wm = largest.get('wm', -1.0)
        out['码号'] = f"id={largest.get('id', -1)}"
        out['尺寸'] = f"{largest.get('w', 0)}x{largest.get('h', 0)}px"
        out['占比'] = f"{100.0 * largest.get('area', 0.0):.1f}%"
        out['偏移'] = f'({dx:+.2f}, {dy:+.2f})'
        out['实宽'] = f'{wm:.2f}m' if wm and wm > 0 else '— (无位姿)'
        out['位置'] = _tvec_text(largest)
    return out


def _tvec_text(data):
    """tvec=[x,y,z] (相机系, x右 y下 z前, 米) -> 显示文本; 无位姿返回占位。"""
    tvec = data.get('tvec')
    if tvec and len(tvec) == 3:
        return f'({tvec[0]:+.2f}, {tvec[1]:+.2f}, {tvec[2]:.2f})'
    return '— (无位姿)'


def _landing_fields(data):
    """marker_landing_node 降落状态 -> {字段名: 文本|None}。"""
    state = data.get('state', 'IDLE')
    out = {
        '码号': None,
        '尺寸': None,
        '实宽': None,
        '位置': None,
        '误差': None,
        '码距': None,
        '占比': None,
        '趋势角': None,
        '航向': None,
        '速度': None,
    }
    tid = data.get('tracked_id', -1)
    out['码号'] = f'id={tid}' if tid >= 0 else None
    if state == 'TRACKING' and not data.get('lost'):
        w, h_px = data.get('w', 0), data.get('h', 0)
        wm = data.get('wm', -1.0)
        if w and h_px:
            out['尺寸'] = f'{w}x{h_px}px'
            out['实宽'] = f'{wm:.2f}m' if wm and wm > 0 else '— (无位姿)'
        out['位置'] = _tvec_text(data)
        mode3d = data.get('mode') == 'pose3d'
        out['误差'] = (
            f"{data.get('err', 0.0):.2f}m "
            f"({'达标' if data.get('aligned') else '未达标'})" if mode3d else
            f"{data.get('err', 0.0):.3f} "
            f"({'达标' if data.get('aligned') else '未达标'})")
        out['码距'] = (
            f"{data.get('h_m', -1.0):.2f}m" if mode3d else None)
        out['占比'] = f"{100.0 * data.get('area', 0.0):.1f}%"
        trend = data.get('trend')
        out['趋势角'] = f'{trend:.1f}°' if trend is not None else None
        if data.get('yaw_en'):
            if data.get('yaw_locked'):
                out['航向'] = f"已锁定 (偏置 {data.get('yaw_off', 0.0):.0f}°)"
            else:
                out['航向'] = (
                    f"{data.get('yaw_err', 0.0):+.1f}° "
                    f"({'对齐中' if data.get('aligned') else '待对齐'})")
        vel = data.get('vel')
        out['速度'] = (
            f'({vel[0]:+.2f}, {vel[1]:+.2f}, {vel[2]:+.2f})' if vel else None)
    return out


def _depth_fields(data):
    """yolo_depth_node 深度摘要 -> {字段名: 文本|None}。"""
    out = {'范围': None, '中心': None, '平均': None, '画幅': None}
    if 'dmin' in data:
        out['范围'] = \
            f"{data.get('dmin', 0.0):.2f} ~ {data.get('dmax', 0.0):.2f} m"
        out['中心'] = f"{data.get('dctr', 0.0):.2f} m"
        out['平均'] = f"{data.get('dmean', 0.0):.2f} m"
    w, h = data.get('w'), data.get('h')
    if w and h:
        out['画幅'] = f'{w}x{h}'
    return out


def _semantic_fields(data):
    """yolo_semantic_node 分割摘要 -> {字段名: 文本|None}。"""
    out = {'检出': None, '图例': None, '画幅': None}
    n = data.get('nclass')
    if n is not None:
        out['检出'] = f'{n} 类'
    present = data.get('present')
    if present:
        # 一个类别一行, 只显示占比最大的前 3 行 (present 已按占比降序);
        # 类别名前涂同色小方块 (RGB 由节点随数据下发, 与类别图同 cityscapes
        # 配色), 富文本渲染, 行高仍由 {'wrap': 3} 固定
        rows = []
        for p in present[:3]:
            name, share = p[0], p[1]
            text = f'{name} {share:.1f}%'
            if len(p) >= 3 and len(p[2]) == 3:
                r, g, b = p[2]
                text = (f'<span style="background-color:rgb({r},{g},{b});">'
                        '&nbsp;&nbsp;&nbsp;</span>&nbsp;' + text)
            rows.append(text)
        out['图例'] = '<br>'.join(rows)
    w, h = data.get('w'), data.get('h')
    if w and h:
        out['画幅'] = f'{w}x{h}'
    return out


def _track_fields(data):
    """追踪节点状态 -> {字段名: 文本|None|HIDE_ROW}。"""
    out = {'目标': None, '相似度': None, '直线距': HIDE_ROW,
           '水平距': HIDE_ROW, '尺寸': None, '跟随': HIDE_ROW, '检出': HIDE_ROW}
    # 云台跟随状态 (追踪节点伺服开关, 旧节点无 follow 字段整行隐藏)
    follow = data.get('follow')
    if follow is not None:
        out['跟随'] = '开启' if follow else '关闭'
    if data.get('cls'):
        out['目标'] = f"{data['cls']} {data.get('conf', 0.0):.2f}"
    sim = data.get('sim')
    if sim is not None and data.get('state') in ('TRACK', 'LOST'):
        out['相似度'] = f'{sim:.2f}'
    # 当前目标距离 (米): 直线距 dist = 高度/sin视线俯角 (沿视线到目标),
    # 水平距 dist_h = 高度/tan视线俯角; 视线俯角与飞控位置都有效时节点
    # 随帧发布, 缺失 (未升空/视线近水平/丢失) 整行隐藏
    dist = data.get('dist')
    if dist is not None:
        out['直线距'] = f'{dist:.1f}m'
    dist_h = data.get('dist_h')
    if dist_h is not None:
        out['水平距'] = f'{dist_h:.1f}m'
    w, h = data.get('w'), data.get('h')
    if w and h:
        out['尺寸'] = f'{w}x{h}px'
    # 检出数仅 yolo_track 有意义 (逐帧检测候选数); ortrack 无检测候选
    # 不发 count, 整行隐藏
    count = data.get('count')
    if count is not None:
        out['检出'] = f'{count} 个'
    return out


# CelebA 40 属性中文名 (顺序与 face_detect_node 模型输出一致)
_CELEBA_ATTRS_ZH = [
    '胡渣', '弓形眉', '有魅力', '眼袋', '秃头', '刘海', '厚唇', '大鼻',
    '黑发', '金发', '画面模糊', '棕发', '浓眉', '圆脸', '双下巴', '眼镜',
    '山羊胡', '灰白头发', '浓妆', '高颧骨', '男性', '微张嘴', '八字胡',
    '细长眼', '无须', '瓜子脸', '苍白肤色', '尖鼻', '发际线后移',
    '红润脸颊', '鬓角', '微笑', '直发', '卷发', '戴耳环', '戴帽',
    '涂口红', '戴项链', '戴领带', '年轻',
]


def _face_result_text(item):
    """face_detect_node 最高置信度脸的任务结果 -> 显示文本。
    attributes 传开启属性索引表 (attrs), 其余任务传现成文本 (result)。"""
    attrs = item.get('attrs')
    if attrs is not None:
        names = [_CELEBA_ATTRS_ZH[i] for i in attrs
                 if isinstance(i, int) and 0 <= i < len(_CELEBA_ATTRS_ZH)]
        if not names:
            return '无开启属性'
        head = '、'.join(names[:4])
        more = len(names) - 4
        return f'{head} 等 {len(names)} 项' if more > 0 else head
    return item.get('result')


# 分组注册表: key (= 消息 src) -> {title, title_fn, fields} 或表格模式
# {table: 列数, rows: [(属性, extract(item, col))]}
# fields: [(label, extract)] —— extract(data) -> 文本 | None (显示 '—')
# | HIDE_ROW (整行隐藏)
# 行序在并列显示的分组间保持语义对应 (同行 = 同类信息), 新分组接入时照此排列
SECTIONS = {
    'aruco': {
        'title': 'ArUco 检测',
        'fields': [
            ('码号', lambda d: _detect_fields(d)['码号']),
            ('尺寸', lambda d: _detect_fields(d)['尺寸']),
            ('实宽', lambda d: _detect_fields(d)['实宽']),
            ('位置', lambda d: _detect_fields(d)['位置']),
            ('占比', lambda d: _detect_fields(d)['占比']),
            ('偏移', lambda d: _detect_fields(d)['偏移']),
            ('检出', lambda d: _detect_fields(d)['检出']),
        ],
    },
    'fractal': {
        'title': 'fractal 检测',
        # largest 与 aruco 同构 (字段提取复用 _detect_fields);
        # 模式行表达 Case1/Case2: kp=True 时无层轮廓, 位姿由关键点跟踪接管
        'fields': [
            ('模式', lambda d: 'KP 跟踪' if d.get('kp') else '检测'),
            ('码号', lambda d: _detect_fields(d)['码号']),
            ('尺寸', lambda d: _detect_fields(d)['尺寸']),
            ('实宽', lambda d: _detect_fields(d)['实宽']),
            ('位置', lambda d: _detect_fields(d)['位置']),
            ('占比', lambda d: _detect_fields(d)['占比']),
            ('偏移', lambda d: _detect_fields(d)['偏移']),
            ('检出', lambda d: _detect_fields(d)['检出']),
        ],
    },
    'landing': {
        'title': 'ArUco 降落',
        # 状态不占行位, 动态显示在分组标题里 (对齐下降/目标丢失/…)
        'title_fn': lambda d: (
            'ArUco 降落 · ' + _landing_state_text(d)) if d else 'ArUco 降落',
        # 标题随反控降落的检测器类型变化: 需要 fractal 版时用 landing_spec() 工厂
        # (见下方 landing_spec, AlgoOutputPanel 按运行中算法选用)
        'fields': [
            ('码号', lambda d: _landing_fields(d)['码号']),
            ('尺寸', lambda d: _landing_fields(d)['尺寸']),
            ('实宽', lambda d: _landing_fields(d)['实宽']),
            ('位置', lambda d: _landing_fields(d)['位置']),
            ('占比', lambda d: _landing_fields(d)['占比']),
            ('误差', lambda d: _landing_fields(d)['误差']),
            ('码距', lambda d: _landing_fields(d)['码距']),
            ('趋势角', lambda d: _landing_fields(d)['趋势角']),
            ('航向', lambda d: _landing_fields(d)['航向']),
            ('速度', lambda d: _landing_fields(d)['速度']),
        ],
    },
    'depth': {
        'title': '深度估计',
        'image': True,   # 分组顶部伪彩深度图行 (数据 "img" base64 JPEG)
        'fields': [
            ('范围', lambda d: _depth_fields(d)['范围']),
            ('中心', lambda d: _depth_fields(d)['中心']),
            ('平均', lambda d: _depth_fields(d)['平均']),
            ('画幅', lambda d: _depth_fields(d)['画幅']),
        ],
    },
    'semantic': {
        'title': '语义分割',
        'image': True,   # 分组顶部 cityscapes 配色类别图行 (数据 "img" base64 PNG)
        'fields': [
            ('检出', lambda d: _semantic_fields(d)['检出']),
            # 图例长度逐帧变化: 固定 3 行高防右栏布局跳动 (见 _Section opts)
            ('图例', lambda d: _semantic_fields(d)['图例'], {'wrap': 3}),
            ('画幅', lambda d: _semantic_fields(d)['画幅']),
        ],
    },
    'seg': {
        'title': '实例分割',
        # 表格模式: 前 3 个实例并列成列 (节点端按置信度降序), 目标数不足整列隐藏;
        # 检出数为汇总行, 置于表格上方跨全部列
        'table': 3,
        'summary': ('检出', lambda d: f"{d.get('count', 0)} 个"),
        'rows': [
            ('类别', lambda item, i: item.get('cls') or None),
            ('置信度', lambda item, i: f"{item.get('conf', 0):.2f}"),
            ('位置', lambda item, i:
                f"({item.get('x', 0):.0f}, {item.get('y', 0):.0f})"),
            ('尺寸', lambda item, i:
                f"{item.get('w', 0):.0f}x{item.get('h', 0):.0f}"),
            ('占比', lambda item, i: f"{item.get('area', 0):.1f}%"),
        ],
    },
    'pose': {
        'title': '姿态估计',
        # 表格模式: 前 3 个人体并列成列 (节点端按置信度降序), 目标数不足整列
        # 隐藏; 检出数为汇总行置于表格上方跨全部列 (单类 person 无类别行)
        'table': 3,
        'summary': ('检出', lambda d: f"{d.get('count', 0)} 人"),
        'rows': [
            # 编号 = 列序 (1 起始), 与画面上人体框左上角的编号对应
            ('编号', lambda item, i: str(i + 1)),
            ('置信度', lambda item, i: f"{item.get('conf', 0):.2f}"),
            ('位置', lambda item, i:
                f"({item.get('x', 0):.0f}, {item.get('y', 0):.0f})"),
            ('尺寸', lambda item, i:
                f"{item.get('w', 0):.0f}x{item.get('h', 0):.0f}"),
            ('可见点', lambda item, i:
                f"{item.get('kpts', 0)}/{item.get('kpts_total', 17)}"),
        ],
    },
    'face': {
        'title': '人脸检测',
        # 表格模式: 前 3 张脸并列成列 (节点端按置信度降序), 不足整列隐藏;
        # 人脸数为汇总行置于表格上方跨全部列 (单类 face 无类别行);
        # 标题动态显示当前 FaceXFormer 任务 (人脸检测 · 头部姿态 …);
        # 'none' = 无任务 (仅检测), 标题保持"人脸检测"
        'title_fn': lambda d: (
            '人脸检测 · ' + config.FACE_TASKS.get(
                d.get('task'), d.get('task', '')))
            if d and d.get('task') not in (None, '', 'none') else '人脸检测',
        'table': 3,
        'summary': ('检出', lambda d: f"{d.get('count', 0)} 张"),
        'rows': [
            # 编号 = 列序 (1 起始), 与画面上人脸框左上角的编号对应
            ('编号', lambda item, i: str(i + 1)),
            ('置信度', lambda item, i: f"{item.get('conf', 0):.2f}"),
            ('位置', lambda item, i:
                f"({item.get('x', 0):.0f}, {item.get('y', 0):.0f})"),
            ('尺寸', lambda item, i:
                f"{item.get('w', 0):.0f}x{item.get('h', 0):.0f}"),
            ('占比', lambda item, i: f"{item.get('area', 0):.1f}%"),
            # FaceXFormer 任务结果 (跑过任务的脸各挂自己的, 其余列 '—')
            ('结果', lambda item, i: _face_result_text(item)),
        ],
    },
    'detect': {
        'title': '目标检测',
        # 表格模式: 前 3 个目标并列成列 (节点端按置信度降序), 目标数不足
        # 整列隐藏; 检出数为汇总行置于表格上方跨全部列 (多类别, 有类别行)
        'table': 3,
        'summary': ('检出', lambda d: f"{d.get('count', 0)} 个"),
        'rows': [
            ('类别', lambda item, i: item.get('cls') or None),
            ('置信度', lambda item, i: f"{item.get('conf', 0):.2f}"),
            ('位置', lambda item, i:
                f"({item.get('x', 0):.0f}, {item.get('y', 0):.0f})"),
            ('尺寸', lambda item, i:
                f"{item.get('w', 0):.0f}x{item.get('h', 0):.0f}"),
            ('占比', lambda item, i: f"{item.get('area', 0):.1f}%"),
        ],
    },
    'obb': {
        'title': '旋转框检测',
        # 表格模式: 前 3 个旋转框并列成列 (节点端按置信度降序), 目标数不足
        # 整列隐藏; 检出数为汇总行置于表格上方跨全部列 (多类别, 有类别行);
        # 角度 = RotatedRect::angle (度, 画面坐标系下相对水平的倾斜角)
        'table': 3,
        'summary': ('检出', lambda d: f"{d.get('count', 0)} 个"),
        'rows': [
            ('类别', lambda item, i: item.get('cls') or None),
            ('置信度', lambda item, i: f"{item.get('conf', 0):.2f}"),
            ('位置', lambda item, i:
                f"({item.get('x', 0):.0f}, {item.get('y', 0):.0f})"),
            ('尺寸', lambda item, i:
                f"{item.get('w', 0):.0f}x{item.get('h', 0):.0f}"),
            ('角度', lambda item, i: f"{item.get('angle', 0):.1f}°"),
        ],
    },
    'cls': {
        'title': '图像分类',
        # 表格模式: top-3 类别并列成列 (节点端按置信度降序, 整图分类无
        # 检出数, 无汇总行); 画面无叠加标注, 分类结果只在此显示
        'table': 3,
        'rows': [
            ('名次', lambda item, i: str(i + 1)),
            ('类别', lambda item, i: item.get('cls') or None),
            ('置信度', lambda item, i: f"{item.get('conf', 0):.2f}"),
        ],
    },
    'wilor': {
        'title': '手部姿态',
        # 顶部图片行: 节点端 MANO 网格 z-buffer 着色渲染小图 ("img" base64 JPEG)
        'image': True,
        # 表格模式: 最多 2 只手并列成列 (节点端按检出顺序), 单手只显示一列;
        # 手数为汇总行置于表格上方跨全部列
        'table': 2,
        'summary': ('检出', lambda d: f"{d.get('count', 0)} 只手"),
        'rows': [
            ('左右手', lambda item, i: item.get('side') or None),
            ('置信度', lambda item, i: f"{item.get('conf', 0):.2f}"),
            ('位置', lambda item, i:
                f"({item.get('x', 0):.0f}, {item.get('y', 0):.0f})"),
            ('尺寸', lambda item, i:
                f"{item.get('w', 0):.0f}x{item.get('h', 0):.0f}"),
        ],
    },
    'track': {
        'title': '目标追踪',
        # 状态不占行位, 动态显示在分组标题里 (未选目标/已框选/追踪中/目标丢失)
        'title_fn': lambda d: (
            '目标追踪 · ' + _TRACK_STATE_TEXT.get(
                d.get('state', 'IDLE'), '未选目标')) if d else '目标追踪',
        'fields': [
            ('目标', lambda d: _track_fields(d)['目标']),
            ('相似度', lambda d: _track_fields(d)['相似度']),
            ('直线距', lambda d: _track_fields(d)['直线距']),
            ('水平距', lambda d: _track_fields(d)['水平距']),
            ('尺寸', lambda d: _track_fields(d)['尺寸']),
            ('跟随', lambda d: _track_fields(d)['跟随']),
            ('检出', lambda d: _track_fields(d)['检出']),
        ],
    },
}


def landing_spec(kind='aruco'):
    """landing 分组 spec 工厂: 标题随反控降落的检测器类型 (ArUco/fractal),
    字段提取共用。SECTIONS['landing'] 即 aruco 默认版, fractal 降落由
    AlgoOutputPanel 按运行中算法经本工厂取 spec。"""
    base = 'fractal 降落' if kind == 'fractal' else 'ArUco 降落'
    spec = dict(SECTIONS['landing'])
    spec['title'] = base
    spec['title_fn'] = (
        lambda d: (base + ' · ' + _landing_state_text(d)) if d else base)
    return spec
