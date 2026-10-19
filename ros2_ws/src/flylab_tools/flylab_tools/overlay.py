#!/usr/bin/env python3
"""模型输出叠加接口 —— 任何算法的输出都通过统一 schema 叠加到画面上。

模型节点用法：每帧（或结果变化时）往 config.ANNOTATION_TOPIC 发一条
std_msgs/String。坐标一律为原图像素坐标。

消息级格式（二选一）：
  新: {"algo": "aruco", "items": [...]}   按算法分组——GUI 各组独立替换/清屏,
                                          多个算法可同时叠加互不干扰
  旧: [...]                               纯列表 (视为无名组, 整体替换), 向后兼容

标注 schema（items 每项一个图元，未知 type 会被忽略，向后兼容）：

  {"type": "bbox",     "box": [x, y, w, h],   "label": "car 0.92", "color": [0,255,0]}
      目标框 (YOLO 用: label 放 类别+置信度)
  {"type": "point",    "point": [x, y],       "label": "...", "color": ...}
      关键点/中心点
  {"type": "polyline", "points": [[x,y],...], "close": false, "color": ...}
      折线/多边形 (ArUco 用: 码四边形轮廓 + RGB 位姿轴; close=true 时首尾闭合)
  {"type": "text",     "pos": [x, y],         "text": "fps 30", "color": ...}
      自由文字

示例（检测节点发布）：
  msg = {"algo": "yolo", "items": [
      {"type": "bbox", "box": [100, 200, 80, 60], "label": "car 0.92"}]}
  pub.publish(String(data=json.dumps(msg)))

显示按组整体替换（不是累加），算法停止前发 {"algo": ..., "items": []} 清自己的屏。
"""
import json


def parse_message(json_str):
    """解析一条标注消息 -> (algo, items)；非法返回 None。
    兼容新旧两种消息级格式 (见模块 docstring)。"""
    try:
        data = json.loads(json_str)
    except (ValueError, TypeError):
        return None
    if isinstance(data, dict):
        algo = str(data.get('algo', ''))
        items = data.get('items')
        if not isinstance(items, list):
            return None
    elif isinstance(data, list):
        algo, items = '', data
    else:
        return None
    return algo, [a for a in (normalize(it) for it in items) if a is not None]


def normalize(anno):
    """校验并规范化一条标注，非法返回 None（单条非法不影响其他标注）。"""
    if not isinstance(anno, dict):
        return None
    t = anno.get('type')
    if t == 'bbox':
        box = anno.get('box')
        if not (isinstance(box, (list, tuple)) and len(box) == 4):
            return None
        return {'type': 'bbox', 'box': [float(v) for v in box],
                'label': str(anno.get('label', '')),
                'color': _color(anno.get('color'))}
    if t == 'point':
        pt = anno.get('point')
        if not (isinstance(pt, (list, tuple)) and len(pt) == 2):
            return None
        return {'type': 'point', 'point': [float(v) for v in pt],
                'label': str(anno.get('label', '')),
                'color': _color(anno.get('color'))}
    if t == 'polyline':
        pts = anno.get('points')
        if not (isinstance(pts, list) and len(pts) >= 2
                and all(isinstance(p, (list, tuple)) and len(p) == 2
                        for p in pts)):
            return None
        return {'type': 'polyline',
                'points': [[float(v) for v in p] for p in pts],
                'close': bool(anno.get('close', False)),
                'color': _color(anno.get('color'))}
    if t == 'text':
        pos = anno.get('pos')
        if not (isinstance(pos, (list, tuple)) and len(pos) == 2):
            return None
        return {'type': 'text', 'pos': [float(v) for v in pos],
                'text': str(anno.get('text', '')),
                'color': _color(anno.get('color'))}
    return None


def _color(c):
    """颜色缺省绿色 (0,255,0)。"""
    if isinstance(c, (list, tuple)) and len(c) == 3:
        return [int(v) for v in c]
    return [0, 255, 0]


def parse_annotations(json_str):
    """解析模型节点发来的 JSON 标注列表，非法项跳过。"""
    try:
        items = json.loads(json_str)
    except (ValueError, TypeError):
        return []
    if not isinstance(items, list):
        return []
    return [a for a in (normalize(it) for it in items) if a is not None]
