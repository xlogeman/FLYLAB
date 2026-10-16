#!/usr/bin/env python3
"""模型输出叠加接口 —— 任何模型的输出都通过统一 schema 叠加到画面上。

模型节点用法：每帧（或结果变化时）往 config.ANNOTATION_TOPIC 发一条
std_msgs/String，内容是标注列表的 JSON。坐标一律为原图像素坐标。

schema（列表，每项一个标注，未知 type 会被忽略，向后兼容）：

  {"type": "bbox",  "box": [x, y, w, h], "label": "car", "color": [0, 255, 0]}
      目标框，label 可选，显示在框上方；color 可选 (RGB)
  {"type": "point", "point": [x, y],     "label": "...", "color": [255, 0, 0]}
      关键点/中心点
  {"type": "text",  "pos": [x, y],       "text": "fps 30", "color": ...}
      自由文字

示例（检测节点发布）：
  annos = [{"type": "bbox", "box": [100, 200, 80, 60], "label": "car"}]
  pub.publish(String(data=json.dumps(annos)))

显示优先取最新一条消息（整体替换，不是累加），模型停止发消息前可以发 [] 清屏。
"""
import json


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
