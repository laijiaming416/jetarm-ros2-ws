# -*- coding: utf-8 -*-
"""视觉几何：从 ~/jetarm/grid_grasp.py 复制的 detect / cam2base / pixel_to_plane。

只保留纯几何/图像部分，去掉相机采集与串口访问；参数改为显式传入，
默认值与 2026-11-06 的 grid_grasp.py 一致（min_area 除外，见下）。
"""

import cv2
import numpy as np

import kinematics as KIN


def detect(frame, hue, tol=20, min_area=8000, min_fill=0.40, max_aspect=2.5):
    """返回 (u, v, area) 或 None，并把最大轮廓挂在 detect.contour 上。"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h = hsv[:, :, 0].astype(np.int32)
    mask = ((h >= hue - tol) & (h <= hue + tol) &
            (hsv[:, :, 1] >= 90) & (hsv[:, :, 2] >= 60)).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < min_area:
        return None
    x, y, w, hh = cv2.boundingRect(c)
    if area / float(w * hh) < min_fill:
        return None
    if w and hh and max(w, hh) / float(min(w, hh)) > max_aspect:
        return None
    m = cv2.moments(c)
    if m['m00'] == 0:
        return None
    detect.contour = c
    return m['m10'] / m['m00'], m['m01'] / m['m00'], area


def cam2base(robot, he, pose):
    """相机坐标系 → 底座坐标系（依赖当前关节位置）。"""
    r_g2b, t_g2b = KIN.link4_frame(robot, pose)
    r = r_g2b @ np.array(he['R_cam2gripper'])
    t = r_g2b @ np.array(he['t_cam2gripper_mm']) + t_g2b
    return r, t


def pixel_to_plane(u, v, kc, dist, r_c2b, t_c2b, z_plane):
    """像素 → 指定高度平面上的底座坐标（mm）。"""
    pts = np.array([[[u, v]]], dtype=np.float32)
    und = cv2.undistortPoints(pts, kc, dist, P=kc).reshape(-1, 2)[0]
    d_c = np.array([(und[0] - kc[0, 2]) / kc[0, 0],
                    (und[1] - kc[1, 2]) / kc[1, 1],
                    1.0])
    d_b = r_c2b @ d_c
    if abs(d_b[2]) < 1e-6:
        return None
    scale = (z_plane - t_c2b[2]) / d_b[2]
    if scale <= 0:
        return None
    return t_c2b + scale * d_b


def annotate(frame, contour, u, v, target_mm):
    """调试用标注图（仅 ASCII 文字，cv2 不能画中文）。"""
    img = frame.copy()
    if contour is not None:
        cv2.drawContours(img, [contour], -1, (0, 255, 0), 2)
    cv2.drawMarker(img, (int(u), int(v)), (0, 0, 255),
                   cv2.MARKER_CROSS, 24, 2)
    if target_mm is not None:
        cv2.putText(img, 'base %.0f, %.0f mm' % (target_mm[0], target_mm[1]),
                    (20, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
    cv2.putText(img, 'px %.0f, %.0f' % (u, v), (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
    return buf.tobytes() if ok else b''