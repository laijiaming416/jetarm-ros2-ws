#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
JetArm 正运动学 / 逆运动学

坐标约定（底座坐标系）：
    X 轴：底座前方（正前方）
    Y 轴：底座左侧
    Z 轴：竖直向上
    原点：底座转轴与安装面的交点

关节定义：
    j1 底座旋转：绕 Z 轴
    j2 肩      ：绕水平轴俯仰，0° = 连杆竖直向上
    j3 肘      ：相对大臂的俯仰，0° = 与上一段保持直线
    j4 腕俯仰  ：相对小臂的俯仰，0° = 与上一段保持直线
    j5 腕旋转  ：绕小臂轴线自转，不影响 TCP 位置
    j10 夹爪   ：开合，不影响 TCP 位置

舵机位置 0~1000 对应 0~240°，所以 1° = 1000/240 = 4.1667 个刻度。

用法：
    python3 kinematics.py fk                 # 读当前姿态算 TCP
    python3 kinematics.py fk --pose 1=508 2=515 3=506 4=520 5=499
    python3 kinematics.py zero               # 把"全竖直姿态"记为零位
    python3 kinematics.py info
"""

import argparse
import json
import math
import os
import sys
import time

import jetarm

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_ROBOT = os.path.join(HERE, "robot.json")


def load_robot(path=DEFAULT_ROBOT):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def save_robot(doc, path=DEFAULT_ROBOT):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(doc, f, ensure_ascii=False, indent=2)


def pos_to_deg(doc, sid, pos):
    """舵机刻度 -> 相对零位的关节角度（度）。"""
    j = doc["joints"][str(sid)]
    return (pos - j["zero"]) / j["units_per_deg"] * j["sign"]


def fk(doc, pose):
    """正运动学。pose: {servo_id: position}。返回 (x, y, z) 单位 mm，以及中间关节坐标。"""
    g = doc["geometry"]
    H0, D = g["H0"], g["D"]
    L1, L2, L3, L4 = g["L1"], g["L2"], g["L3"], g["L4"]

    a1 = math.radians(pos_to_deg(doc, 1, pose[1]))
    a2 = math.radians(pos_to_deg(doc, 2, pose[2]))
    a3 = math.radians(pos_to_deg(doc, 3, pose[3]))
    a4 = math.radians(pos_to_deg(doc, 4, pose[4]))

    # 平面内累加角度（相对竖直向上方向）
    p2 = a2
    p3 = a2 + a3
    p4 = a2 + a3 + a4

    # 肩关节相对底座转轴的水平偏移 D
    sx = D
    sz = H0
    ex = sx + L1 * math.sin(p2)
    ez = sz + L1 * math.cos(p2)
    wx = ex + L2 * math.sin(p3)
    wz = ez + L2 * math.cos(p3)
    # 腕俯仰轴到 TCP：L3 + L4 沿腕部方向
    tcp_r = wx + (L3 + L4) * math.sin(p4)
    tcp_z = wz + (L3 + L4) * math.cos(p4)

    # 绕 Z 轴旋转 a1
    ca, sa = math.cos(a1), math.sin(a1)
    x = tcp_r * ca
    y = tcp_r * sa

    return {
        "tcp": (x, y, tcp_z),
        "shoulder": (sx * ca, sx * sa, sz),
        "elbow": (ex * ca, ex * sa, ez),
        "wrist": (wx * ca, wx * sa, wz),
        "plane_deg": (p2 * 180 / math.pi, p3 * 180 / math.pi, p4 * 180 / math.pi),
        "base_deg": a1 * 180 / math.pi,
    }


def cmd_info(args):
    doc = load_robot(args.robot)
    print("几何参数 (mm):")
    for k, v in doc["geometry"].items():
        print("  %-3s %8.1f" % (k, v))
    print("\n关节标定:")
    print("  %-4s %-10s %8s %6s" % ("ID", "名称", "零位", "方向"))
    for sid in ("1", "2", "3", "4", "5", "10"):
        j = doc["joints"][sid]
        print("  %-4s %-10s %8d %6d" % (sid, j["name"], j["zero"], j["sign"]))
    print("\n已标定: %s" % doc.get("calibrated", False))
    print("\n公式: 角度(度) = (position - zero) / 4.1667 * sign")


def cmd_zero(args):
    """把当前姿态记为零位（要求：所有连杆竖直向上，夹爪朝天）。"""
    doc = load_robot(args.robot)
    with jetarm.JetArm(timeout=args.timeout) as arm:
        pose = {}
        for sid in (1, 2, 3, 4, 5, 10):
            pose[sid] = arm.read_position(sid)
    print("当前姿态: %s" % pose)

    if args.yes:
        for sid in (1, 2, 3, 4, 5, 10):
            if pose[sid] is None:
                sys.exit("ID=%d 读不到位置，标定中止" % sid)
            doc["joints"][str(sid)]["zero"] = pose[sid]
        doc["calibrated"] = True
        if args.signs:
            for sid, s in args.signs.items():
                doc["joints"][str(sid)]["sign"] = s
        save_robot(doc, args.robot)
        print("\n已写入 %s" % os.path.abspath(args.robot))
        print("零位: %s" % {k: doc["joints"][k]["zero"] for k in ("1", "2", "3", "4", "5", "10")})
        print("方向: %s" % {k: doc["joints"][k]["sign"] for k in ("1", "2", "3", "4", "5", "10")})
    else:
        print("\n未写入。确认姿态正确后，加 --yes 执行：")
        print("  python3 kinematics.py zero --yes")


def cmd_fk(args):
    doc = load_robot(args.robot)
    if args.pose:
        pose = {}
        for item in args.pose:
            k, v = item.split("=")
            pose[int(k)] = int(v)
    else:
        with jetarm.JetArm(timeout=args.timeout) as arm:
            pose = {sid: arm.read_position(sid) for sid in (1, 2, 3, 4, 5, 10)}
    missing = [s for s in (1, 2, 3, 4, 5) if pose.get(s) is None]
    if missing:
        sys.exit("缺少关节位置: %s" % missing)

    r = fk(doc, pose)
    x, y, z = r["tcp"]
    print("姿态: %s" % pose)
    print("关节角度: 底座 %.2f°  肩 %.2f°  肘 %.2f°  腕俯仰 %.2f°"
          % (r["base_deg"], r["plane_deg"][0], r["plane_deg"][1], r["plane_deg"][2]))
    print("TCP 坐标 (mm): X=%.1f  Y=%.1f  Z=%.1f" % (x, y, z))
    print("            水平距底座轴 r=%.1f  方位角 %.1f°  高度 %.1f"
          % (math.hypot(x, y), math.degrees(math.atan2(y, x)), z))
    print("\n中间点:")
    for k in ("shoulder", "elbow", "wrist"):
        print("  %-9s X=%.1f Y=%.1f Z=%.1f" % (k, *r[k]))


def deg_to_pos(doc, sid, deg):
    j = doc["joints"][str(sid)]
    return int(round(j["zero"] + deg * j["units_per_deg"] * j["sign"]))


def ik(doc, x, y, z, tool_deg=0.0, elbow="up"):
    """逆运动学。

    参数：
        (x, y, z) : 目标 TCP 坐标，mm，原点在底座转轴与地面交点
        tool_deg  : 末端指向角（绝对），0 = 竖直向上，90 = 水平向前，180 = 竖直向下
        elbow     : "up" / "down" 两种肘部解

    返回 dict，包含各关节角度、舵机刻度、是否越界。
    """
    g = doc["geometry"]
    H0, D = g["H0"], g["D"]
    L1, L2, L3, L4 = g["L1"], g["L2"], g["L3"], g["L4"]
    L34 = L3 + L4

    r = math.hypot(x, y)
    a1 = math.atan2(y, x)
    p4 = math.radians(tool_deg)

    # 反推出腕俯仰轴的位置
    wr = r - L34 * math.sin(p4)
    wz = z - L34 * math.cos(p4)

    dr = wr - D
    dz = wz - H0
    d = math.hypot(dr, dz)
    d_max = L1 + L2 - 1e-6
    d_min = abs(L1 - L2) + 1e-6
    reachable = True
    if d > d_max:
        reachable = False
        d = d_max
    if d < d_min:
        reachable = False
        d = d_min

    a = math.atan2(dr, dz)
    cosb = (L1 * L1 + d * d - L2 * L2) / (2 * L1 * d)
    beta = math.acos(max(-1.0, min(1.0, cosb)))

    sols = {}
    for tag, theta2 in (("up", a - beta), ("down", a + beta)):
        ex = D + L1 * math.sin(theta2)
        ez = H0 + L1 * math.cos(theta2)
        p3 = math.atan2(wr - ex, wz - ez)
        theta3 = p3 - theta2
        a4 = p4 - p3
        sols[tag] = (theta2, theta3, p3, a4)

    if elbow not in sols:
        elbow = "up"
    theta2, theta3, p3, a4 = sols[elbow]

    deg = {
        1: math.degrees(a1),
        2: math.degrees(theta2),
        3: math.degrees(theta3),
        4: math.degrees(a4),
    }
    pos = {sid: deg_to_pos(doc, sid, deg[sid]) for sid in (1, 2, 3, 4)}

    problems = []
    limits = doc.get("limits", {})
    for sid in (1, 2, 3, 4):
        lo, hi = limits.get(str(sid), (0, 1000))
        if not (lo <= pos[sid] <= hi):
            problems.append("ID%d 需要 %d，超出 %d~%d" % (sid, pos[sid], lo, hi))

    return {
        "reachable": reachable,
        "distance_mm": d,
        "deg": deg,
        "pos": pos,
        "problems": problems,
        "alternatives": {k: {s: math.degrees(v[i]) for i, s in zip(range(3), (2, 3, 4))}
                         for k, v in sols.items()},
    }


def cmd_ik(args):
    doc = load_robot(args.robot)
    res = ik(doc, args.x, args.y, args.z, args.tool_deg, args.elbow)
    print("目标 TCP: X=%.1f Y=%.1f Z=%.1f  末端指向 %.1f°  肘部解=%s"
          % (args.x, args.y, args.z, args.tool_deg, args.elbow))
    if not res["reachable"]:
        print("警告：目标超出可达范围，已按最近点求解（距离 %.1fmm）" % res["distance_mm"])
    print("\n关节角度:")
    for sid in (1, 2, 3, 4):
        print("  ID%-3d %-9s %8.2f°" % (sid, doc["joints"][str(sid)]["name"], res["deg"][sid]))
    print("\n舵机刻度:")
    for sid in (1, 2, 3, 4):
        print("  ID%-3d %-9s %5d" % (sid, doc["joints"][str(sid)]["name"], res["pos"][sid]))
    if res["problems"]:
        print("\n越界:")
        for p in res["problems"]:
            print("  " + p)
    else:
        print("\n所有关节在安全范围内。")

    if args.goto:
        if res["problems"]:
            sys.exit("\n存在越界，拒绝执行。可用 --elbow down 或调整 tool_deg 再试。")
        targets = dict(res["pos"])
        targets[5] = args.roll
        targets[10] = doc["joints"]["10"]["open"] if args.open_gripper else \
            doc["joints"]["10"]["close"]
        with jetarm.JetArm(timeout=args.timeout) as arm:
            for sid in targets:
                arm.load(sid)
                time.sleep(0.03)
            print("\n运动到目标 ...")
            arm.move(targets, args.duration)
            time.sleep(args.duration / 1000.0 + 0.3)
            pose = {sid: arm.read_position(sid) for sid in sorted(targets)}
            print("到位姿态: %s" % pose)
            r2 = fk(doc, {**pose, 10: pose.get(10, 600)})
            print("实际 TCP (FK): X=%.1f Y=%.1f Z=%.1f" % r2["tcp"])
            print("误差: %.1f mm" % math.dist((args.x, args.y, args.z), r2["tcp"]))
            if args.hold > 0:
                print("保持 %d 秒 ..." % args.hold)
                time.sleep(args.hold)


def link4_frame(doc, pose):
    """ID4 腕俯仰输出端的坐标系（相机安装基准）。

    原点：腕俯仰轴中心
    Z 轴：该段连杆指向（随腕俯仰转动，因此随 p4）
    Y 轴：腕俯仰转轴方向（水平，垂直于手臂平面）
    X 轴：Y×Z，位于手臂平面内

    返回 (R 3x3, t 3)，t 单位 mm，均在底座坐标系下。
    """
    import numpy as np

    g = doc["geometry"]
    H0, D = g["H0"], g["D"]
    L1, L2 = g["L1"], g["L2"]

    a1 = math.radians(pos_to_deg(doc, 1, pose[1]))
    a2 = math.radians(pos_to_deg(doc, 2, pose[2]))
    a3 = math.radians(pos_to_deg(doc, 3, pose[3]))
    a4 = math.radians(pos_to_deg(doc, 4, pose[4]))

    p2 = a2
    p3 = a2 + a3
    p4 = a2 + a3 + a4

    sx, sz = D, H0
    ex = sx + L1 * math.sin(p2)
    ez = sz + L1 * math.cos(p2)
    wx = ex + L2 * math.sin(p3)
    wz = ez + L2 * math.cos(p3)

    ca, sa = math.cos(a1), math.sin(a1)
    t = np.array([wx * ca, wx * sa, wz], dtype=float)

    z = np.array([math.sin(p4) * ca, math.sin(p4) * sa, math.cos(p4)])
    y = np.array([-sa, ca, 0.0])
    x = np.cross(y, z)
    x /= np.linalg.norm(x)
    y /= np.linalg.norm(y)
    z /= np.linalg.norm(z)
    R = np.column_stack([x, y, z])
    return R, t


def cmd_measure(args):
    """打印一份"人工测量指引"，用于验证 FK。"""
    doc = load_robot(args.robot)
    with jetarm.JetArm(timeout=args.timeout) as arm:
        pose = {sid: arm.read_position(sid) for sid in (1, 2, 3, 4, 5, 10)}
    r = fk(doc, pose)
    x, y, z = r["tcp"]
    print("当前姿态: %s" % pose)
    print("\n请用尺子量以下三项，然后对比：")
    print("  1) 夹爪指尖到桌面的高度      预测 %.1f mm" % z)
    print("  2) 夹爪指尖到【底座转轴中心】的水平距离  预测 %.1f mm" % math.hypot(x, y))
    print("  3) 夹爪朝向：从正前方起，俯视顺时针多少度   预测 %.1f°"
          % math.degrees(math.atan2(y, x)))
    print("\n误差在 ±20mm 以内可以接受；超过说明 H0/L1/L2/L3/L4 或零位有问题。")


def main():
    p = argparse.ArgumentParser(description="JetArm 运动学")
    p.add_argument("--robot", default=DEFAULT_ROBOT)
    p.add_argument("--timeout", type=float, default=0.3)
    sub = p.add_subparsers(dest="cmd", required=True)

    sp = sub.add_parser("info", help="查看几何参数与标定")
    sp.set_defaults(func=cmd_info)

    sp = sub.add_parser("zero", help="把当前姿态记为零位（需摆成全竖直）")
    sp.add_argument("--yes", action="store_true", help="确认写入")
    sp.add_argument("--signs", type=json.loads, default=None,
                    help='方向修正，例如 \'{"2":-1,"3":-1,"4":-1}\'')
    sp.set_defaults(func=cmd_zero)

    sp = sub.add_parser("fk", help="计算 TCP 坐标")
    sp.add_argument("--pose", nargs="*", help="1=508 2=515 ...")
    sp.set_defaults(func=cmd_fk)

    sp = sub.add_parser("measure", help="打印人工测量指引以验证 FK")
    sp.set_defaults(func=cmd_measure)

    sp = sub.add_parser("ik", help="逆运动学：给 XYZ 反解关节角度")
    sp.add_argument("--x", type=float, required=True)
    sp.add_argument("--y", type=float, required=True)
    sp.add_argument("--z", type=float, required=True)
    sp.add_argument("--tool-deg", type=float, default=0.0,
                    help="末端指向角：0=竖直向上 90=水平向前 180=竖直向下")
    sp.add_argument("--elbow", choices=["up", "down"], default="up")
    sp.add_argument("--goto", action="store_true", help="计算后直接运动过去")
    sp.add_argument("--duration", type=int, default=2500)
    sp.add_argument("--hold", type=int, default=0, help="到位后保持秒数")
    sp.add_argument("--roll", type=int, default=498, help="腕旋转刻度")
    sp.add_argument("--open-gripper", action="store_true", help="到位后夹爪张开")
    sp.set_defaults(func=cmd_ik)

    args = p.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
