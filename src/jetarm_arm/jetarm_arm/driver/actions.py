#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
JetArm 预设动作组播放器

动作定义在 actions.json，本脚本负责：
    1. 播放前校验每个目标值是否在 safe_limits 内（超出直接拒绝，不发指令）
    2. 播放前确保舵机已锁力
    3. 逐帧下发，帧间留极小间隔；支持循环
    4. Ctrl+C 立即停止并锁力

用法：
    python3 actions.py list
    python3 actions.py play wave
    python3 actions.py play dance --loop
    python3 actions.py playall
    python3 actions.py home
"""

import argparse
import json
import os
import sys
import time

import jetarm

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_FILE = os.path.join(HERE, "actions.json")


def load_def(path=DEFAULT_FILE):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def validate(doc, name):
    """检查动作的所有帧是否越界，返回问题列表。"""
    limits = {int(k): v for k, v in doc["safe_limits"].items()}
    action = doc["actions"][name]
    problems = []
    for fi, frame in enumerate(action["frames"]):
        for sid_s, pos in frame["pos"].items():
            sid = int(sid_s)
            lo, hi = limits.get(sid, (0, 1000))
            if not (lo <= pos <= hi):
                problems.append("第%d帧 ID=%d(%s) 目标 %d 超出安全范围 %d~%d"
                                % (fi + 1, sid, jetarm.joint_name(sid), pos, lo, hi))
    return problems


def play_action(arm, doc, name, loop=False, gap=0.02, verbose=True, speed=0.0):
    action = doc["actions"][name]
    frames = action["frames"]
    limits = {int(k): v for k, v in doc["safe_limits"].items()}

    # 安全值钳制（双保险：即便 JSON 被改坏也不会发出越界指令）
    for frame in frames:
        for sid_s in frame["pos"]:
            sid = int(sid_s)
            lo, hi = limits[sid]
            if frame["pos"][sid_s] < lo:
                frame["pos"][sid_s] = lo
            if frame["pos"][sid_s] > hi:
                frame["pos"][sid_s] = hi

    if verbose:
        print("播放 '%s'：%s（%d 帧）" % (name, action.get("desc", ""), len(frames)))

    while True:
        for i, frame in enumerate(frames):
            targets = {int(k): int(v) for k, v in frame["pos"].items()}
            nominal = int(frame.get("duration", 600))
            if speed and speed > 0:
                dur = max(280, int(nominal * speed))
            else:
                cur = {s: arm.read_position(s) for s in targets}
                delta = max(abs(targets[s] - (cur[s] if cur[s] is not None else targets[s]))
                            for s in targets)
                dur = int(max(280, min(2000, delta * 3.0)))
            if verbose:
                print("  [%d/%d] %4dms  %s" % (
                    i + 1, len(frames), dur,
                    " ".join("%s=%d" % (jetarm.joint_name(k), v)
                             for k, v in sorted(targets.items()))))
            arm.move(targets, dur)
            time.sleep(dur / 1000.0 + gap)
        if not loop:
            break
    if verbose:
        print("  完成")


def cmd_list(args):
    doc = load_def(args.file)
    print("动作列表（%s）\n" % args.file)
    print("%-12s %-6s %s" % ("名称", "帧数", "说明"))
    print("-" * 70)
    for name, a in doc["actions"].items():
        print("%-12s %-6d %s" % (name, len(a["frames"]), a.get("desc", "")))
    print("\n安全范围：")
    for sid, (lo, hi) in sorted(doc["safe_limits"].items(), key=lambda x: int(x[0])):
        print("  ID%-3s %-9s %d ~ %d" % (sid, jetarm.joint_name(int(sid)), lo, hi))


def cmd_play(args):
    doc = load_def(args.file)
    if args.name not in doc["actions"]:
        sys.exit("没有动作 '%s'，可用：%s" % (args.name, ", ".join(doc["actions"])))

    problems = validate(doc, args.name)
    if problems:
        print("拒绝播放，存在越界：")
        for p in problems:
            print("  " + p)
        sys.exit(1)

    with jetarm.JetArm(timeout=args.timeout) as arm:
        print("锁力 ...")
        for sid in [1, 2, 3, 4, 5, 10]:
            arm.load(sid)
            time.sleep(0.03)
        time.sleep(0.5)
        try:
            play_action(arm, doc, args.name, loop=args.loop, speed=getattr(args, "speed", 0.0))
        except KeyboardInterrupt:
            print("\n已中断")
        finally:
            for sid in [1, 2, 3, 4, 5, 10]:
                arm.load(sid)


def cmd_playall(args):
    doc = load_def(args.file)
    names = list(doc["actions"].keys())
    with jetarm.JetArm(timeout=args.timeout) as arm:
        print("锁力 ...")
        for sid in [1, 2, 3, 4, 5, 10]:
            arm.load(sid)
            time.sleep(0.03)
        time.sleep(0.5)
        for name in names:
            problems = validate(doc, name)
            if problems:
                print("跳过 '%s'（越界）" % name)
                continue
            try:
                play_action(arm, doc, name, speed=getattr(args, "speed", 0.0))
            except KeyboardInterrupt:
                print("\n已中断")
                break


def cmd_home(args):
    doc = load_def(args.file)
    home = {int(k): int(v) for k, v in doc["home"].items()}
    with jetarm.JetArm(timeout=args.timeout) as arm:
        print("回 home 位姿 ...")
        for sid in home:
            arm.load(sid)
            time.sleep(0.03)
        arm.move(home, 1500)
        time.sleep(2.0)
        for sid in sorted(home):
            print("  ID%-3d %-9s %s" % (sid, jetarm.joint_name(sid), arm.read_position(sid)))


def main():
    p = argparse.ArgumentParser(description="JetArm 预设动作组播放器")
    p.add_argument("--port", default=jetarm.DEFAULT_PORT)
    p.add_argument("--baud", type=int, default=jetarm.DEFAULT_BAUD)
    p.add_argument("--timeout", type=float, default=0.3)
    p.add_argument("-f", "--file", default=DEFAULT_FILE)
    sub = p.add_subparsers(dest="cmd", required=True)

    sp = sub.add_parser("list", help="列出所有动作")
    sp.set_defaults(func=cmd_list)

    sp = sub.add_parser("play", help="播放一个动作")
    sp.add_argument("name")
    sp.add_argument("--loop", action="store_true")
    p.add_argument("--speed", type=float, default=0.0,
                   help="0=最快(约3ms/刻度)；0.5=一半速度；1.0=原设计速度")
    sp.set_defaults(func=cmd_play)

    sp = sub.add_parser("playall", help="依次播放所有动作")
    sp.set_defaults(func=cmd_playall)

    sp = sub.add_parser("home", help="回到 home 位姿")
    sp.set_defaults(func=cmd_home)

    args = p.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
