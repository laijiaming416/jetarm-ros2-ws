#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""性能测量工具：从运行日志统计延迟，并实测视觉预览与系统资源。

用法：
    python3 measure_perf.py                 # 只做日志统计 + 资源采样
    python3 measure_perf.py --cv 10         # 额外实测 10 次 /grasp/preview
    python3 measure_perf.py --logs "/tmp/pickup*.log" --out /tmp/perf.md

输出：Markdown 表格，可直接粘进《测试报告》。
"""

import argparse
import glob
import os
import re
import time

TS = re.compile(r'\[(\d{10}\.\d+)\]')
INFER = re.compile(r'本句录音([\d.]+)秒；推理([\d.]+)秒')
APLAY = re.compile(r'aplay (\S+).*?(\d+\.\d+)s')


def read_lines(patterns):
    files = []
    for pat in patterns:
        files.extend(sorted(glob.glob(pat)))
    out = []
    for f in files:
        base = os.path.basename(f)
        for line in open(f, encoding='utf-8', errors='replace'):
            m = TS.search(line)
            out.append((float(m.group(1)) if m else None, base, line.rstrip()))
    return out


def log_metrics(rows):
    infer, wake_cue, play = [], [], []
    talking = None
    for ts, base, line in rows:
        m = INFER.search(line)
        if m:
            infer.append(float(m.group(2)))
        if '识别中，请暂时不要说下一句' in line:
            talking = ts
        elif talking is not None and '识别文字' in line and ts:
            infer.append(round(ts - talking, 3))
            talking = None
        if '唤醒词命中' in line and ts:
            wake_cue.append([ts, None])
        if wake_cue and wake_cue[-1][1] is None and 'aplay cue.wav' in line and ts:
            wake_cue[-1][1] = round(ts - wake_cue[-1][0], 3)
        m = APLAY.search(line)
        if m:
            play.append(float(m.group(2)))
    wake = [x[1] for x in wake_cue if x[1] is not None]
    return infer, wake, play


def cv_latency(count):
    import rclpy
    from jetarm_interfaces.srv import GraspPreview
    rclpy.init()
    node = rclpy.create_node('perf_cv')
    cli = node.create_client(GraspPreview, '/grasp/preview')
    if not cli.wait_for_service(timeout_sec=10.0):
        print('!! /grasp/preview 未就绪')
        node.destroy_node(); rclpy.shutdown()
        return []
    out = []
    for i in range(count):
        req = GraspPreview.Request()
        req.hue = 118
        req.height_mm = 70.0
        req.min_area = 8000.0
        t0 = time.monotonic()
        fut = cli.call_async(req)
        while not fut.done() and time.monotonic() - t0 < 30:
            rclpy.spin_once(node, timeout_sec=0.05)
        dt = time.monotonic() - t0
        ok = fut.done() and fut.result().ok
        print('  CV #%d  %.2fs  %s' % (i + 1, dt, 'ok' if ok else 'fail'))
        if ok:
            out.append(dt)
        time.sleep(0.5)
    node.destroy_node(); rclpy.shutdown()
    return out


def resource_sample(seconds=6.0):
    hz = os.sysconf('SC_CLK_TCK')
    cpu_total = []
    for line in open('/proc/stat'):
        if line.startswith('cpu '):
            vals = [int(x) for x in line.split()[1:]]
            cpu_total = vals
            break

    def snap():
        d = {}
        for pid in os.listdir('/proc'):
            if not pid.isdigit():
                continue
            try:
                f = open('/proc/%s/stat' % pid).read().split()
                d[pid] = (int(f[13]) + int(f[14]),
                          ' '.join(f[1:2]).strip('()'))
            except Exception:
                continue
        return d

    a = snap()
    tot0 = sum(cpu_total)
    idle0 = cpu_total[3] + cpu_total[4]
    time.sleep(seconds)
    b = snap()
    cpu_total = []
    for line in open('/proc/stat'):
        if line.startswith('cpu '):
            cpu_total = [int(x) for x in line.split()[1:]]
            break
    tot1, idle1 = sum(cpu_total), cpu_total[3] + cpu_total[4]
    busy = 1.0 - (idle1 - idle0) / max(1, (tot1 - tot0))
    rows = []
    for pid, (t, name) in b.items():
        if pid in a:
            rows.append(((t - a[pid][0]) / hz / seconds * 100.0, name, pid))
    rows.sort(reverse=True)
    return busy * 100.0, rows[:10]


def stats(vals):
    if not vals:
        return None
    vals = sorted(vals)
    return {
        'n': len(vals),
        'min': vals[0],
        'max': vals[-1],
        'avg': round(sum(vals) / len(vals), 3),
        'p50': round(vals[len(vals) // 2], 3),
    }


def fmt_stat(name, vals, unit='s'):
    s = stats(vals)
    if not s:
        return '| %s | 0 | - | - | - | - |' % name
    return '| %s | %d | %.3f%s | %.3f%s | %.3f%s | %.3f%s |' % (
        name, s['n'], s['min'], unit, s['max'], unit, s['avg'], unit, s['p50'], unit)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--logs', nargs='+', default=[
        '/tmp/pickup*.log', '/tmp/full.log', '/tmp/asr.log', '/tmp/tts*.log'])
    ap.add_argument('--cv', type=int, default=0, help='实测 N 次 /grasp/preview')
    ap.add_argument('--out', default='')
    args = ap.parse_args()

    rows = read_lines(args.logs)
    infer, wake, play = log_metrics(rows)
    cv = cv_latency(args.cv) if args.cv > 0 else []
    busy, top = resource_sample()

    lines = []
    lines.append('# 性能测量结果\n')
    lines.append('生成时间：%s\n' % time.strftime('%Y-%m-%d %H:%M:%S'))
    lines.append('## 1. 延迟统计（来自运行日志）\n')
    lines.append('| 指标 | 次数 | 最小 | 最大 | 平均 | 中位 |')
    lines.append('|---|---|---|---|---|---|')
    lines.append(fmt_stat('ASR 单句推理', infer))
    lines.append(fmt_stat('唤醒→提示音', wake))
    lines.append(fmt_stat('TTS 播放时长', play))
    if cv:
        lines.append(fmt_stat('CV /grasp/preview', cv))
    lines.append('')
    lines.append('## 2. 系统资源\n')
    lines.append('系统整体 CPU 占用：**%.1f%%**（采样 6 s）\n' % busy)
    lines.append('| 进程 | CPU% |')
    lines.append('|---|---|')
    for pct, name, _ in top:
        lines.append('| %s | %.1f%% |' % (name, pct))
    lines.append('')

    text = '\n'.join(lines)
    print(text)
    if args.out:
        open(args.out, 'w', encoding='utf-8').write(text)
        print('\n已写入 %s' % args.out)


if __name__ == '__main__':
    main()
