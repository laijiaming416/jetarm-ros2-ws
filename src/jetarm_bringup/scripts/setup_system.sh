#!/bin/bash
# 启动后的小调整（可重复执行）：
#   1. USB 音箱音量拉满（重启后会复位到 14/30）
#   2. 网页面板是展示类节点，降优先级，给底盘和导航让 CPU
amixer -c 0 cset numid=6 20 >/dev/null 2>&1 || true

for pid in $(pgrep -f "^/usr/bin/python3 .*/install/mecanum_nav/lib/mecanum_nav/map_web_view"); do
    renice 10 -p "$pid" >/dev/null 2>&1 || true
done