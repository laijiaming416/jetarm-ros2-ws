#!/usr/bin/env python3
"""网页地图 / 导航面板。

功能：
    - 地图、车体轮廓（使用 car_outline.json）、雷达点云、全局路径
    - 左键点选/拖动发送导航目标；Shift+左键设置初始位姿
    - 右键平移、滚轮缩放、跟随车体
    - 航点队列（依次发送）、取消目标
    - 保存地图（slam_toolbox save_map）、加载地图（nav2 map_server load_map）
    - 电池电压自动刷新（订阅底盘 /battery_voltage）

浏览器：http://<板卡IP>:8080/
"""
import json
import math
import os
import shutil
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from jetarm_interfaces.msg import MissionState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import LoadMap
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy)
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import SaveMap
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

PAGE = """<!DOCTYPE html>
<html lang="zh">
<head><meta charset="utf-8"><title>麦轮车 导航面板</title>
<style>
  :root{
    --bg:#0e1116; --panel:rgba(17,22,29,.88); --line:rgba(120,150,180,.22);
    --fg:#e3eaf2; --muted:#8fa1b3; --ok:#54d18a; --warn:#ffb454; --bad:#ff6b6b;
  }
  *{box-sizing:border-box}
  html,body{height:100%}
  body{margin:0;background:var(--bg);color:var(--fg);overflow:hidden;
       font:13px/1.55 ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;
       -webkit-user-select:none;user-select:none}
  canvas{position:fixed;inset:0;display:block;cursor:crosshair}
  .panel{background:var(--panel);border:1px solid var(--line);border-radius:12px;
         backdrop-filter:blur(8px);box-shadow:0 8px 24px rgba(0,0,0,.35)}
  #topbar{position:fixed;left:12px;right:12px;top:10px;height:50px;z-index:5;
          display:flex;align-items:center;gap:14px;padding:0 14px}
  .brand{display:flex;align-items:center;gap:8px;font-size:14px;font-weight:600;
         letter-spacing:.04em;white-space:nowrap}
  .brand i{width:8px;height:8px;border-radius:50%;background:var(--ok);
           box-shadow:0 0 8px var(--ok)}
  #modeSwitch{display:flex;gap:3px;padding:3px;border-radius:10px;
              background:#0b0f15;border:1px solid var(--line)}
  #modeSwitch button{height:36px;min-width:96px;border:1px solid transparent;
              border-radius:8px;background:transparent;color:var(--muted);
              font:600 14px ui-monospace,Menlo,Consolas,monospace;cursor:pointer;
              transition:background .15s,color .15s}
  #modeSwitch button:hover{color:var(--fg);background:#1a2431}
  #btnMap.active{background:#1f5f8f;border-color:#2f8fbf;color:#dff0ff}
  #btnNav.active{background:#1f7a4d;border-color:#2f9f6a;color:#d9ffe9}
  #btnNav:disabled,#btnMap:disabled{opacity:.5;cursor:not-allowed}
  #hint{margin-left:auto;color:var(--muted);font-size:12px;white-space:nowrap;
        overflow:hidden;text-overflow:ellipsis}
  #hud{position:fixed;left:12px;top:72px;width:252px;padding:12px 14px;z-index:4;
       pointer-events:none}
  .hud-head{display:flex;justify-content:space-between;align-items:center;
            margin-bottom:8px}
  .hud-title{font-size:11px;color:var(--muted);letter-spacing:.14em}
  .badge{padding:2px 9px;border-radius:999px;font-size:12px;font-weight:600;color:#fff}
  .kv{display:flex;justify-content:space-between;gap:10px;padding:2px 0}
  .kv>span{color:var(--muted);white-space:nowrap}
  .kv>b{font-weight:500;text-align:right}
  .ok{color:var(--ok)}.warn{color:var(--warn)}.bad{color:var(--bad)}
  #panel{position:fixed;right:12px;top:72px;width:218px;padding:10px 12px;z-index:4;
         max-height:calc(100vh - 150px);overflow:auto}
  #panel section+section{border-top:1px dashed var(--line);margin-top:9px;padding-top:9px}
  #panel h3{margin:0 0 7px;font-size:11px;font-weight:600;color:var(--muted);
            letter-spacing:.14em}
  button{height:34px;padding:0 9px;border-radius:8px;border:1px solid #33465c;
         background:#1c2634;color:var(--fg);font:12.5px ui-monospace,Menlo,Consolas,monospace;
         cursor:pointer;transition:background .15s,border-color .15s}
  button:hover{background:#26364a}
  button:active{transform:translateY(1px)}
  button.on{background:#1f5f3f;border-color:#2f8f5f;color:#c8ffe2}
  button:disabled{opacity:.45;cursor:not-allowed}
  button.wide{width:100%;margin-bottom:6px}
  .row2{display:grid;grid-template-columns:1fr 1fr;gap:6px}
  input{width:100%;height:32px;margin-bottom:6px;padding:0 8px;border-radius:8px;
        border:1px solid #33465c;background:#0d141d;color:var(--fg);
        font:inherit}
  #toast{position:fixed;left:50%;bottom:26px;transform:translate(-50%,10px);z-index:6;
         max-width:78vw;padding:9px 16px;border-radius:10px;font-size:13px;
         background:rgba(20,26,34,.96);border:1px solid var(--line);
         opacity:0;pointer-events:none;transition:opacity .2s,transform .2s}
  #toast.show{opacity:1;transform:translate(-50%,0)}
  #toast.err{border-color:#7a2f2f;color:#ffb3b3}
  @media (max-width:900px){
    #hint{display:none}
    #hud{width:216px;padding:10px 12px}
    #toast{top:70px;bottom:auto}
    #panel{left:12px;right:12px;width:auto;top:auto;bottom:12px;display:flex;
           gap:10px;flex-wrap:wrap;max-height:42vh}
    #panel section{flex:1 1 150px}
    #panel section+section{border-top:none;margin:0;padding:0}
  }
</style></head>
<body>
<canvas id="cv"></canvas>

<div id="topbar" class="panel">
  <div class="brand"><i></i>麦轮车</div>
  <div id="modeSwitch">
    <button id="btnMap">建图</button>
    <button id="btnNav">导航</button>
  </div>
  <div id="hint">左键点选目标 · 拖动定朝向 · Shift+左键设初始位姿 · 右键平移 · 滚轮缩放</div>
</div>

<div id="hud" class="panel">等待地图…</div>

<aside id="panel" class="panel">
  <section>
    <h3>视图</h3>
    <div class="row2">
      <button id="btnFollow">跟随: 开</button>
      <button id="btnFit">适配地图</button>
    </div>
  </section>
  <section>
    <h3>导航</h3>
    <button id="btnNavClick" class="wide">点选导航: 关</button>
    <div class="row2">
      <button id="btnCancel">取消目标</button>
      <button id="btnReloc">重新定位</button>
    </div>
  </section>
  <section>
    <h3>地图</h3>
    <input id="mapName" value="room5" placeholder="地图名" title="地图名">
    <div class="row2">
      <button id="btnSaveMap">保存地图</button>
      <button id="btnLoadMap">加载地图</button>
    </div>
  </section>
  <section>
    <h3>航点</h3>
    <button id="btnWpMode" class="wide">航点模式: 关</button>
    <div class="row2">
      <button id="btnWpStart">开始队列</button>
      <button id="btnWpClear">清空航点</button>
    </div>
  </section>
  <section>
    <h3>取物任务</h3>
    <button id="btnMissionMode" class="wide">取物点模式: 关</button>
    <div class="row2">
      <button id="btnMissionStart">开始任务</button>
      <button id="btnMissionCancel">取消任务</button>
    </div>
    <div id="missionInfo" class="kv"></div>
  </section>
</aside>

<div id="toast"></div>
<script>
const cv = document.getElementById('cv');
const ctx = cv.getContext('2d');
const hud = document.getElementById('hud');
const off = document.createElement('canvas');
const offCtx = off.getContext('2d');
let mapSeq = -1, state = null, linkErr = false;
let view = {scale: 80, cx: 0, cy: 0, follow: true};
let press = null, pan = null;
let wpMode = false, wps = [];
let missionMode = false, navClickMode = false;

const STATUS = {idle:'空闲', sending:'发送目标', active:'导航中', canceling:'取消中',
                succeeded:'已到达', canceled:'已取消', aborted:'失败', rejected:'被拒绝'};
const MODE = {mapping:'建图', navigation:'导航', switching:'切换中…', error:'异常'};
const MISSION = {idle:'空闲', armed:'已选点', to_target:'前往取物点',
                 grasping:'抓取中', to_home:'返回起点', done:'已取回',
                 error:'失败', canceled:'已取消'};

function resize(){ cv.width = window.innerWidth; cv.height = window.innerHeight; }
window.addEventListener('resize', resize); resize();

function s2w(sx, sy){
  return [view.cx + (sx - cv.width/2)/view.scale,
          view.cy - (sy - cv.height/2)/view.scale];
}
function w2s(x, y){
  return [cv.width/2 + (x - view.cx)*view.scale,
          cv.height/2 - (y - view.cy)*view.scale];
}

function rebuildMap(s){
  off.width = s.w; off.height = s.h;
  const img = offCtx.createImageData(s.w, s.h);
  let idx = 0;
  for (const [v, n] of s.rle){
    for (let i = 0; i < n && idx < s.w*s.h; i++, idx++){
      const c = v < 0 ? [58,62,70] : (v >= 50 ? [12,14,18] : [236,239,243]);
      const p = idx*4;   // OccupancyGrid 第 0 行是 y 最小，绘制时按正高度直接铺开
      img.data[p]=c[0]; img.data[p+1]=c[1]; img.data[p+2]=c[2]; img.data[p+3]=255;
    }
  }
  offCtx.putImageData(img, 0, 0);
  mapSeq = s.seq;
}

function syncMode(s){
  const mode = s ? s.mode : null;
  const switching = (mode === 'switching');
  const bMap = document.getElementById('btnMap');
  const bNav = document.getElementById('btnNav');
  bMap.classList.toggle('active', mode === 'mapping');
  bNav.classList.toggle('active', mode === 'navigation');
  bMap.disabled = switching;
  bNav.disabled = switching;
}

function draw(){
  const s = state;
  ctx.setTransform(1,0,0,1,0,0);
  ctx.fillStyle = '#0e1116';
  ctx.fillRect(0,0,cv.width,cv.height);
  syncMode(s);
  if (!s || !s.w){
    hud.innerHTML = '<div class="hud-head"><span class="hud-title">状态</span>'
                  + '<span class="badge" style="background:' + (s && s.mode === 'switching'
                    ? '#8a6d1f' : '#333') + '">'
                  + (s ? (MODE[s.mode] || s.mode) : '连接中') + '</span></div>'
                  + '<div class="kv"><span>等待地图…</span></div>';
    return;
  }
  if (s.seq !== mapSeq) rebuildMap(s);
  if (view.follow && s.pose){ view.cx = s.pose[0]; view.cy = s.pose[1]; }

  const mw = s.w*s.res, mh = s.h*s.res;
  ctx.translate(cv.width/2, cv.height/2);
  ctx.scale(view.scale, -view.scale);
  ctx.translate(-view.cx, -view.cy);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(off, s.ox, s.oy, mw, mh);

  if (s.plan && s.plan.length > 1){
    ctx.strokeStyle = '#38d67a';
    ctx.lineWidth = 1.8/view.scale;
    ctx.beginPath();
    ctx.moveTo(s.plan[0][0], s.plan[0][1]);
    for (let i=1;i<s.plan.length;i++) ctx.lineTo(s.plan[i][0], s.plan[i][1]);
    ctx.stroke();
  }

  ctx.fillStyle = '#41a7ff';
  for (const [x,y] of s.scan){
    ctx.beginPath();
    ctx.arc(x, y, 1.2/view.scale, 0, Math.PI*2);
    ctx.fill();
  }

  // 车体轮廓（红色，随位姿移动）
  if (s.outline){
    ctx.save();
    if (s.pose){
      ctx.translate(s.pose[0], s.pose[1]);
      ctx.rotate(s.pose[2]);
    }
    ctx.strokeStyle = '#ff3b3b';
    ctx.lineWidth = 1.5/view.scale;
    for (const contour of s.outline){
      if (!contour.length) continue;
      ctx.beginPath();
      ctx.moveTo(contour[0][0], contour[0][1]);
      for (let i=1;i<contour.length;i++) ctx.lineTo(contour[i][0], contour[i][1]);
      ctx.closePath();
      ctx.stroke();
    }
    ctx.restore();
  }

  // 导航目标
  if (s.goal){
    const [gx,gy,gyaw] = s.goal;
    ctx.save();
    ctx.translate(gx, gy); ctx.rotate(gyaw);
    ctx.strokeStyle = '#ffb454'; ctx.fillStyle = '#ffb454';
    ctx.lineWidth = 1.6/view.scale;
    ctx.beginPath(); ctx.moveTo(0,0); ctx.lineTo(0.30,0); ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(0.38,0); ctx.lineTo(0.24,-0.08); ctx.lineTo(0.24,0.08);
    ctx.closePath(); ctx.fill();
    ctx.restore();
  }

  // 取物点（品红）
  if (s.mission_target){
    const [gx,gy,gyaw] = s.mission_target;
    ctx.save();
    ctx.translate(gx, gy); ctx.rotate(gyaw);
    ctx.strokeStyle = '#ff59d6'; ctx.fillStyle = '#ff59d6';
    ctx.lineWidth = 1.6/view.scale;
    ctx.beginPath(); ctx.arc(0, 0, 0.10, 0, Math.PI*2); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(0,0); ctx.lineTo(0.26,0); ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(0.34,0); ctx.lineTo(0.22,-0.07); ctx.lineTo(0.22,0.07);
    ctx.closePath(); ctx.fill();
    ctx.restore();
  }

  // 车头箭头
  if (s.pose){
    const [x,y,yaw] = s.pose;
    ctx.save();
    ctx.translate(x, y); ctx.rotate(yaw);
    ctx.strokeStyle = '#ff4d4d'; ctx.fillStyle = '#ff4d4d';
    ctx.lineWidth = 1.6/view.scale;
    ctx.beginPath(); ctx.moveTo(0,0); ctx.lineTo(0.24,0); ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(0.32,0); ctx.lineTo(0.20,-0.07); ctx.lineTo(0.20,0.07);
    ctx.closePath(); ctx.fill();
    ctx.restore();
  }

  ctx.setTransform(1,0,0,1,0,0);

  // 本地航点（屏幕坐标，带编号）
  if (wps.length){
    ctx.fillStyle = '#ffe066';
    ctx.strokeStyle = '#ffe066';
    ctx.font = '12px monospace';
    for (let i=0;i<wps.length;i++){
      const [px,py] = w2s(wps[i][0], wps[i][1]);
      ctx.beginPath(); ctx.arc(px,py,4,0,Math.PI*2); ctx.fill();
      ctx.fillText(String(i+1), px+7, py-6);
    }
  }

  const st = STATUS[s.status] || s.status || '空闲';
  const stColor = (s.status === 'active' || s.status === 'sending') ? 'warn'
                : (s.status === 'succeeded' ? 'ok' : (s.status === 'aborted' ? 'bad' : ''));
  const MODE_BG = {mapping:'#1f5f8f', navigation:'#1f7a4d',
                   switching:'#8a6d1f', error:'#7a2f2f'};
  const kv = (k, v, cls) => '<div class="kv"><span>' + k + '</span><b class="'
                + (cls || '') + '">' + v + '</b></div>';
  let html = '<div class="hud-head"><span class="hud-title">状态</span>'
           + '<span class="badge" style="background:' + (MODE_BG[s.mode] || '#333')
           + '">' + (MODE[s.mode] || s.mode || '未知') + '</span></div>';
  html += kv('地图', s.w + '×' + s.h + ' · ' + mw.toFixed(1) + '×' + mh.toFixed(1) + 'm'
             + (s.has_map ? '' : ' · 未加载'), s.has_map ? '' : 'bad');
  html += kv('位姿', s.pose ? s.pose[0].toFixed(2) + ', ' + s.pose[1].toFixed(2) + ', '
             + (s.pose[2]*57.3).toFixed(0) + '°' : '无');
  html += kv('导航', st
             + (s.dist !== null && s.dist !== undefined ? ' · 剩余 ' + s.dist.toFixed(2) + 'm' : '')
             + (s.mode === 'navigation' && !s.nav_ready ? ' · 未就绪' : ''), stColor);
  if (s.queue && (s.queue.active || s.queue.n))
    html += kv('队列', (s.queue.i+1) + ' / ' + s.queue.n
               + (s.queue.active ? ' 执行中' : ''), s.queue.active ? 'warn' : '');
  if (s.battery) html += kv('电压', s.battery);
  if (s.mission) html += kv('任务', (MISSION[s.mission.state] || s.mission.state)
             + (s.mission.detail ? ' · ' + s.mission.detail : ''),
             s.mission.state === 'error' ? 'bad'
             : (s.mission.state === 'done' ? 'ok' : ''));
  html += kv('雷达', s.scan.length + ' 点 · ' + view.scale.toFixed(0) + 'px/m');
  if (s.mode === 'switching')
    html += '<div class="kv"><span class="warn">切换中：底盘已暂停</span></div>';
  hud.innerHTML = html;
}

async function post(path, data){
  try{
    const r = await fetch(path, {method:'POST', headers:{'Content-Type':'application/json'},
                                 body:JSON.stringify(data||{})});
    return await r.json();
  }catch(e){ return {ok:false, message:String(e)}; }
}
function setFlash(t, isErr){
  const el = document.getElementById('toast');
  if (!t){ el.className = ''; return; }
  el.textContent = t;
  el.className = 'show' + (isErr ? ' err' : '');
  clearTimeout(setFlash._t);
  setFlash._t = setTimeout(() => { el.className = ''; }, 3000);
}

function fitMap(){
  const s = state;
  if (!s || !s.w){ setFlash('还没有地图', true); return; }
  const narrow = window.innerWidth <= 900;
  const mw = s.w*s.res, mh = s.h*s.res;
  const availW = Math.max(320, cv.width - (narrow ? 40 : 490));
  const availH = Math.max(220, cv.height - (narrow ? 320 : 170));
  view.scale = Math.max(15, Math.min(400, Math.min(availW/mw, availH/mh)));
  view.cx = s.ox + mw/2;
  view.cy = s.oy + mh/2;
  view.follow = false;
  setFollowBtn();
  draw();
  setFlash('已适配地图 · ' + view.scale.toFixed(0) + 'px/m');
}

async function tick(){
  try{
    const r = await fetch('/state', {cache:'no-store'});
    state = await r.json();
    linkErr = false;
    draw();
  }catch(e){
    if (!linkErr){ linkErr = true; setFlash('连接失败: ' + e, true); }
  }
  setTimeout(tick, 300);
}

cv.addEventListener('contextmenu', e => e.preventDefault());
cv.addEventListener('mousedown', e => {
  if (e.button === 2){ pan = {sx:e.clientX, sy:e.clientY}; return; }
  if (e.button === 0){
    press = {sx:e.clientX, sy:e.clientY, shift:e.shiftKey, moved:false};
  }
});
cv.addEventListener('mousemove', e => {
  if (pan){
    view.cx -= (e.clientX - pan.sx)/view.scale;
    view.cy += (e.clientY - pan.sy)/view.scale;
    pan.sx = e.clientX; pan.sy = e.clientY;
    view.follow = false; setFollowBtn(); draw(); return;
  }
  if (press && Math.hypot(e.clientX-press.sx, e.clientY-press.sy) > 5)
    press.moved = true;
});
cv.addEventListener('mouseup', e => {
  if (e.button === 2){ pan = null; return; }
  if (e.button !== 0 || !press) return;
  const start = s2w(press.sx, press.sy);
  const end = s2w(e.clientX, e.clientY);
  let yaw = 0;
  if (press.moved) yaw = Math.atan2(end[1]-start[1], end[0]-start[0]);
  else if (state && state.pose) yaw = Math.atan2(start[1]-state.pose[1], start[0]-state.pose[0]);
  const payload = {x:start[0], y:start[1], yaw:yaw};
  if (press.shift){
    post('/initialpose', payload).then(r => setFlash(r.message, !r.ok));
  } else if (wpMode){
    wps.push([start[0], start[1], yaw]);
    setFlash('已添加航点 ' + wps.length);
  } else if (missionMode){
    post('/mission_target', payload).then(r => setFlash(r.message, !r.ok));
  } else if (navClickMode){
    post('/goal', payload).then(r => setFlash(r.message, !r.ok));
  } else {
    setFlash('点击未生效：请先开启「取物点模式」或「点选导航」', true);
  }
  press = null;
});
cv.addEventListener('wheel', e => {
  e.preventDefault();
  const before = s2w(e.clientX, e.clientY);
  const factor = e.deltaY < 0 ? 1.15 : 1/1.15;
  view.scale = Math.max(15, Math.min(500, view.scale*factor));
  if (!view.follow){
    view.cx = before[0] - (e.clientX - cv.width/2)/view.scale;
    view.cy = before[1] + (e.clientY - cv.height/2)/view.scale;
  }
  draw();
}, {passive:false});

function setFollowBtn(){
  const b = document.getElementById('btnFollow');
  b.textContent = '跟随: ' + (view.follow ? '开' : '关');
  b.className = view.follow ? 'on' : '';
}
document.getElementById('btnFollow').onclick = () => { view.follow = !view.follow; setFollowBtn(); draw(); };
document.getElementById('btnFit').onclick = fitMap;
cv.addEventListener('dblclick', fitMap);
document.getElementById('btnCancel').onclick = async () => {
  wps = [];
  await post('/clear_waypoints', {});
  const r = await post('/cancel', {});
  const m = await post('/mission_cancel', {});
  setFlash('已取消：' + (r.ok ? '导航' : '') + (m.ok ? ' 任务' : ''), false);
  draw();
};
document.getElementById('btnReloc').onclick = async () => {
  const r = await post('/relocalize', {}); setFlash(r.message, !r.ok);
};
document.getElementById('btnNav').onclick = async () => {
  if (!confirm('切换到导航？会先保存地图并记录当前位置。')) return;
  const r = await post('/set_mode', {mode:'navigation'}); setFlash(r.message, !r.ok);
};
document.getElementById('btnMap').onclick = async () => {
  if (!confirm('返回建图？导航会暂停，并开始新一轮建图。')) return;
  const r = await post('/set_mode', {mode:'mapping'}); setFlash(r.message, !r.ok);
};
document.getElementById('btnSaveMap').onclick = async () => {
  const name = document.getElementById('mapName').value.trim();
  const r = await post('/save_map', {name:name}); setFlash(r.message, !r.ok);
};
document.getElementById('btnLoadMap').onclick = async () => {
  const name = document.getElementById('mapName').value.trim();
  const r = await post('/load_map', {name:name}); setFlash(r.message, !r.ok);
};
function saveModes(){
  try {
    localStorage.setItem('missionMode', missionMode ? '1' : '0');
    localStorage.setItem('navClickMode', navClickMode ? '1' : '0');
    localStorage.setItem('wpMode', wpMode ? '1' : '0');
  } catch(e){}
}
function loadModes(){
  try {
    missionMode = localStorage.getItem('missionMode') === '1';
    navClickMode = localStorage.getItem('navClickMode') === '1';
    wpMode = localStorage.getItem('wpMode') === '1';
  } catch(e){}
  syncModeButtons();
}
function syncModeButtons(){
  const m = document.getElementById('btnMissionMode');
  m.textContent = '取物点模式: ' + (missionMode ? '开' : '关');
  m.className = missionMode ? 'on' : '';
  m.classList.toggle('wide', true);
  const n = document.getElementById('btnNavClick');
  n.textContent = '点选导航: ' + (navClickMode ? '开' : '关');
  n.className = navClickMode ? 'on' : '';
  const w = document.getElementById('btnWpMode');
  w.textContent = '航点模式: ' + (wpMode ? '开' : '关');
  w.className = wpMode ? 'on' : '';
}
document.getElementById('btnNavClick').onclick = () => {
  navClickMode = !navClickMode;
  if (navClickMode && missionMode) { missionMode = false; }
  saveModes(); syncModeButtons();
  if (navClickMode) setFlash('点图将直接下发导航目标');
};
document.getElementById('btnWpMode').onclick = () => {
  wpMode = !wpMode;
  saveModes(); syncModeButtons();
};
document.getElementById('btnWpStart').onclick = async () => {
  if (!wps.length) { setFlash('没有航点', true); return; }
  const r = await post('/waypoints', {points:wps, start:true}); setFlash(r.message, !r.ok);
};
document.getElementById('btnWpClear').onclick = async () => {
  wps = []; await post('/clear_waypoints', {}); setFlash('已清空航点'); draw();
};
document.getElementById('btnMissionMode').onclick = () => {
  missionMode = !missionMode;
  if (missionMode && navClickMode) { navClickMode = false; }
  saveModes(); syncModeButtons();
  if (missionMode) setFlash('在图上点一下选取物点');
};
document.getElementById('btnMissionStart').onclick = async () => {
  const r = await post('/mission_start', {});
  setFlash(r.message, !r.ok);
};
document.getElementById('btnMissionCancel').onclick = async () => {
  const r = await post('/mission_cancel', {});
  setFlash(r.message, !r.ok);
};
setFollowBtn();
loadModes();
tick();
</script>
</body></html>
"""


class WebView(Node):

    def __init__(self):
        super().__init__('map_web_view')

        self.declare_parameter('port', 8080)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('scan_frame', 'laser')
        self.declare_parameter('max_scan_points', 300)
        self.declare_parameter('map_dir', os.path.expanduser('~/maps'))

        self.port = int(self.get_parameter('port').value)
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.scan_frame = self.get_parameter('scan_frame').value
        self.max_pts = int(self.get_parameter('max_scan_points').value)
        self.map_dir = str(self.get_parameter('map_dir').value)

        self.lock = threading.Lock()
        self.map_msg = None
        self.scan_msg = None
        self.seq = 0
        # 最近一次浏览器访问 /state 的时刻；没人看网页时就不做 TF 查询和
        # 雷达点转换，否则这个节点空转也要吃掉半个核
        self.last_client = 0.0
        self.rle = []          # 收到新地图时计算一次，供 /state 复用
        self.scan_pts = []     # 每帧雷达更新一次，供 /state 复用
        self.pose_cache = None # 10Hz 定时刷新
        self.plan = []
        self.goal_xy = None
        self.goal_handle = None
        self.goal_gen = 0
        self.nav_status = 'idle'
        self.distance = None
        self.battery = None
        self.waypoints = []
        self.wp_index = 0
        self.queue_active = False
        self.mode = 'unknown'
        self.mission = None
        self.mission_target = None

        self.outline = self._load_outline()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 与 slam_toolbox / map_server 的 TRANSIENT_LOCAL 匹配，能立刻拿到已发布的地图
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, 'map', self.on_map, map_qos)
        self.create_subscription(LaserScan, 'scan', self.on_scan, 10)
        self.create_subscription(Path, 'plan', self.on_plan, 10)
        self.create_subscription(Float32, 'battery_voltage', self.on_battery, 10)
        self.create_subscription(String, '/mode/state', self.on_mode, 10)
        self.create_subscription(MissionState, '/mission/state',
                                 self.on_mission, 10)
        self.create_timer(0.2, self.refresh_pose)

        self.goal_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.init_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.relocalize_cli = self.create_client(Trigger, '/relocalize')
        self.save_cli = self.create_client(SaveMap, '/slam_toolbox/save_map')
        self.load_cli = self.create_client(LoadMap, '/map_server/load_map')
        self.mode_map_cli = self.create_client(Trigger, '/mode/start_mapping')
        self.mode_nav_cli = self.create_client(Trigger, '/mode/start_navigation')
        self.mission_start_cli = self.create_client(Trigger, '/mission/start')
        self.mission_cancel_cli = self.create_client(Trigger, '/mission/cancel')
        self.mission_pub = self.create_publisher(
            PoseStamped, '/mission/target',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.server = ThreadingHTTPServer(('0.0.0.0', self.port), Handler)
        Handler.node = self
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.get_logger().info('网页面板已启动：http://<板卡IP>:%d/' % self.port)

    def _load_outline(self):
        try:
            path = os.path.join(get_package_share_directory('mecanum_nav'),
                                'config', 'car_outline.json')
            with open(path, 'r') as f:
                return json.load(f)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('读取车体轮廓失败: %s' % exc)
            return []

    # ---------- 订阅 ----------
    def on_map(self, msg):
        rle = []
        prev = None
        cnt = 0
        for v in msg.data:
            val = -1 if v < 0 else (100 if v >= 50 else 0)
            if val == prev:
                cnt += 1
            else:
                if prev is not None:
                    rle.append([prev, cnt])
                prev, cnt = val, 1
        rle.append([prev, cnt])
        with self.lock:
            self.map_msg = msg
            self.rle = rle
            self.seq += 1

    def on_scan(self, msg):
        if time.monotonic() - self.last_client > 2.0:
            return
        pts = self._scan_points(msg)
        with self.lock:
            self.scan_msg = msg
            if pts:
                self.scan_pts = pts

    def on_plan(self, msg):
        pts = [[round(p.pose.position.x, 2), round(p.pose.position.y, 2)]
               for p in msg.poses]
        with self.lock:
            self.plan = pts

    def on_battery(self, msg):
        self.battery = '%.2f V' % float(msg.data)

    def on_mode(self, msg):
        self.mode = str(msg.data)

    def on_mission(self, msg):
        # 目标以网页自己选的点为准：MissionState 里没选点时会带 (0,0) 默认值
        self.mission = {'state': str(msg.state), 'detail': str(msg.detail)}

    def set_mission_target(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.z = math.sin(float(yaw) / 2.0)
        msg.pose.orientation.w = math.cos(float(yaw) / 2.0)
        self.mission_pub.publish(msg)
        self.mission_target = [round(float(x), 2), round(float(y), 2),
                               round(float(yaw), 3)]
        return True, '取物点已选 (%.2f, %.2f)' % (x, y)

    def start_mission(self):
        if not self.mission_start_cli.service_is_ready():
            return False, '任务节点不可用（是否启动了 jetarm_mission？）'
        r, err = self.call_service(self.mission_start_cli,
                                   Trigger.Request(), timeout=5.0)
        if r is None:
            return False, err
        return bool(r.success), str(r.message)

    def cancel_mission(self):
        if not self.mission_cancel_cli.service_is_ready():
            return False, '任务节点不可用'
        r, err = self.call_service(self.mission_cancel_cli,
                                   Trigger.Request(), timeout=5.0)
        if r is None:
            return False, err
        if bool(r.success):
            self.mission_target = None
        return bool(r.success), str(r.message)

    # ---------- 位姿与点云 ----------
    def refresh_pose(self):
        if time.monotonic() - self.last_client > 2.0:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time(),
                rclpy.time.Duration(seconds=0.0))
            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.pose_cache = [round(t.x, 2), round(t.y, 2), round(yaw, 3)]
        except Exception:  # noqa: BLE001
            pass

    def _scan_points(self, scan):
        try:
            fb = self.tf_buffer.lookup_transform(
                self.map_frame, self.scan_frame, rclpy.time.Time(),
                rclpy.time.Duration(seconds=0.05))
        except Exception:  # noqa: BLE001
            return []
        t = fb.transform.translation
        q = fb.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        ranges = scan.ranges
        n = len(ranges)
        stride = max(1, int(math.ceil(n / float(max(1, self.max_pts)))))
        out = []
        for i in range(0, n, stride):
            r = ranges[i]
            if not (scan.range_min <= r <= scan.range_max):
                continue
            a = scan.angle_min + i * scan.angle_increment + yaw
            out.append([round(t.x + r * math.cos(a), 2),
                        round(t.y + r * math.sin(a), 2)])
        return out

    def state(self):
        self.last_client = time.monotonic()
        with self.lock:
            m = self.map_msg
            seq = self.seq
            plan = self.plan
            rle = self.rle
            scan_pts = self.scan_pts
            pose = self.pose_cache
        if m is None:
            return {'w': 0, 'has_map': False, 'mode': self.mode,
                    'nav_ready': self.goal_client.server_is_ready()
                                 and self.mode == 'navigation',
                    'outline': self.outline,
                    'battery': self.battery, 'scan': [],
                    'status': self.nav_status, 'dist': self.distance,
                    'queue': {'n': len(self.waypoints),
                              'i': self.wp_index,
                              'active': self.queue_active},
                    'goal': self.goal_xy,
                    'mission': self.mission,
                    'mission_target': self.mission_target}
        return {
            'w': m.info.width, 'h': m.info.height,
            'res': m.info.resolution,
            'ox': m.info.origin.position.x,
            'oy': m.info.origin.position.y,
            'seq': seq, 'rle': rle, 'has_map': True, 'mode': self.mode,
            'nav_ready': self.goal_client.server_is_ready()
                         and self.mode == 'navigation',
            'outline': self.outline,
            'pose': pose,
            'scan': scan_pts,
            'plan': plan,
            'goal': self.goal_xy,
            'status': self.nav_status,
            'dist': self.distance,
            'battery': self.battery,
            'queue': {'n': len(self.waypoints),
                      'i': self.wp_index,
                      'active': self.queue_active},
            'mission': self.mission,
            'mission_target': self.mission_target,
        }

    # ---------- 导航 ----------
    def publish_initial_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        msg.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
        cov = [0.0] * 36
        cov[0] = 0.25
        cov[7] = 0.25
        cov[35] = 0.068
        msg.pose.covariance = cov
        self.init_pub.publish(msg)

    def send_goal(self, x, y, yaw):
        with self.lock:
            has_map = self.map_msg is not None
        if not has_map:
            return False, '尚未加载地图'
        if self.mode != 'navigation':
            return False, '当前是建图模式，请先点击“进入导航”'
        if not self.goal_client.server_is_ready():
            return False, '导航 action 未就绪（是否启动了 nav2？）'
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        goal.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)

        self.goal_gen += 1
        gen = self.goal_gen
        self.goal_xy = [round(float(x), 2), round(float(y), 2),
                        round(float(yaw), 3)]
        self.nav_status = 'sending'
        self.distance = None
        fut = self.goal_client.send_goal_async(
            goal, feedback_callback=lambda fb: self.on_feedback(fb, gen))
        fut.add_done_callback(lambda f: self.on_goal_response(f, gen))
        return True, '目标已发送'

    def on_feedback(self, msg, gen):
        if gen != self.goal_gen:
            return
        try:
            d = msg.feedback.distance_remaining
            if d is not None and d >= 0:
                self.distance = float(d)
        except Exception:  # noqa: BLE001
            pass

    def on_goal_response(self, fut, gen):
        if gen != self.goal_gen:
            return
        handle = fut.result()
        if not handle.accepted:
            self.nav_status = 'rejected'
            self.queue_active = False
            return
        self.goal_handle = handle
        self.nav_status = 'active'
        handle.get_result_async().add_done_callback(
            lambda f: self.on_result(f, gen))

    def on_result(self, fut, gen):
        if gen != self.goal_gen:
            return
        try:
            st = fut.result().status
        except Exception:  # noqa: BLE001
            st = 6
        self.nav_status = ('succeeded' if st == 4 else
                           'canceled' if st == 5 else 'aborted')
        self.goal_handle = None
        if self.queue_active:
            if st == 4 and self.wp_index + 1 < len(self.waypoints):
                self.wp_index += 1
                threading.Timer(0.5, self._send_next_waypoint).start()
            else:
                self.queue_active = False

    def cancel_goal(self):
        self.waypoints = []
        self.queue_active = False
        self.goal_xy = None
        if self.goal_handle is None:
            return False, '当前没有进行中的目标'
        self.goal_handle.cancel_goal_async()
        self.goal_handle = None
        self.nav_status = 'canceling'
        return True, '已取消'

    def set_waypoints(self, points, start):
        self.waypoints = [(float(p['x']), float(p['y']), float(p['yaw']))
                          for p in points]
        self.wp_index = 0
        self.queue_active = bool(start and self.waypoints)
        if self.queue_active:
            self._send_next_waypoint()
            return True, '队列已开始（%d 个航点）' % len(self.waypoints)
        return True, '已保存 %d 个航点' % len(self.waypoints)

    def _send_next_waypoint(self):
        if not self.queue_active or self.wp_index >= len(self.waypoints):
            self.queue_active = False
            return
        x, y, yaw = self.waypoints[self.wp_index]
        ok, msg = self.send_goal(x, y, yaw)
        if not ok:
            self.queue_active = False
            self.get_logger().warn('航点发送失败: %s' % msg)

    def clear_waypoints(self):
        self.waypoints = []
        self.wp_index = 0
        self.queue_active = False

    # ---------- 服务 ----------
    def call_service(self, client, request, timeout=3.0):
        if not client.service_is_ready():
            return None, '服务未就绪'
        ev = threading.Event()
        box = {}

        def done(fut):
            try:
                box['r'] = fut.result()
            except Exception as exc:  # noqa: BLE001
                box['e'] = exc
            ev.set()

        client.call_async(request).add_done_callback(done)
        if not ev.wait(timeout):
            return None, '服务超时'
        if 'e' in box:
            return None, str(box['e'])
        return box['r'], ''

    def relocalize(self):
        r, err = self.call_service(self.relocalize_cli, Trigger.Request())
        if r is None:
            return False, err
        return bool(r.success), str(r.message)

    def save_map(self, name):
        identity = name or ''
        if not identity:
            return False, '请填写地图名'
        prefix = os.path.join(self.map_dir, identity)
        saver = '/opt/ros/humble/lib/nav2_map_server/map_saver_cli'
        if not os.path.exists(saver):
            saver = 'map_saver_cli'
        cmd = [saver, '-f', prefix, '-t', '/map',
               '--ros-args', '-p', 'save_map_timeout:=30.0']
        try:
            proc = subprocess.run(cmd, timeout=60.0,
                                  capture_output=True, text=True)
        except Exception as exc:  # noqa: BLE001
            return False, '保存失败: %s' % exc
        if os.path.exists(prefix + '.yaml'):
            return True, '已保存到 %s.yaml' % prefix
        tail = (proc.stderr or proc.stdout or '').strip().splitlines()
        return False, '保存失败（rc=%d %s）' % (
            proc.returncode, tail[-1] if tail else '')

    def load_map(self, name):
        if not name:
            return False, '请填写地图名'
        if not self.load_cli.service_is_ready():
            return False, '加载地图服务不可用（需在导航模式）'
        path = os.path.join(self.map_dir, name + '.yaml')
        if not os.path.exists(path):
            return False, '地图不存在: %s' % path
        req = LoadMap.Request()
        req.map_url = path
        r, err = self.call_service(self.load_cli, req, timeout=5.0)
        if r is None:
            return False, err
        code = int(getattr(r, 'result', 0))
        ok = (code == 0)
        return ok, ('已加载 %s' % path) if ok else \
            '加载失败（result=%d）' % code

    def set_mode(self, mode):
        if self.mode == 'switching':
            return False, '正在切换中，请稍候'
        if mode == 'navigation':
            cli = self.mode_nav_cli
            text = '正在切换到导航（会保存地图并记录当前位置）'
        elif mode == 'mapping':
            cli = self.mode_map_cli
            text = '正在切换到建图'
        else:
            return False, '未知模式: %s' % mode
        if not cli.service_is_ready():
            return False, '模式管理服务不可用（是否用 robot.launch.py 启动？）'
        r, err = self.call_service(cli, Trigger.Request(), timeout=8.0)
        if r is None:
            return False, err
        return bool(r.success), str(r.message) if r.message else text


class Handler(BaseHTTPRequestHandler):

    node = None

    def _json(self, obj, code=200):
        body = json.dumps(obj, separators=(',', ':')).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            body = PAGE.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path.startswith('/state'):
            self._json(self.node.state())
        else:
            self.send_error(404)

    def do_POST(self):
        try:
            n = int(self.headers.get('Content-Length', 0))
            data = json.loads(self.rfile.read(n) or b'{}')
        except Exception:  # noqa: BLE001
            self._json({'ok': False, 'message': '请求格式错误'}, 400)
            return

        node = self.node
        try:
            if self.path == '/goal':
                ok, msg = node.send_goal(data.get('x', 0.0),
                                         data.get('y', 0.0),
                                         data.get('yaw', 0.0))
            elif self.path == '/initialpose':
                node.publish_initial_pose(data.get('x', 0.0),
                                          data.get('y', 0.0),
                                          data.get('yaw', 0.0))
                ok, msg = True, '初始位姿已发布'
            elif self.path == '/cancel':
                ok, msg = node.cancel_goal()
            elif self.path == '/relocalize':
                ok, msg = node.relocalize()
            elif self.path == '/set_mode':
                ok, msg = node.set_mode(str(data.get('mode', '')))
            elif self.path == '/battery':
                ok, msg = (bool(node.battery), node.battery or '暂无电压')
            elif self.path == '/save_map':
                ok, msg = node.save_map(str(data.get('name', '')))
            elif self.path == '/load_map':
                ok, msg = node.load_map(str(data.get('name', '')))
            elif self.path == '/waypoints':
                ok, msg = node.set_waypoints(data.get('points', []),
                                             bool(data.get('start', False)))
            elif self.path == '/clear_waypoints':
                node.clear_waypoints()
                ok, msg = True, '已清空航点'
            elif self.path == '/mission_target':
                ok, msg = node.set_mission_target(data.get('x', 0.0),
                                                  data.get('y', 0.0),
                                                  data.get('yaw', 0.0))
            elif self.path == '/mission_start':
                ok, msg = node.start_mission()
            elif self.path == '/mission_cancel':
                ok, msg = node.cancel_mission()
            else:
                self._json({'ok': False, 'message': '未知接口'}, 404)
                return
        except Exception as exc:  # noqa: BLE001
            ok, msg = False, str(exc)
        self._json({'ok': ok, 'message': msg, 'battery': node.battery})

    def log_message(self, *args):
        pass


def main():
    rclpy.init()
    node = WebView()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
