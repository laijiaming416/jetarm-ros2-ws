#!/usr/bin/env python3
"""建图 / 导航模式管理。

建图模式：
    启动 slam_toolbox 进程（slam_toolbox 没有生命周期，只能进程启停）

导航模式：
    停止 slam 进程，激活 nav2 的 localization / navigation 生命周期管理器

切换时：
    1. 底盘零速
    2. 建图→导航：保存地图、记录 map->base_link 位姿、停 slam、激活 nav2、发布 /initialpose
    3. 导航→建图：nav2 用 PAUSE 停活（保留状态，之后可用 RESUME 恢复），再启动 slam

注意：
    返回建图会开始新一轮建图；旧地图已在切换时保存到 ~/maps/<map_name>.yaml，
    导航模式下次会重新加载该文件。

服务：
    /mode/start_mapping   std_srvs/Trigger
    /mode/start_navigation std_srvs/Trigger
状态：
    /mode/state           std_msgs/String（mapping / navigation / switching / error）
"""
import math
import os
import shutil
import signal
import subprocess
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.srv import LoadMap, ManageLifecycleNodes
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from slam_toolbox.srv import SaveMap
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# ManageLifecycleNodes 命令码
MANAGE_STARTUP = 0
MANAGE_PAUSE = 1
MANAGE_RESUME = 2


class ModeManager(Node):

    def __init__(self):
        super().__init__('mode_manager')

        self.declare_parameter('map_name', 'map')
        self.declare_parameter('map_dir', os.path.expanduser('~/maps'))
        self.declare_parameter('start_mode', 'mapping')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('slam_params_file',
                               os.path.join(get_package_share_directory('mecanum_nav'),
                                            'config', 'slam_params.yaml'))
        # 取物相关节点（机械臂/相机/抓取/任务/语音）只在导航模式需要
        self.declare_parameter('extras_enable', True)
        self.declare_parameter('extras_package', 'jetarm_bringup')
        self.declare_parameter('extras_launch', 'pickup.launch.py')

        self.map_name = str(self.get_parameter('map_name').value)
        self.map_dir = str(self.get_parameter('map_dir').value)
        self.start_mode = str(self.get_parameter('start_mode').value)
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.slam_params = str(self.get_parameter('slam_params_file').value)
        self.map_path = os.path.join(self.map_dir, self.map_name + '.yaml')
        self.pose_file = os.path.expanduser(
            '~/.ros/last_pose_%s.yaml' % self.map_name)

        self.mode = 'unknown'
        self.switching = False
        self.switch_lock = threading.Lock()
        self.slam_proc = None
        self.extras_proc = None
        self.extras_log = None
        self.nav_started = False

        self.state_pub = self.create_publisher(String, '/mode/state', 10)
        self.init_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.stop_pub = self.create_publisher(Twist, '/cmd_vel_final', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.loc_cli = self.create_client(
            ManageLifecycleNodes, '/lifecycle_manager_localization/manage_nodes')
        self.nav_cli = self.create_client(
            ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes')
        self.save_cli = self.create_client(SaveMap, '/slam_toolbox/save_map')
        self.load_map_cli = self.create_client(LoadMap,
                                               '/map_server/load_map')

        self.create_service(Trigger, '/mode/start_mapping', self.on_start_mapping)
        self.create_service(Trigger, '/mode/start_navigation',
                            self.on_start_navigation)
        self.create_timer(1.0, self.publish_state)

        self.get_logger().info('模式管理启动：start_mode=%s，地图=%s'
                               % (self.start_mode, self.map_path))
        threading.Timer(2.0, self.boot).start()

    # ---------- 对外服务 ----------
    def on_start_mapping(self, request, response):
        if self.switching:
            response.success = False
            response.message = '正在切换中'
            return response
        if self.mode == 'mapping':
            response.success = True
            response.message = '已是建图模式'
            return response
        self.switching = True
        threading.Thread(target=self.switch_to_mapping, daemon=True).start()
        response.success = True
        response.message = '正在切换到建图'
        return response

    def on_start_navigation(self, request, response):
        if self.switching:
            response.success = False
            response.message = '正在切换中'
            return response
        if self.mode == 'navigation':
            response.success = True
            response.message = '已是导航模式'
            return response
        self.switching = True
        threading.Thread(target=self.switch_to_navigation, daemon=True).start()
        response.success = True
        response.message = '正在切换到导航'
        return response

    def publish_state(self):
        msg = String()
        msg.data = self.mode
        self.state_pub.publish(msg)

    def set_mode(self, mode):
        self.mode = mode
        self.publish_state()
        self.get_logger().info('当前模式: %s' % mode)

    # ---------- 启动 ----------
    def boot(self):
        if self.start_mode == 'navigation':
            self.switching = True
            self.switch_to_navigation(from_boot=True)
        else:
            self.switching = True
            self.switch_to_mapping(from_boot=True)

    # ---------- 取物节点（按需启停） ----------
    def start_extras(self):
        if not bool(self.get_parameter('extras_enable').value):
            return
        if self.extras_proc is not None and self.extras_proc.poll() is None:
            return
        ros2 = shutil.which('ros2') or '/opt/ros/humble/bin/ros2'
        cmd = [ros2, 'launch',
               str(self.get_parameter('extras_package').value),
               str(self.get_parameter('extras_launch').value)]
        self.get_logger().info('启动取物节点: %s' % ' '.join(cmd))
        try:
            self.extras_log = open('/tmp/pickup.log', 'a', buffering=1)
            self.extras_proc = subprocess.Popen(
                cmd, start_new_session=True,
                stdout=self.extras_log, stderr=subprocess.STDOUT,
                preexec_fn=lambda: os.nice(10))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error('启动取物节点失败: %s' % exc)

    def stop_extras(self):
        proc = self.extras_proc
        self.extras_proc = None
        if self.extras_log is not None:
            try:
                self.extras_log.close()
            except Exception:  # noqa: BLE001
                pass
            self.extras_log = None
        if proc is None or proc.poll() is not None:
            return
        self.get_logger().info('停止取物节点')
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except Exception:  # noqa: BLE001
            pass
        for delay in (3.0, 4.0):
            time.sleep(delay)
            if proc.poll() is not None:
                return
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:  # noqa: BLE001
            pass

    # ---------- 建图 ----------
    def switch_to_mapping(self, from_boot=False):
        with self.switch_lock:
            try:
                self.set_mode('switching')
                self.stop_robot()
                self.stop_extras()
                if not from_boot:
                    # 先暂停 nav2（保留生命周期状态，之后可 RESUME）
                    if self.nav_started:
                        self.pause_nav()
                self.start_slam()
                time.sleep(3.0)
                self.set_mode('mapping')
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error('切换到建图失败: %s' % exc)
                self.set_mode('error')
            finally:
                self.switching = False

    def start_slam(self):
        if self.slam_proc is not None and self.slam_proc.poll() is None:
            return
        ros2 = shutil.which('ros2') or '/opt/ros/humble/bin/ros2'
        cmd = [ros2, 'run', 'slam_toolbox', 'async_slam_toolbox_node',
               '--ros-args', '--params-file', self.slam_params]
        self.get_logger().info('启动 slam: %s' % ' '.join(cmd))
        self.slam_proc = subprocess.Popen(
            cmd, start_new_session=True,
            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)

    def stop_slam(self):
        proc = self.slam_proc
        if proc is None or proc.poll() is not None:
            self.slam_proc = None
            return
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except Exception:  # noqa: BLE001
            pass
        for delay in (1.0, 2.0, 3.0):
            time.sleep(delay)
            if proc.poll() is not None:
                self.slam_proc = None
                return
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:  # noqa: BLE001
            pass
        self.slam_proc = None

    # ---------- 导航 ----------
    def switch_to_navigation(self, from_boot=False):
        with self.switch_lock:
            recover = (self.mode == 'error')
            try:
                self.set_mode('switching')
                self.stop_robot()
                if recover and self.nav_started:
                    self.pause_nav()
                if not from_boot:
                    # 存图失败必须中止切换：否则 stop_slam() 会在存图未完成时
                    # 杀掉 slam，地图丢失后 map_server 加载不到文件，整栈进 error
                    if not self.save_map():
                        self.get_logger().error(
                            '保存地图失败，取消切换到导航，保持建图模式')
                        self.set_mode('mapping')
                        return
                pose = self.capture_pose()
                confident = pose is not None
                if pose is None:
                    pose = self.load_pose_file()
                    confident = pose is not None
                if pose is None:
                    pose = (0.0, 0.0, 0.0)
                self.stop_slam()
                time.sleep(2.0)
                self.startup_nav(pose, confident)
                self.save_pose_file(pose)
                self.set_mode('navigation')
                self.start_extras()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error('切换到导航失败: %s' % exc)
                self.set_mode('error')
            finally:
                self.switching = False

    def startup_nav(self, pose, confident=True):
        resumed = self.nav_started
        if resumed:
            self.get_logger().info('恢复 nav2 localization...')
            if not self.call_manage(self.loc_cli, MANAGE_RESUME):
                raise RuntimeError('localization RESUME 失败')
            # 建图期间地图可能已更新，重新加载地图文件
            self.load_map_file()
        else:
            self.get_logger().info('激活 nav2 localization...')
            if not self.call_manage(self.loc_cli, MANAGE_STARTUP):
                raise RuntimeError('localization STARTUP 失败')
            self.nav_started = True
        # 先给 AMCL 初始位姿，否则 global_costmap 收不到 map->base_link 会卡死
        self.publish_initial_pose(pose, confident)
        self.wait_map_tf()
        if resumed:
            self.get_logger().info('恢复 nav2 navigation...')
            if not self.call_manage(self.nav_cli, MANAGE_RESUME):
                raise RuntimeError('navigation RESUME 失败')
        else:
            self.get_logger().info('激活 nav2 navigation...')
            if not self.call_manage(self.nav_cli, MANAGE_STARTUP):
                raise RuntimeError('navigation STARTUP 失败')
        time.sleep(2.0)

    def wait_map_tf(self, timeout=15.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.capture_pose() is not None:
                return True
            time.sleep(0.3)
        self.get_logger().warn('等待 map->base_link 超时，继续激活 nav2')
        return False

    def pause_nav(self):
        self.get_logger().info('暂停 nav2 navigation...')
        self.call_manage(self.nav_cli, MANAGE_PAUSE)
        self.get_logger().info('暂停 nav2 localization...')
        self.call_manage(self.loc_cli, MANAGE_PAUSE)
        time.sleep(2.0)

    def call_manage(self, client, command, timeout=45.0):
        if not client.wait_for_service(timeout_sec=20.0):
            return False
        req = ManageLifecycleNodes.Request()
        req.command = command
        ev = threading.Event()
        box = {}

        def done(fut):
            try:
                box['r'] = fut.result()
            except Exception as exc:  # noqa: BLE001
                box['e'] = exc
            ev.set()

        client.call_async(req).add_done_callback(done)
        if not ev.wait(timeout):
            return False
        if 'e' in box:
            return False
        return bool(box['r'].success)

    # ---------- 地图 / 位姿 ----------
    def save_map(self):
        """用 nav2_map_server 的 map_saver_cli 写 pgm/yaml。

        这块板子的 slam_toolbox save_map 服务固定返回 255（posegraph 的
        serialize_map 正常），所以不再走那个服务，直接调独立存图工具。
        """
        if self.slam_proc is None or self.slam_proc.poll() is not None:
            self.get_logger().warn('slam 未运行，跳过保存地图')
            return False
        prefix = os.path.join(self.map_dir, self.map_name)
        saver = '/opt/ros/humble/lib/nav2_map_server/map_saver_cli'
        if not os.path.exists(saver):
            saver = 'map_saver_cli'
        # 默认 save_map_timeout 只有 2 秒；localhost-only 下 DDS 发现更慢，
        # 实测需要 5~7 秒，放宽到 30 秒。
        cmd = [saver, '-f', prefix, '-t', '/map',
               '--ros-args', '-p', 'save_map_timeout:=30.0']
        for attempt in range(1, 4):
            try:
                proc = subprocess.run(cmd, timeout=60.0,
                                      capture_output=True, text=True)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error('存图命令异常（第 %d 次）：%s'
                                        % (attempt, exc))
                time.sleep(2.0)
                continue
            if proc.returncode == 0 and os.path.exists(self.map_path):
                self.get_logger().info('地图已保存: %s' % self.map_path)
                return True
            tail = (proc.stderr or proc.stdout or '').strip().splitlines()
            self.get_logger().error('存图失败（第 %d 次，rc=%d）：%s'
                                    % (attempt, proc.returncode,
                                       tail[-1] if tail else ''))
            time.sleep(2.0)
        self.get_logger().error('地图连续 3 次保存失败')
        return False

    def _call_client(self, client, request, timeout=8.0):
        if not client.wait_for_service(timeout_sec=3.0):
            return None
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
            return None
        return box.get('r')

    def load_map_file(self):
        """让已激活的 map_server 重新读取最新保存的地图文件。"""
        if not os.path.exists(self.map_path):
            self.get_logger().warn('地图文件不存在: %s' % self.map_path)
            return False
        req = LoadMap.Request()
        req.map_url = self.map_path
        r = self._call_client(self.load_map_cli, req, timeout=10.0)
        code = 255 if r is None else int(getattr(r, 'result', 255))
        if code == 0:
            self.get_logger().info('导航已加载地图: %s' % self.map_path)
            return True
        self.get_logger().warn('加载地图失败（result=%d）' % code)
        return False

    def capture_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time(),
                timeout=Duration(seconds=0.5))
            t = tf.transform.translation
            yaw = yaw_from_quat(tf.transform.rotation)
            return (t.x, t.y, yaw)
        except Exception:  # noqa: BLE001
            return None

    def load_pose_file(self):
        try:
            with open(self.pose_file, 'r') as f:
                data = yaml.safe_load(f) or {}
            return (float(data['x']), float(data['y']), float(data['yaw']))
        except Exception:  # noqa: BLE001
            return None

    def save_pose_file(self, pose):
        try:
            os.makedirs(os.path.dirname(self.pose_file), exist_ok=True)
            with open(self.pose_file, 'w') as f:
                yaml.safe_dump({'x': round(pose[0], 4),
                                'y': round(pose[1], 4),
                                'yaw': round(pose[2], 4)}, f)
        except Exception:  # noqa: BLE001
            pass

    def publish_initial_pose(self, pose, confident=True):
        for _ in range(3):
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = self.map_frame
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.pose.position.x = float(pose[0])
            msg.pose.pose.position.y = float(pose[1])
            msg.pose.pose.orientation.z = math.sin(float(pose[2]) / 2.0)
            msg.pose.pose.orientation.w = math.cos(float(pose[2]) / 2.0)
            cov = [0.0] * 36
            cov[0] = 0.25 if confident else 4.0
            cov[7] = 0.25 if confident else 4.0
            cov[35] = 0.068 if confident else 1.0
            msg.pose.covariance = cov
            self.init_pub.publish(msg)
            time.sleep(0.5)
        self.get_logger().info('初始位姿: x=%.2f y=%.2f yaw=%.2f（%s）'
                               % (pose[0], pose[1], pose[2],
                                  '存档' if confident else '无存档'))

    def stop_robot(self):
        msg = Twist()
        for _ in range(10):
            self.stop_pub.publish(msg)
            time.sleep(0.05)


def main():
    rclpy.init()
    node = ModeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_slam()
        node.stop_extras()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
