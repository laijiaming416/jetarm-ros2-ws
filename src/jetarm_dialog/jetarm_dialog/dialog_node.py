#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""对话节点：唤醒词门控 + 任务关键词。

- 唤醒词默认「你好小赖」，同音别名可配；唤醒后 8 秒内接受命令。
- 「停止/取消/停下/别动」不需要唤醒，任何时候生效。
- 任务意图只发 /voice/intent，不自己播报（由 mission 节点负责反馈）。
- 唤醒成功只播 80ms 提示音，不合成语音，省掉 10 秒合成时间。

话题：
    /voice/text    VoiceText        输入
    /voice/intent  Intent           输出：mission_start / mission_home / stop
    /voice/reply   std_msgs/String  输出：没听懂时的提示语
服务：
    /voice/cue     Empty            唤醒提示音（TTS 订阅后播放）
"""

import re
import time

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import Empty, String

from jetarm_interfaces.msg import Intent, VoiceText

PUNCT = ' \t\r\n，。！？、：；“”‘’（）,.!?%'
STOP_WORDS = ('停止', '取消', '停下', '别动', '不用了')
START_WORDS = ('去拿东西', '去取东西', '帮我拿', '拿东西', '取东西',
               '去拿', '去哪', '去那', '去呢', '标记', '标记的点',
               '出发', '开始任务', '开始')
# 识别经常把「拿」听成「哪/那」，唤醒后只要求含这类动词
START_CHARS = ('去', '拿', '取')
HOME_WORDS = ('回来', '回原点', '回到起点', '返回起点', '回去', '返航')


class DialogNode(Node):

    def __init__(self):
        super().__init__('dialog_node')

        self.declare_parameter('wake_word', '你好小车')
        self.declare_parameter('wake_aliases', [
            '你好小车', '你好小扯', '你好小澈', '你好小陈',
            '你好小程', '你好小成', '你好你好'])
        # 识别经常把名字吃掉，只留下“你好”；命中前缀也算唤醒
        self.declare_parameter('wake_prefixes', ['你好'])
        self.declare_parameter('wake_window_s', 8.0)
        self.declare_parameter('wake_cue', True)

        self.wake_word = str(self.get_parameter('wake_word').value).strip()
        self.aliases = [str(a) for a in
                        self.get_parameter('wake_aliases').value if str(a)]
        if self.wake_word and self.wake_word not in self.aliases:
            self.aliases.insert(0, self.wake_word)
        self.aliases.sort(key=len, reverse=True)
        self.prefixes = [str(a) for a in
                         self.get_parameter('wake_prefixes').value if str(a)]
        self.prefixes.sort(key=len, reverse=True)
        self.window = float(self.get_parameter('wake_window_s').value)
        self.cue_enabled = bool(self.get_parameter('wake_cue').value)
        self.wake_until = 0.0

        self.create_subscription(VoiceText, '/voice/text', self.on_text, 10,
                                 callback_group=ReentrantCallbackGroup())
        self.intent_pub = self.create_publisher(Intent, '/voice/intent', 10)
        self.reply_pub = self.create_publisher(String, '/voice/reply', 10)
        self.wake_pub = self.create_publisher(String, '/voice/wake', 10)
        # 只发话题，不做任何同步服务调用：rclpy 的 client.call() 没有超时，
        # 一旦某个服务不回就会把回调线程永久卡住。
        self.cue_pub = self.create_publisher(Empty, '/voice/cue', 10)
        self.resume_pub = self.create_publisher(Empty, '/asr/resume_cmd', 10)
        self.get_logger().info('dialog_node 启动，唤醒词=%s，窗口 %.0f 秒'
                               % ('/'.join(self.aliases), self.window))

    @staticmethod
    def _clean(text):
        # SenseVoice 会带 。，😔 这类符号，而且“你好，小赖”中间常有逗号；
        # 统一去掉所有标点、空白和表情，避免别名匹配被打断。
        text = re.sub(r'[\U0001F300-\U0001FAFF\u2600-\u27BF\uFE0F]', '',
                      text)
        return ''.join(ch for ch in text
                       if ch not in PUNCT and not ch.isspace())

    def _cue(self):
        if self.cue_enabled:
            self.cue_pub.publish(Empty())
        # 唤醒词本身是一句识别结果，worker 已停采；立刻恢复才能接住命令
        self.resume_pub.publish(Empty())

    def _reply(self, text):
        msg = String()
        msg.data = text
        self.reply_pub.publish(msg)

    def _intent(self, action, detail=''):
        msg = Intent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.action = action
        msg.detail = detail
        self.intent_pub.publish(msg)
        self.get_logger().info('意图：%s（%s）' % (action, detail))

    def _match_wake(self, text):
        """返回 (命中的词, 剩余命令文本)；没命中返回 (None, text)。

        先按别名找（取最长），再按前缀找。前缀命中时如果剩余部分还是
        前缀/别名（如“你好你好”），视为纯唤醒。
        """
        for alias in self.aliases:
            index = text.find(alias)
            if index >= 0:
                rest = (text[:index] + text[index + len(alias):]).strip(PUNCT)
                return alias, rest
        for prefix in self.prefixes:
            index = text.find(prefix)
            if index >= 0:
                rest = (text[:index] + text[index + len(prefix):]).strip(PUNCT)
                if rest in self.aliases or rest in self.prefixes:
                    rest = ''
                return prefix, rest
        return None, text

    def _wake(self, source):
        msg = String()
        msg.data = source
        self.wake_pub.publish(msg)

    def on_text(self, msg):
        text = self._clean(str(msg.text))
        if not text:
            return
        now = time.monotonic()

        if any(word in text for word in STOP_WORDS):
            self.wake_until = 0.0
            self._wake('stop')
            self._intent('stop', text)
            return

        alias, rest = self._match_wake(text)
        if alias is not None:
            self.wake_until = now + self.window
            is_command = (any(w in rest for w in START_WORDS)
                          or any(ch in rest for ch in START_CHARS)
                          or any(w in rest for w in HOME_WORDS))
            if not rest or (len(rest) <= 3 and not is_command):
                # “你好小车”常被识别成“你好傻车/你好小熊”，后半截不是命令时
                # 一律按纯唤醒处理，直接给提示音
                self.get_logger().info('唤醒词命中：%s（尾音：%s）'
                                       % (alias, rest))
                self._wake(alias)
                self._cue()
                return
            self.get_logger().info('唤醒词+命令：%s / %s' % (alias, rest))
            self._wake(alias)
            text = rest
        elif now > self.wake_until:
            self.get_logger().info('未唤醒，忽略：%s' % text)
            return

        # 先判返航：「回去」里也含「去」，顺序不能反
        if any(word in text for word in HOME_WORDS):
            self.wake_until = 0.0
            self._intent('mission_home', text)
            return
        if (any(word in text for word in START_WORDS)
                or any(ch in text for ch in START_CHARS)):
            self.wake_until = 0.0
            self._intent('mission_start', text)
            return

        self.wake_until = 0.0
        self._reply('没听清。')


def main(args=None):
    rclpy.init(args=args)
    node = DialogNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()