"""Bounded local utterance segmentation using measured ambient RMS."""
import array
from collections import deque
import math
import sys

FRAME_BYTES = 640
MAX_FRAMES = 500
QUIET_FRAMES = 40


def rms(frame):
    if len(frame) != FRAME_BYTES:
        raise ValueError('incomplete_frame')
    samples = array.array('h', frame)
    if sys.byteorder != 'little':
        samples.byteswap()
    return math.sqrt(sum(v*v for v in samples) / len(samples))


def thresholds(minimum, noise):
    if not math.isfinite(noise) or noise < 0:
        raise ValueError('invalid_noise_or_minimum')
    if minimum is not None and minimum > 0:
        # 显式指定阈值时直接采用，不再依赖启动那 2 秒的底噪标定
        start = float(minimum)
        if not math.isfinite(start) or start < 1:
            raise ValueError('invalid_noise_or_minimum')
        release = max(40.0, start * 0.65)
        return start, min(start, release)
    minimum = max(10.0, min(180.0, noise * 1.5))
    if any(not math.isfinite(x) or x < 0 for x in (minimum, noise)) or minimum < 1:
        raise ValueError('invalid_noise_or_minimum')
    # 自动模式下给上限兜底：启动标定可能被瞬时噪声污染（实测 4.8~305 都出现过）
    start = min(max(minimum, noise * 2.2, 60.0), 400.0)
    release = min(max(minimum * .65, noise * 1.6, 40.0), 300.0)
    return start, min(start, release)


class Segmenter:
    def __init__(self, minimum, noise):
        self.start_threshold, self.release_threshold = thresholds(minimum, noise)
        self.discarding = False
        self.notice = None
        self.reset()

    def reset(self):
        self.pre = deque(maxlen=10)
        self.parts = None
        self.consecutive = self.voiced = self.silent = 0

    def feed(self, frame):
        level = rms(frame)
        if self.discarding:
            self.silent = self.silent + 1 if level < self.release_threshold else 0
            if self.silent >= QUIET_FRAMES:
                self.discarding = False
                self.reset()
            return None
        if self.parts is None:
            self.pre.append(frame)
            self.consecutive = self.consecutive + 1 if level >= self.start_threshold else 0
            if self.consecutive >= 3:
                self.parts = list(self.pre)
                self.voiced = 3
            return None
        self.parts.append(frame)
        voiced = level >= self.release_threshold
        self.voiced += int(voiced)
        self.silent = 0 if voiced else self.silent + 1
        if len(self.parts) >= MAX_FRAMES:
            self.reset()
            self.discarding = True
            self.notice = 'overlong_utterance_rejected'
            return None
        if self.silent >= QUIET_FRAMES:
            result = b''.join(self.parts) if self.voiced >= 10 else None
            self.reset()
            return result
        return None


def normalize_confirmation(text):
    # These are emotion labels produced by the original model's postprocessor,
    # not words, units, event icons or arbitrary punctuation.
    emotion = ('😊', '😔', '😡', '😰', '🤢', '😮')
    if text.startswith('确认执行') and text[4:] in emotion:
        return '确认执行'
    return text
