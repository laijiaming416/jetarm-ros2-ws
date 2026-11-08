# -*- coding: utf-8 -*-
"""ASR 私有环境的发布校验与小工具。

从 voice-dialog-ep-20261101-v1/bridge_io.py 摘出 ASR 节点需要的部分，
去掉底盘 SDK 相关内容。
"""

import hashlib
import json
from pathlib import Path

HOME = Path('/home/laijinxuan')
ASR = HOME / 'file-asr-original-20261024-v1'
LIVE = HOME / 'live-asr-20261024-v1'
ANCHORS = {
    'file-asr-original-20261024-v1':
        'e275432e0f29d366db927c3998b3819a80347ea40597e31e3b3ab088b13aec7e',
    'live-asr-20261024-v1':
        '9acad8c1413ef9464c7903795b00dd6b34687eeb5b203c0754cdd5af77915013',
}


def verify_release(root):
    if root.is_symlink() or root.resolve() != root.absolute():
        raise ValueError('release_root')
    manifest = root / 'SHA256SUMS'
    if manifest.is_symlink():
        raise ValueError('release_manifest_link')
    data = manifest.read_bytes()
    if hashlib.sha256(data).hexdigest() != ANCHORS[root.name]:
        raise ValueError('release_manifest_anchor:' + root.name)
    for line in data.decode('ascii').splitlines():
        digest, name = line.split('  ', 1)
        path = root / name
        if path.is_symlink() or not path.is_file() or \
                not path.resolve().is_relative_to(root.resolve()):
            raise ValueError('release_member:' + name)
        hasher = hashlib.sha256()
        with path.open('rb') as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b''):
                hasher.update(block)
        if hasher.hexdigest() != digest:
            raise ValueError('release_digest:' + name)


def send(stream, row):
    data = (json.dumps(row, ensure_ascii=False, allow_nan=False) + '\n').encode('utf-8')
    if len(data) > 60000:
        raise ValueError('outgoing_message_size')
    stream.write(data)
    stream.flush()