"""Private-environment audio worker. Dedicated inherited pipe for JSON only."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import select
import signal
import statistics
import sys
import time
import wave
from jetarm_release import ASR, LIVE, send, verify_release
from adaptive_segmenter import Segmenter, normalize_confirmation, rms


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--event-fd', type=int)
    parser.add_argument('--threshold', type=float, default=None)
    parser.add_argument('--record-dir')
    # 0 表示不设上限（节点常驻时使用）；默认沿用原版 300 秒
    parser.add_argument('--session-s', type=float, default=300.0)
    args = parser.parse_args(argv)
    if args.event_fd is None:
        print('ASR_WORKER_INERT')
        return 0
    if sys.platform != 'linux' or args.event_fd < 3:
        raise ValueError('linux_inherited_pipe_required')
    record_dir=Path(args.record_dir) if args.record_dir else None
    if record_dir is not None:
        allowed=Path(__file__).resolve().parent/'runs'
        if record_dir.is_symlink() or not record_dir.resolve().is_relative_to(allowed):
            raise ValueError('record_directory_outside_session')
        record_dir.mkdir(parents=True,exist_ok=True)
    utterances=0
    def trace(event, **fields):
        if record_dir is not None:
            with (record_dir/'capture-events.jsonl').open('a', encoding='utf-8') as log:
                log.write(json.dumps(dict(event=event, monotonic_s=time.monotonic(), **fields), ensure_ascii=False)+'\n')
    def interrupted(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)

    # 非阻塞命令通道：LISTEN / PAUSE / RESUME 任意时刻都能处理
    cmd_buf = bytearray()

    def poll_cmd():
        try:
            ready, _, _ = select.select([0], [], [], 0)
        except (OSError, ValueError):
            return None
        if not ready:
            return None
        chunk = os.read(0, 128)
        if not chunk:
            return 'EOF'
        cmd_buf.extend(chunk)
        if b'\n' in cmd_buf:
            line, _, rest = cmd_buf.partition(b'\n')
            cmd_buf[:] = rest
            return line.decode('utf-8', 'replace').strip()
        return None

    def wait_cmd(wanted):
        while True:
            cmd = poll_cmd()
            if cmd is None:
                time.sleep(0.02)
                continue
            if cmd == 'EOF':
                raise RuntimeError('stdin_eof')
            if cmd in wanted:
                return cmd
    with os.fdopen(args.event_fd, 'wb', buffering=0) as events:
        verify_release(ASR)
        verify_release(LIVE)
        spec = importlib.util.spec_from_file_location('_frozen_live_asr', LIVE / 'live_asr.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        from optimized_recognizer import load_recognizer
        recognize = load_recognizer()
        send(events, dict(event='model_ready', issued_s=time.monotonic()))
        wait_cmd(('LISTEN',))
        def transcribe(pcm):
            nonlocal utterances
            utterances+=1
            if record_dir is not None:
                with (record_dir/('utterance-%04d.wav'%utterances)).open('xb') as f:
                    with wave.open(f,'wb') as w:
                        w.setparams((1,2,16000,0,'NONE','not compressed'))
                        w.writeframes(pcm)
            text = recognize(pcm)
            if type(text) is str and len(text) <= 128:
                print('识别文字：' + (text or '（空）'), flush=True)
                send(events, dict(event='text', issued_s=time.monotonic(), text=normalize_confirmation(text)))
            else:
                send(events, dict(event='transcript_rejected', reason='too_long_or_invalid'))
            return text
        with module.Capture() as capture:
            print('麦克风预热及底噪采样约2秒，请先保持安静……', flush=True)
            for _ in range(50):
                capture.get()
            noise = statistics.median(rms(capture.get()) for _ in range(50))
            segmenter = Segmenter(args.threshold, noise)
            trace('capture_ready', noise_rms=noise, start_threshold=segmenter.start_threshold,
                  release_threshold=segmenter.release_threshold)
            send(events, dict(event='listen_ready', issued_s=time.monotonic()))
            print('底噪RMS=%.1f，起点阈值=%.1f，结束阈值=%.1f。' %
                  (noise, segmenter.start_threshold, segmenter.release_threshold), flush=True)
            print('可以说话。说完停顿约0.8秒。', flush=True)
            deadline = (time.monotonic()+args.session_s
                        if args.session_s and args.session_s > 0
                        else float('inf'))
            meter_frames=0
            meter_peak=0.
            paused = False
            while time.monotonic() < deadline:
                if paused:
                    # 暂停期间不能调用 capture.get()：accepting=False 时队列不
                    # 会进数据，get() 3 秒后就会超时崩溃。_pump 线程仍在排空管道。
                    cmd = poll_cmd()
                    if cmd == 'EOF':
                        raise RuntimeError('stdin_eof')
                    if cmd == 'RESUME':
                        paused = False
                        capture.set_listening(True)
                        segmenter.reset()
                        trace('owner_resumed')
                    time.sleep(0.02)
                    continue
                frame=capture.get()
                cmd = poll_cmd()
                if cmd == 'EOF':
                    raise RuntimeError('stdin_eof')
                if cmd == 'PAUSE':
                    paused = True
                    capture.set_listening(False)
                    segmenter.reset()
                    trace('owner_paused')
                    continue
                meter_frames+=1
                meter_peak=max(meter_peak, rms(frame))
                if meter_frames>=250:
                    trace('listening_window', frames=meter_frames, peak_frame_rms=meter_peak)
                    meter_frames=0
                    meter_peak=0.
                pcm = segmenter.feed(frame)
                if segmenter.notice:
                    capture.set_listening(False)
                    send(events, dict(event='transcript_rejected', reason=segmenter.notice))
                    segmenter.notice = None
                    wait_cmd(('RESUME',))
                    capture.set_listening(True)
                    trace('capture_resumed', reason='rejected_segment')
                    meter_frames=0;meter_peak=0.
                    continue
                if pcm is None:
                    continue
                capture.set_listening(False)
                trace('capture_paused', utterance=utterances+1, duration_s=len(pcm)/32000.)
                print('TIMING_SEGMENT_END_MONO=%.6f' % time.monotonic(), flush=True)
                print('识别中，请暂时不要说下一句……', flush=True)
                started = time.monotonic()
                transcribe(pcm)
                print('本句录音%.2f秒；推理%.3f秒。' %
                      (len(pcm)/32000., time.monotonic()-started), flush=True)
                # Capture was paused before transcription. The owner resumes it
                # only after its spoken question and speaker tail have finished.
                wait_cmd(('RESUME',))
                capture.set_listening(True)
                trace('capture_resumed', reason='dialog_ready')
                meter_frames=0;meter_peak=0.
                print('可以说下一句。', flush=True)
        send(events, dict(event='audio_done'))
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('音频工作进程已退出。', file=sys.stderr, flush=True)
        raise SystemExit(130)
    except Exception as exc:
        print('ASR_WORKER_FAILED:', type(exc).__name__, str(exc), file=sys.stderr, flush=True)
        raise SystemExit(1)
