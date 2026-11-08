"""Same frozen SenseVoice weights, deterministic frontend and real ITN option."""
import hashlib
from pathlib import Path
import sys
import time

BASE=Path('/home/laijinxuan/file-asr-original-20261024-v1')
LIB_HASH='3077e8aa1ccd0d115e877814f1972260aefe51f5d57781acc3d05403ec8e4e20'


def load_recognizer():
    if Path(sys.prefix).resolve()!=BASE/'.venv':raise RuntimeError('original_private_venv_required')
    library=BASE/'lib/libortextensions.so.0.12.0'
    if hashlib.sha256(library.read_bytes()).hexdigest()!=LIB_HASH:raise ValueError('decoder_extension_digest')
    sys.path.insert(0,str(BASE))
    import numpy as np
    import onnxruntime as ort
    if not Path(ort.__file__).resolve().is_relative_to(BASE/'.venv'):raise ValueError('private_ort_required')
    from official_models.sensevoice_bin import SenseVoiceSmall
    from official_models.postprocess_utils import rich_transcription_postprocess
    from run_file_asr import unwrap_text
    start=time.monotonic()
    model=SenseVoiceSmall(str(BASE/'models'),batch_size=1,quantize=True,extension_path=library)
    model.frontend.opts.frame_opts.dither=0.
    print('原SenseVoice已加载：%.3f秒，关闭随机扰动，启用数字规范化。'%(time.monotonic()-start),flush=True)
    def recognize(pcm):
        if type(pcm) is not bytes or len(pcm)%2:raise ValueError('PCM16_required')
        waveform=np.frombuffer(pcm,dtype='<i2').astype(np.float32)/32768.
        return rich_transcription_postprocess(unwrap_text(model(waveform,language='zh',textnorm='withitn')))
    return recognize
