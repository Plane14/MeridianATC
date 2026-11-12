"""PCM boundaries shared by capture, recognition and playback."""

from __future__ import annotations

import numpy as np


def pcm(audio, *, mono: bool = False) -> np.ndarray:
    samples = np.asarray(audio, dtype=np.float32)
    if samples.ndim not in (1, 2) or (samples.ndim == 2 and samples.shape[1] == 0):
        raise ValueError("audio must contain frames, optionally with channels")
    samples = np.clip(np.nan_to_num(samples, nan=0.0, posinf=1.0, neginf=-1.0), -1.0, 1.0)
    if mono and samples.ndim == 2:
        samples = samples.mean(axis=1)
    return np.ascontiguousarray(samples, dtype=np.float32)


def rate(value) -> int:
    try:
        result = int(value)
        if isinstance(value, bool) or result <= 0 or float(value) != result:
            raise ValueError
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("sample rate must be a positive integer") from exc
    return result
