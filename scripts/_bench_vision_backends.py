"""Time the DINOv3 / SigLIP vision backbones in both backends, in one process.

The HTTP server's own numbers cannot answer "is the TensorRT switch worth it":
``/eval_dual`` returns after the policy forward, the vision features are cached
across requests, and the reported fields do not add up to the measured wall
time.  So this times the two towers directly -- the torch modules the server
builds for ``--dino_backend torch``, then the same cacher with the TRT proxies
the server installs for ``--dino_backend trt_direct`` swapped in, exactly as
``_maybe_install_dino_trt`` / ``_maybe_install_siglip_trt`` do.

It also compares the features themselves (cosine similarity), because a faster
backend that hands the policy different tokens is not a drop-in replacement.

Runs inside the follow container; needs no ROS and touches no robot hardware.
"""

from __future__ import annotations

import os
import statistics
import sys
import time
from pathlib import Path

import numpy as np
import torch
from PIL import Image

TRACK_ROOT = Path(os.environ["MINICPM_ROBOT_TRACK_ROOT"])
sys.path.insert(0, str(TRACK_ROOT))
sys.path.insert(0, str(TRACK_ROOT / "realworld" / "sample"))

from minicpm_robot_track.vision_cache import (  # noqa: E402
    VisionCacheConfig,
    VisionFeatureCacher,
)
from dino_trt_direct import DinoTrtDirect  # noqa: E402
from siglip_trt_direct import SigLipTrtDirect  # noqa: E402

ITERS = 15
WARMUP = 3


def frame(seed: int = 0) -> Image.Image:
    """A 384x384 person-ish frame; only the timing matters here."""

    rng = np.random.default_rng(seed)
    image = np.full((720, 1280, 3), 40, np.uint8)
    image[200:700, 560:720] = (170, 150, 130)
    image[120:300, 590:690] = (205, 185, 165)
    image = np.clip(image.astype(np.int16) + rng.integers(-6, 7, image.shape), 0, 255)
    return Image.fromarray(image.astype(np.uint8))


def timeit(fn, iters: int = ITERS) -> float:
    """Median ms per call, CUDA-synchronized."""

    for _ in range(WARMUP):
        fn()
    torch.cuda.synchronize()
    samples = []
    for _ in range(iters):
        torch.cuda.synchronize()
        start = time.perf_counter()
        fn()
        torch.cuda.synchronize()
        samples.append((time.perf_counter() - start) * 1000.0)
    return statistics.median(samples)


def main() -> int:
    device = "cuda" if torch.cuda.is_available() else "cpu"
    cfg = VisionCacheConfig(
        image_size=384,
        batch_size=1,
        device=device,
        compute_dtype=torch.float32,
        vision_amp_dtype=torch.bfloat16,   # --vision_amp bf16, the deployed value
        timing_sync=False,
        timing_mode="fast",
    )
    image = frame()

    print("[bench] building torch vision cache ...")
    build_t0 = time.perf_counter()
    cacher = VisionFeatureCacher(cfg)
    cacher.eval()
    print(f"[bench] torch cache ready in {time.perf_counter() - build_t0:.1f}s")

    torch_dino_ms = timeit(lambda: cacher._encode_dino([image]))
    dino_tokens_torch, hp, wp = cacher._encode_dino([image])
    torch_siglip_ms = timeit(lambda: cacher._encode_siglip([image], (hp, wp)))
    siglip_tokens_torch = cacher._encode_siglip([image], (hp, wp))
    print(f"[bench] torch   DINO {torch_dino_ms:.1f} ms   SigLIP {torch_siglip_ms:.1f} ms"
          f"   (tokens {tuple(dino_tokens_torch.shape)} / {tuple(siglip_tokens_torch.shape)})")

    print("[bench] swapping in the TRT engines ...")
    swap_t0 = time.perf_counter()
    # The name has to be dropped first: the cacher is an nn.Module, and torch
    # refuses to rebind a child-module attribute to a plain object.  The server
    # does the same del-then-assign in _maybe_install_*_trt.
    try:
        del cacher.dino
    except AttributeError:
        pass
    cacher.dino = DinoTrtDirect(output_tokens=576)
    cacher.dino_hidden, cacher.dino_patch, cacher.dino_regs = 384, 16, 0
    try:
        del cacher.siglip
    except AttributeError:
        pass
    cacher.siglip = SigLipTrtDirect(output_tokens=576)
    cacher.siglip_hidden = 1152
    torch.cuda.empty_cache()
    print(f"[bench] TRT proxies ready in {time.perf_counter() - swap_t0:.1f}s")

    trt_dino_ms = timeit(lambda: cacher._encode_dino([image]))
    dino_tokens_trt, hp2, wp2 = cacher._encode_dino([image])
    trt_siglip_ms = timeit(lambda: cacher._encode_siglip([image], (hp2, wp2)))
    siglip_tokens_trt = cacher._encode_siglip([image], (hp2, wp2))
    print(f"[bench] trt     DINO {trt_dino_ms:.1f} ms   SigLIP {trt_siglip_ms:.1f} ms"
          f"   (tokens {tuple(dino_tokens_trt.shape)} / {tuple(siglip_tokens_trt.shape)})")

    def cosine(a: torch.Tensor, b: torch.Tensor) -> float:
        a = a.reshape(-1).float()
        b = b.reshape(-1).float()
        if a.numel() != b.numel():
            return float("nan")
        return float(torch.nn.functional.cosine_similarity(a, b, dim=0))

    print()
    print(f"[bench] DINO   speedup x{torch_dino_ms / trt_dino_ms:.2f}"
          f"   token cosine {cosine(dino_tokens_torch, dino_tokens_trt):.5f}")
    print(f"[bench] SigLIP speedup x{torch_siglip_ms / trt_siglip_ms:.2f}"
          f"   token cosine {cosine(siglip_tokens_torch, siglip_tokens_trt):.5f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
