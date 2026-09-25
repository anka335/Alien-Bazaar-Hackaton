"""Client of the remote SAM3 segmentation service (D-012): image + text prompt → instance masks."""

from __future__ import annotations

import ast
import json
import os
import urllib.error
import urllib.request
import uuid
from dataclasses import dataclass

import cv2
import numpy as np

from sorter.color_classifier.config import SamConfig
from sorter.core.errors import SorterError

API_KEY_ENV = "SAM3_API_KEY"


class SegmentationError(SorterError):
    """The segmentation service is unreachable, refused the request, or sent a bad answer."""


@dataclass(frozen=True)
class Instance:
    mask: np.ndarray  # HxW bool
    score: float


def decode_rle(rle: dict | str) -> np.ndarray:
    """COCO RLE (compressed `counts` string or a list of runs) → HxW bool mask."""
    if isinstance(rle, str):
        rle = ast.literal_eval(rle)
    h, w = rle["size"]
    counts = rle["counts"]
    if isinstance(counts, str):
        counts = _counts_from_string(counts)
    flat = np.zeros(h * w, dtype=bool)
    pos = 0
    for i, n in enumerate(counts):
        if i % 2:
            flat[pos : pos + n] = True
        pos += n
    return flat.reshape((h, w), order="F")  # COCO runs are column-major


def _counts_from_string(s: str) -> list[int]:
    """pycocotools `rleFrString`: 6-bit chunks with a sign bit, deltas from two runs back."""
    counts: list[int] = []
    p = 0
    while p < len(s):
        x, k, more = 0, 0, True
        while more:
            c = ord(s[p]) - 48
            x |= (c & 0x1F) << (5 * k)
            more = bool(c & 0x20)
            p += 1
            k += 1
            if not more and c & 0x10:
                x |= -1 << (5 * k)
        if len(counts) > 2:
            x += counts[-2]
        counts.append(x)
    return counts


class SamSegmenter:
    """`segment(bgr)` → instances found for `cfg.prompt`. Raises SegmentationError on failure."""

    def __init__(self, cfg: SamConfig):
        self.cfg = cfg
        self.api_key = cfg.api_key or os.environ.get(API_KEY_ENV, "")
        if not self.api_key:
            raise SegmentationError(
                f"no SAM3 API key: set color_classifier.sam.api_key in config/local.yaml "
                f"or the {API_KEY_ENV} env var"
            )

    def segment(self, bgr: np.ndarray) -> list[Instance]:
        ok, png = cv2.imencode(".png", bgr)
        if not ok:
            raise SegmentationError("could not encode the frame")
        fields = {
            "prompt": self.cfg.prompt,
            "threshold": str(self.cfg.threshold),
            "mask_threshold": str(self.cfg.mask_threshold),
            "output": "json",
        }
        body, content_type = _multipart(fields, "image", "frame.png", png.tobytes())
        req = urllib.request.Request(
            self.cfg.url.rstrip("/") + "/segment",
            data=body,
            headers={"Content-Type": content_type, "X-API-Key": self.api_key},
            method="POST",
        )
        try:
            with urllib.request.urlopen(req, timeout=self.cfg.timeout_s) as resp:
                data = json.loads(resp.read())
        except urllib.error.HTTPError as e:
            detail = e.read()[:200].decode(errors="replace")
            raise SegmentationError(f"SAM3 service: HTTP {e.code} {detail}") from None
        except (urllib.error.URLError, TimeoutError, OSError) as e:
            raise SegmentationError(f"SAM3 service unreachable: {e}") from None
        except ValueError as e:
            raise SegmentationError(f"SAM3 service: bad JSON: {e}") from None
        return self._parse(data, bgr.shape[:2])

    @staticmethod
    def _parse(data: dict, shape: tuple[int, int]) -> list[Instance]:
        try:
            out = []
            for inst in data["instances"]:
                mask = decode_rle(inst["mask_rle"])
                if mask.shape != shape:
                    raise SegmentationError(f"mask {mask.shape} does not match frame {shape}")
                out.append(Instance(mask, float(inst["score"])))
            return out
        except (KeyError, TypeError, ValueError, SyntaxError) as e:
            raise SegmentationError(f"SAM3 service: unexpected answer: {e!r}") from None


def _multipart(fields: dict[str, str], file_field: str, filename: str, content: bytes):
    boundary = uuid.uuid4().hex
    parts = []
    for name, value in fields.items():
        head = f'--{boundary}\r\nContent-Disposition: form-data; name="{name}"\r\n\r\n'
        parts.append(f"{head}{value}\r\n".encode())
    parts.append(
        (
            f'--{boundary}\r\nContent-Disposition: form-data; name="{file_field}"; '
            f'filename="{filename}"\r\nContent-Type: image/png\r\n\r\n'
        ).encode()
        + content
        + b"\r\n"
    )
    parts.append(f"--{boundary}--\r\n".encode())
    return b"".join(parts), f"multipart/form-data; boundary={boundary}"
