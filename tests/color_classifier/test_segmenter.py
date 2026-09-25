"""SAM3 client: RLE decoding, request / answer handling, errors. No network except the live test."""

import io
import json
import os
import urllib.error

import cv2
import numpy as np
import pytest

from sorter.color_classifier import segmenter as seg
from sorter.color_classifier.config import SamConfig
from sorter.color_classifier.segmenter import SamSegmenter, SegmentationError, decode_rle


def encode_rle(mask: np.ndarray) -> dict:
    """pycocotools `rleToString`, to test the decoder against."""
    flat = mask.flatten(order="F").astype(np.uint8)
    counts, prev, n = [], 0, 0
    for x in flat:
        if x != prev:
            counts.append(n)
            prev, n = x, 0
        n += 1
    counts.append(n)
    out = []
    for i, x in enumerate(counts):
        if i > 2:
            x -= counts[i - 2]
        more = True
        while more:
            c = x & 0x1F
            x >>= 5
            more = (x != -1) if c & 0x10 else (x != 0)
            if more:
                c |= 0x20
            out.append(chr(c + 48))
    return {"size": list(mask.shape), "counts": "".join(out)}


def _random_mask(seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    m = np.zeros((48, 64), np.uint8)
    for _ in range(4):
        c = tuple(int(v) for v in rng.integers(0, [64, 48]))
        cv2.circle(m, c, int(rng.integers(3, 20)), 1, -1)
    return m.astype(bool)


@pytest.mark.parametrize("seed", range(5))
def test_decode_compressed_rle(seed):
    mask = _random_mask(seed)
    assert np.array_equal(decode_rle(encode_rle(mask)), mask)
    assert np.array_equal(decode_rle(str(encode_rle(mask))), mask)  # a repr string works too


def test_decode_uncompressed_rle():
    mask = decode_rle({"size": [2, 3], "counts": [1, 2, 3]})
    # column-major: (0,0) off, (1,0) (0,1) on, rest off
    assert mask.tolist() == [[False, True, False], [True, False, False]]


def test_missing_api_key(monkeypatch):
    monkeypatch.delenv(seg.API_KEY_ENV, raising=False)
    with pytest.raises(SegmentationError, match="API key"):
        SamSegmenter(SamConfig(api_key=""))


class _Resp(io.BytesIO):
    def __enter__(self):
        return self

    def __exit__(self, *a):
        pass


def test_segment_request_and_answer(monkeypatch):
    mask = _random_mask(1)
    sent = {}

    def urlopen(req, timeout):
        sent.update(url=req.full_url, headers=dict(req.header_items()), body=req.data)
        answer = {"instances": [{"score": 0.8, "mask_rle": encode_rle(mask)}]}
        return _Resp(json.dumps(answer).encode())

    monkeypatch.setattr(seg.urllib.request, "urlopen", urlopen)
    s = SamSegmenter(SamConfig(url="http://sam/", api_key="k", prompts=["clothing"]))
    out = s.segment(np.zeros((48, 64, 3), np.uint8))

    assert sent["url"] == "http://sam/segment"
    assert sent["headers"]["X-api-key"] == "k"
    assert b'name="prompt"\r\n\r\nclothing' in sent["body"]
    assert b"\x89PNG" in sent["body"]
    assert len(out) == 1 and out[0].score == 0.8 and np.array_equal(out[0].mask, mask)


def test_one_request_per_prompt(monkeypatch):
    masks = {"clothing": _random_mask(1), "sock": _random_mask(2)}

    def urlopen(req, timeout):
        prompt = req.data.split(b'name="prompt"\r\n\r\n')[1].split(b"\r\n")[0].decode()
        answer = {"instances": [{"score": 0.9, "mask_rle": encode_rle(masks[prompt])}]}
        return _Resp(json.dumps(answer).encode())

    monkeypatch.setattr(seg.urllib.request, "urlopen", urlopen)
    out = SamSegmenter(SamConfig(api_key="k", prompts=["clothing", "sock"])).segment(
        np.zeros((48, 64, 3), np.uint8)
    )
    assert len(out) == 2
    assert np.array_equal(out[0].mask, masks["clothing"])
    assert np.array_equal(out[1].mask, masks["sock"])


@pytest.mark.parametrize(
    "error",
    [
        urllib.error.HTTPError("u", 401, "no", {}, io.BytesIO(b'{"detail":"bad key"}')),
        urllib.error.URLError("down"),
        TimeoutError("slow"),
    ],
)
def test_service_errors_raise(monkeypatch, error):
    def urlopen(req, timeout):
        raise error

    monkeypatch.setattr(seg.urllib.request, "urlopen", urlopen)
    with pytest.raises(SegmentationError):
        SamSegmenter(SamConfig(api_key="k")).segment(np.zeros((8, 8, 3), np.uint8))


def test_bad_answer_raises(monkeypatch):
    monkeypatch.setattr(seg.urllib.request, "urlopen", lambda req, timeout: _Resp(b'{"x": 1}'))
    with pytest.raises(SegmentationError, match="unexpected"):
        SamSegmenter(SamConfig(api_key="k")).segment(np.zeros((8, 8, 3), np.uint8))


@pytest.mark.skipif(not os.environ.get(seg.API_KEY_ENV), reason="needs SAM3_API_KEY and network")
def test_live_service():
    img = np.full((240, 320, 3), 124, np.uint8)
    cv2.circle(img, (160, 120), 50, (40, 40, 200), -1)
    out = SamSegmenter(SamConfig(prompts=["blob"], threshold=0.2)).segment(img)
    assert all(i.mask.shape == (240, 320) for i in out)
