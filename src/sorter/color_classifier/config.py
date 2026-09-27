"""Config models for block 4 (color classifier)."""

from pydantic import BaseModel, ConfigDict, Field


class SamConfig(BaseModel):
    """`color_classifier.sam`: the remote SAM3 segmentation service (D-012)."""

    model_config = ConfigDict(extra="forbid")

    url: str = "https://exes-shape-zoning.ngrok-free.dev"
    api_key: str = ""  # empty = env var SAM3_API_KEY; put a real key only in config/local.yaml
    prompts: list[str] = Field(default=["clothing", "sock"], min_length=1)  # one request each
    threshold: float = 0.5  # instance score
    mask_threshold: float = 0.5
    timeout_s: float = 10.0


class ColorClassifierConfig(BaseModel):
    """`color_classifier`: segmentation service, class thresholds, erosion, item area."""

    model_config = ConfigDict(extra="forbid")

    sam: SamConfig = Field(default_factory=SamConfig)
    min_area_px: int = 1500  # smaller blobs are noise (or the tip of a finger)
    max_area_frac: float = 0.8  # a blob covering more of the ROI is the background itself
    overlap_max: float = 0.5  # drop an instance this much covered by a better-scored one
    erode_px: int = 5  # erode the mask before color stats: shadows, edges
    chroma_colored: float = 20.0  # median Lab chroma at or above → colored
    lightness_dark: float = 22.0  # median L* below → dark whatever the chroma
    # median L* below and chroma below → dark too: a dim, muted color (navy, dark brown), which
    # a lamp or a slanting view lifts over `lightness_dark`; colored socks are brighter or bolder
    lightness_dim: float = 35.0
    chroma_muted: float = 35.0
    lightness_light: float = 55.0  # median L* (0..100) at or above → light, else dark
    confidence_margin: float = 10.0  # distance from a threshold that gives confidence 1
    grasp_inset_px: int = 15  # the re-grasp point is at least this far inside the blob
    grasp_depth_tol_mm: float = 5.0  # points this close to the highest count as the highest
    depth_window_px: int = 5  # window for the robust depth of the grasp point
