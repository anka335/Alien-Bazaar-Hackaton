import numpy as np
import pytest
import yaml

from rover_nav.keepout import (
    ALLOWED,
    FORBIDDEN,
    forbidden_rect,
    forbidden_side,
    main,
    make_keepout,
    read_pgm,
    write_pgm,
)

# 10 x 6 cells of 0.1 m: x from -0.5 to 0.5, y from -0.3 to 0.3
RES, ORIGIN, SHAPE = 0.1, (-0.5, -0.3), (6, 10)


def _save_map(tmp_path, origin=(-0.5, -0.3, 0.0)):
    write_pgm(tmp_path / "room.pgm", np.full(SHAPE, 205, dtype=np.uint8))
    meta = {
        "image": "room.pgm",
        "mode": "trinary",
        "resolution": RES,
        "origin": list(origin),
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    (tmp_path / "room.yaml").write_text(yaml.safe_dump(meta))
    return str(tmp_path / "room.yaml")


def test_vertical_line_forbids_the_other_side():
    mask = forbidden_side(SHAPE, RES, ORIGIN, (0.0, -1.0), (0.0, 1.0), keep=(-0.2, 0.0))
    assert not mask[:, :5].any()  # x < 0: kept
    assert mask[:, 5:].all()  # x > 0: forbidden


def test_rows_run_top_down():
    # keep y > 0: the top rows of the image (row 0 = largest y) stay allowed
    mask = forbidden_side(SHAPE, RES, ORIGIN, (-1.0, 0.0), (1.0, 0.0), keep=(0.0, 0.2))
    assert not mask[:3].any()
    assert mask[3:].all()


def test_line_direction_does_not_matter():
    a = forbidden_side(SHAPE, RES, ORIGIN, (0.0, -1.0), (0.3, 1.0), keep=(-0.4, 0.0))
    b = forbidden_side(SHAPE, RES, ORIGIN, (0.3, 1.0), (0.0, -1.0), keep=(-0.4, 0.0))
    assert (a == b).all()
    assert 0 < a.mean() < 1


def test_keep_on_the_line_or_degenerate_line_is_refused():
    with pytest.raises(ValueError, match="on the line"):
        forbidden_side(SHAPE, RES, ORIGIN, (0.0, -1.0), (0.0, 1.0), keep=(0.0, 0.5))
    with pytest.raises(ValueError, match="two different points"):
        forbidden_side(SHAPE, RES, ORIGIN, (0.1, 0.1), (0.1, 0.1), keep=(0.0, 0.5))


def test_pgm_round_trip_with_comment(tmp_path):
    img = np.arange(60, dtype=np.uint8).reshape(SHAPE)
    path = tmp_path / "x.pgm"
    path.write_bytes(b"P5\n# CREATOR: map_saver\n10 6\n255\n" + img.tobytes())
    assert (read_pgm(str(path)) == img).all()
    write_pgm(str(path), img)
    assert (read_pgm(str(path)) == img).all()


def test_make_keepout_writes_a_matching_mask(tmp_path):
    share = make_keepout(
        _save_map(tmp_path),
        str(tmp_path / "keepout"),
        line=((0.0, -1.0), (0.0, 1.0), (-0.2, 0.0)),
    )
    assert share == pytest.approx(0.5)
    img = read_pgm(str(tmp_path / "keepout.pgm"))
    assert img.shape == SHAPE
    assert (img[:, :5] == ALLOWED).all() and (img[:, 5:] == FORBIDDEN).all()
    meta = yaml.safe_load((tmp_path / "keepout.yaml").read_text())
    assert meta["image"] == "keepout.pgm"
    assert meta["resolution"] == RES
    assert meta["origin"] == [-0.5, -0.3, 0.0]
    assert meta["mode"] == "trinary" and meta["negate"] == 0


def test_rotated_map_origin_is_refused(tmp_path):
    with pytest.raises(ValueError, match="yaw"):
        make_keepout(
            _save_map(tmp_path, origin=(-0.5, -0.3, 0.5)),
            str(tmp_path / "keepout"),
            line=((0.0, -1.0), (0.0, 1.0), (-0.2, 0.0)),
        )


def test_cli_defaults_next_to_the_map(tmp_path, capsys):
    map_yaml = _save_map(tmp_path)
    assert main([map_yaml, "--line", "0", "-1", "0", "1", "--keep", "-0.2", "0"]) == 0
    assert (tmp_path / "keepout.pgm").exists() and (tmp_path / "keepout.yaml").exists()
    assert "50% of the map is forbidden" in capsys.readouterr().out
    assert (
        main([str(tmp_path / "missing.yaml"), "--line", "0", "0", "1", "1", "--keep", "1", "0"])
        == 1
    )


def test_rect_forbids_only_cells_inside():
    # corners in any order; cells with centres x in [-0.2, 0.2], y in [0.0, 0.2]
    mask = forbidden_rect(SHAPE, RES, ORIGIN, (0.2, 0.2), (-0.2, 0.0))
    assert mask.sum() == 4 * 2
    assert mask[1:3, 3:7].all()  # rows 1-2 = y 0.15, 0.05; cols 3-6 = x -0.15 .. 0.15
    with pytest.raises(ValueError, match="no area"):
        forbidden_rect(SHAPE, RES, ORIGIN, (0.1, 0.0), (0.1, 0.2))


def test_line_and_rect_combine(tmp_path):
    share = make_keepout(
        _save_map(tmp_path),
        str(tmp_path / "keepout"),
        line=((0.0, -1.0), (0.0, 1.0), (-0.2, 0.0)),  # x > 0 forbidden
        rects=[((-0.5, -0.3), (-0.3, 0.3))],  # plus the two leftmost columns
    )
    img = read_pgm(str(tmp_path / "keepout.pgm"))
    assert (img[:, :2] == FORBIDDEN).all() and (img[:, 2:5] == ALLOWED).all()
    assert (img[:, 5:] == FORBIDDEN).all()
    assert share == pytest.approx(0.7)


def test_cli_rect_only_and_argument_checks(tmp_path, capsys):
    map_yaml = _save_map(tmp_path)
    assert main([map_yaml, "--forbid-rect", "-0.5", "-0.3", "-0.3", "0.3"]) == 0
    assert "20% of the map is forbidden" in capsys.readouterr().out
    assert main([map_yaml]) == 1  # nothing to forbid
    with pytest.raises(SystemExit):
        main([map_yaml, "--line", "0", "-1", "0", "1"])  # --line without --keep
