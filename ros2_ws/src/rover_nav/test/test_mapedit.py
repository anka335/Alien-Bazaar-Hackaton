import numpy as np
import pytest
import yaml

from rover_nav.keepout import read_pgm, write_pgm
from rover_nav.mapedit import OCCUPIED, add_to_map, main

# 10 x 6 cells of 0.1 m: x from -0.5 to 0.5, y from -0.3 to 0.3, all free
SHAPE = (6, 10)


def _save_map(tmp_path, origin=(-0.5, -0.3, 0.0), negate=0):
    write_pgm(tmp_path / "room.pgm", np.full(SHAPE, 254, dtype=np.uint8))
    meta = {
        "image": "room.pgm",
        "mode": "trinary",
        "resolution": 0.1,
        "origin": list(origin),
        "negate": negate,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    (tmp_path / "room.yaml").write_text(yaml.safe_dump(meta))
    return str(tmp_path / "room.yaml")


def test_box_is_drawn_and_the_original_kept(tmp_path):
    map_yaml = _save_map(tmp_path)
    changed = add_to_map(map_yaml, [((-0.1, 0.0), (0.1, 0.2))], str(tmp_path / "room_nav"))
    assert changed == 4  # x centres -0.05, 0.05; y centres 0.05, 0.15
    img = read_pgm(str(tmp_path / "room_nav.pgm"))
    assert (img[1:3, 4:6] == OCCUPIED).all()
    assert (img == OCCUPIED).sum() == 4
    assert (read_pgm(str(tmp_path / "room.pgm")) == 254).all()  # original untouched
    meta = yaml.safe_load((tmp_path / "room_nav.yaml").read_text())
    assert meta["image"] == "room_nav.pgm"
    assert meta["origin"] == [-0.5, -0.3, 0.0] and meta["free_thresh"] == 0.196


def test_already_occupied_cells_are_not_counted(tmp_path):
    map_yaml = _save_map(tmp_path)
    add_to_map(map_yaml, [((-0.1, 0.0), (0.1, 0.2))], str(tmp_path / "a"))
    assert (
        add_to_map(str(tmp_path / "a.yaml"), [((-0.1, 0.0), (0.1, 0.2))], str(tmp_path / "b")) == 0
    )


def test_bad_inputs_are_refused(tmp_path):
    with pytest.raises(ValueError, match="nothing to add"):
        add_to_map(_save_map(tmp_path), [], str(tmp_path / "x"))
    with pytest.raises(ValueError, match="outside the map"):
        add_to_map(_save_map(tmp_path), [((5.0, 5.0), (6.0, 6.0))], str(tmp_path / "x"))
    with pytest.raises(ValueError, match="yaw"):
        add_to_map(_save_map(tmp_path, origin=(-0.5, -0.3, 0.3)), [((0, 0), (0.1, 0.1))], "x")
    with pytest.raises(ValueError, match="negate"):
        add_to_map(_save_map(tmp_path, negate=1), [((0, 0), (0.1, 0.1))], str(tmp_path / "x"))


def test_cli_default_output_and_no_overwrite(tmp_path, capsys):
    map_yaml = _save_map(tmp_path)
    assert main([map_yaml, "--rect", "-0.1", "0.0", "0.1", "0.2"]) == 0
    assert (tmp_path / "room_nav.pgm").exists() and (tmp_path / "room_nav.yaml").exists()
    assert "4 cells newly occupied" in capsys.readouterr().out
    assert main([map_yaml, "--rect", "0", "0", "0.1", "0.1", "-o", str(tmp_path / "room")]) == 1
