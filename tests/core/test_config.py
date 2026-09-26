import pytest
import yaml
from pydantic import ValidationError

from sorter.core.config import Backend, deep_merge, load_config
from sorter.core.types import Zone


def test_deep_merge_merges_nested_and_replaces_leaves():
    base = {"a": {"x": 1, "y": [1, 2]}, "b": 1}
    over = {"a": {"y": [3]}, "c": 2}
    assert deep_merge(base, over) == {"a": {"x": 1, "y": [3]}, "b": 1, "c": 2}
    assert base == {"a": {"x": 1, "y": [1, 2]}, "b": 1}


def test_committed_config_loads():
    cfg = load_config()
    assert cfg.backends.camera is Backend.SIM
    assert set(cfg.zones) == {Zone.BOX, Zone.BACKGROUND}
    assert {"rest", "home", "look_box", "look_bg", "place_bg"} <= set(cfg.poses)


def test_files_merge_in_order(tmp_path):
    (tmp_path / "default.yaml").write_text(yaml.safe_dump({"dashboard": {"port": 1, "host": "a"}}))
    (tmp_path / "rig.yaml").write_text(yaml.safe_dump({"dashboard": {"port": 2}}))
    (tmp_path / "local.yaml").write_text(yaml.safe_dump({"backends": {"camera": "real"}}))
    cfg = load_config(tmp_path, overrides={"dashboard": {"host": "b"}})
    assert (cfg.dashboard.host, cfg.dashboard.port) == ("b", 2)
    assert cfg.backends.camera is Backend.REAL


def test_unknown_section_is_rejected(tmp_path):
    (tmp_path / "default.yaml").write_text("dashbord: {}\n")
    with pytest.raises(ValidationError):
        load_config(tmp_path)


def test_missing_default_file(tmp_path):
    with pytest.raises(FileNotFoundError):
        load_config(tmp_path)
