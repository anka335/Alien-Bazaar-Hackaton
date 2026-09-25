import shutil

import pytest
import yaml
from pydantic import ValidationError

from sorter.core.config import DEFAULT_CONFIG_DIR, Backend, deep_merge, load_config, update_yaml
from sorter.core.types import Zone


def test_deep_merge_merges_nested_and_replaces_leaves():
    base = {"a": {"x": 1, "y": [1, 2]}, "b": 1}
    over = {"a": {"y": [3]}, "c": 2}
    assert deep_merge(base, over) == {"a": {"x": 1, "y": [3]}, "b": 1, "c": 2}
    assert base == {"a": {"x": 1, "y": [1, 2]}, "b": 1}


def test_committed_config_loads(tmp_path):
    for name in ("default.yaml", "rig.yaml", "calibration.yaml"):  # not the machine's local.yaml
        if (DEFAULT_CONFIG_DIR / name).is_file():
            shutil.copy(DEFAULT_CONFIG_DIR / name, tmp_path / name)
    cfg = load_config(tmp_path)
    assert cfg.backends.camera is Backend.SIM
    assert set(cfg.sim.zones) == {Zone.BOX, Zone.BACKGROUND}


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


def test_update_yaml_merges_and_keeps_the_header(tmp_path):
    path = tmp_path / "rig.yaml"
    path.write_text("# header\n# two\nposes:\n  home: [0, 1]\nzones: {}\n")
    update_yaml(path, {"poses": {"rest": [1, 2]}})
    text = path.read_text()
    assert text.startswith("# header\n# two\n")
    assert yaml.safe_load(text) == {"poses": {"home": [0, 1], "rest": [1, 2]}, "zones": {}}
    update_yaml(tmp_path / "new.yaml", {"a": 1})
    assert yaml.safe_load((tmp_path / "new.yaml").read_text()) == {"a": 1}
