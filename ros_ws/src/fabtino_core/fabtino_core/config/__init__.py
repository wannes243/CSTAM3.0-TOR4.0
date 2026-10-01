from __future__ import annotations

from pathlib import Path
import json


def load_config(path: str | Path | None = None) -> dict:
    if path is None:
        yaml_path = Path(__file__).resolve().parent / "robot_parameters.yaml"
        path = yaml_path if yaml_path.exists() else Path(__file__).resolve().parent / "robot_parameters.json"
    path = Path(path)
    with open(path, "r", encoding="utf-8") as f:
        if path.suffix.lower() in (".yaml", ".yml"):
            try:
                import yaml
            except ImportError as exc:
                raise RuntimeError(
                    "YAML configuration requires PyYAML. Install with: python -m pip install PyYAML"
                ) from exc
            return yaml.safe_load(f)
        return json.load(f)
