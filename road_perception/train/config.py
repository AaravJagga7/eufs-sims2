import yaml


def load_config(path, **overrides):
    """Read a YAML settings file; any non-None override replaces a value."""
    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    for k, v in overrides.items():
        if v is not None:
            cfg[k] = v
    return cfg