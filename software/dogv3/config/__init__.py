"""Universal config: the single source of truth (``robot_config.json``)."""

from .schema import RobotConfig, SCHEMA_VERSION
from .loader import load_config, save_config, ConfigError

__all__ = ["RobotConfig", "SCHEMA_VERSION", "load_config", "save_config", "ConfigError"]
