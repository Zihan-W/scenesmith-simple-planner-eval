"""Public environment composition entry point."""

from src.online_manipulation.protocols import (
    EnvironmentConfig,
    OnlineEnvironment,
)


def make_env(config: EnvironmentConfig) -> OnlineEnvironment:
    """Build an online environment from a public configuration object."""
    if not isinstance(config, EnvironmentConfig):
        raise TypeError("config must implement EnvironmentConfig")
    environment = config.build_environment()
    if not isinstance(environment, OnlineEnvironment):
        raise TypeError(
            "EnvironmentConfig.build_environment() must return an "
            "OnlineEnvironment"
        )
    return environment
