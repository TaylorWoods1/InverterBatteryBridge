"""PlatformIO build script to enable OTA updates."""

from typing import Any

# PlatformIO's build system provides these
env: Any = None  # type: ignore

Import("env")  # type: ignore

# Add OTA update configuration
env.Append(
    CFLAGS=["-DOTA_ENABLED"],
    CXXFLAGS=["-DOTA_ENABLED"],
    LINKFLAGS=["-DOTA_ENABLED"]
)

# Add OTA partition to the build
env.Replace(
    PARTITIONS_CSV="partitions_ota.csv"
) 