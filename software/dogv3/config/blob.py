"""Versioned robot_config blob compiler for onboard firmware storage."""
from __future__ import annotations

import argparse
import binascii
import json
import struct
import sys
from dataclasses import dataclass
from pathlib import Path

from pydantic import ValidationError

from .loader import ConfigError, load_config
from .schema import RobotConfig

BLOB_MAGIC = b"SDBCFG1\0"
BLOB_VERSION = 1
HEADER_STRUCT = struct.Struct("<8sHHII")  # magic, blob_version, schema_version, payload_len, crc32
HEADER_LEN = HEADER_STRUCT.size


@dataclass(frozen=True)
class ConfigBlobInfo:
    blob_version: int
    schema_version: int
    payload_len: int
    crc32: int
    firmware_expected: str
    fully_commissioned: bool

    def to_dict(self) -> dict:
        return {
            "blob_version": self.blob_version,
            "schema_version": self.schema_version,
            "payload_len": self.payload_len,
            "crc32": f"0x{self.crc32:08x}",
            "firmware_expected": self.firmware_expected,
            "fully_commissioned": self.fully_commissioned,
        }


class ConfigBlobError(Exception):
    """Raised when a config blob is malformed or fails CRC validation."""


def canonical_config_payload(config: RobotConfig) -> bytes:
    """Return stable JSON bytes used as the blob payload."""
    data = config.model_dump(mode="json")
    return json.dumps(data, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("ascii")


def build_config_blob(config: RobotConfig) -> bytes:
    payload = canonical_config_payload(config)
    crc = binascii.crc32(payload) & 0xFFFFFFFF
    header = HEADER_STRUCT.pack(BLOB_MAGIC, BLOB_VERSION, config.schema_version, len(payload), crc)
    return header + payload


def parse_config_blob(blob: bytes) -> tuple[ConfigBlobInfo, RobotConfig]:
    if len(blob) < HEADER_LEN:
        raise ConfigBlobError(f"blob too short: {len(blob)} bytes")
    magic, blob_version, schema_version, payload_len, expected_crc = HEADER_STRUCT.unpack(blob[:HEADER_LEN])
    if magic != BLOB_MAGIC:
        raise ConfigBlobError(f"bad magic: {magic!r}")
    if blob_version != BLOB_VERSION:
        raise ConfigBlobError(f"blob_version {blob_version} != supported {BLOB_VERSION}")
    payload = blob[HEADER_LEN:]
    if len(payload) != payload_len:
        raise ConfigBlobError(f"payload_len {payload_len} != actual {len(payload)}")
    actual_crc = binascii.crc32(payload) & 0xFFFFFFFF
    if actual_crc != expected_crc:
        raise ConfigBlobError(f"crc32 0x{actual_crc:08x} != expected 0x{expected_crc:08x}")
    try:
        raw = json.loads(payload.decode("ascii"))
        config = RobotConfig.model_validate(raw)
    except (UnicodeDecodeError, json.JSONDecodeError, ValidationError) as e:
        raise ConfigBlobError(f"payload is not a valid RobotConfig: {e}") from e
    if config.schema_version != schema_version:
        raise ConfigBlobError(f"header schema_version {schema_version} != payload {config.schema_version}")
    info = ConfigBlobInfo(
        blob_version=blob_version,
        schema_version=schema_version,
        payload_len=payload_len,
        crc32=actual_crc,
        firmware_expected=config.firmware_expected,
        fully_commissioned=config.fully_commissioned(),
    )
    return info, config


def compile_config_blob(config_path: str | Path, *, require_commissioned: bool = True) -> bytes:
    config = load_config(config_path, require_commissioned=require_commissioned)
    return build_config_blob(config)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Compile or inspect DogV3 robot_config blobs for onboard firmware storage")
    ap.add_argument("--config", default=None, help="robot_config.json to compile")
    ap.add_argument("--out", default=None, help="write compiled blob to this path")
    ap.add_argument("--inspect", default=None, help="inspect an existing blob")
    ap.add_argument("--allow-uncommissioned", action="store_true", help="compile even if commissioning is incomplete")
    ap.add_argument("--json", action="store_true", help="print blob metadata as JSON")
    args = ap.parse_args(argv)

    try:
        if args.inspect:
            info, _ = parse_config_blob(Path(args.inspect).read_bytes())
        else:
            if not args.config:
                ap.error("give --config robot_config.json or --inspect blob.bin")
            blob = compile_config_blob(args.config, require_commissioned=not args.allow_uncommissioned)
            info, _ = parse_config_blob(blob)
            if args.out:
                Path(args.out).write_bytes(blob)
    except (ConfigError, ConfigBlobError, OSError) as e:
        print(f"dogv3-config-blob: {e}", file=sys.stderr)
        return 2

    if args.json:
        print(json.dumps(info.to_dict(), indent=2))
    else:
        print("DogV3 config blob:")
        for key, value in info.to_dict().items():
            print(f"  {key}: {value}")
        if args.out:
            print(f"  wrote: {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
