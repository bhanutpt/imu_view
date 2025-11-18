
"""Convert captured IMU records into per-sensor CSV files."""

import argparse
import base64
import csv
import itertools
import sys
from contextlib import ExitStack
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Iterator, Tuple

SOH = 0x02
RECORD_FRAME_LENGTH = 227
RECORD_DATA_LENGTH = 224
CRC_LENGTH = 2
RAW_FRAMES_PER_RECORD = 10
SUPPORTED_VERSIONS = {2}
FRAME_TYPE_RESPONSE = 0xF2
BLE_RESPONSE_COMMAND_IDS = {0x05, 0x06, 0x10, 0x11, 0x12}


def crc16_modbus(data: bytes) -> int:
    """Return Modbus CRC-16 for the provided payload."""
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def parse_record(record_bytes: bytes) -> Tuple[int, int, int, Tuple[Tuple[int, ...], ...], Dict[str, int]]:
    """Parse a single data frame and return record data."""
    if len(record_bytes) != RECORD_FRAME_LENGTH:
        raise ValueError(f"Expected {RECORD_FRAME_LENGTH} bytes, got {len(record_bytes)}")

    if record_bytes[0] != SOH:
        raise ValueError(f"Unexpected start byte: 0x{record_bytes[0]:02X}")

    payload = record_bytes[1 : 1 + RECORD_DATA_LENGTH]
    received_crc = int.from_bytes(record_bytes[-CRC_LENGTH:], byteorder="big")
    calculated_crc = crc16_modbus(record_bytes[:-CRC_LENGTH])
    if received_crc != calculated_crc:
        raise ValueError(
            f"CRC mismatch: received 0x{received_crc:04X}, expected 0x{calculated_crc:04X}"
        )

    version = int.from_bytes(payload[0:4], byteorder="little", signed=False)
    if version not in SUPPORTED_VERSIONS:
        raise ValueError(f"Unsupported record version {version}")

    record_num = int.from_bytes(payload[4:8], byteorder="little", signed=False)
    timestamp = int.from_bytes(payload[8:16], byteorder="little", signed=False)

    samples_bytes = payload[16:16 + RAW_FRAMES_PER_RECORD * 18]
    offset = 0
    frames = []
    for _ in range(RAW_FRAMES_PER_RECORD):
        ax = int.from_bytes(samples_bytes[offset : offset + 2], byteorder="little", signed=True)
        ay = int.from_bytes(samples_bytes[offset + 2 : offset + 4], byteorder="little", signed=True)
        az = int.from_bytes(samples_bytes[offset + 4 : offset + 6], byteorder="little", signed=True)
        gx = int.from_bytes(samples_bytes[offset + 6 : offset + 10], byteorder="little", signed=True)
        gy = int.from_bytes(samples_bytes[offset + 10 : offset + 14], byteorder="little", signed=True)
        gz = int.from_bytes(samples_bytes[offset + 14 : offset + 18], byteorder="little", signed=True)
        frames.append((ax, ay, az, gx, gy, gz))
        offset += 18

    aux_offset = 16 + RAW_FRAMES_PER_RECORD * 18
    aux_timestamp = int.from_bytes(payload[aux_offset : aux_offset + 8], byteorder="little", signed=False)
    aux_offset += 8
    pressure_pa_x10 = int.from_bytes(payload[aux_offset : aux_offset + 4], byteorder="little", signed=True)
    aux_offset += 4
    pressure_temp_cC = int.from_bytes(payload[aux_offset : aux_offset + 2], byteorder="little", signed=True)
    aux_offset += 2
    ir_object_temp_cC = int.from_bytes(payload[aux_offset : aux_offset + 2], byteorder="little", signed=True)
    aux_offset += 2
    ir_ambient_temp_cC = int.from_bytes(payload[aux_offset : aux_offset + 2], byteorder="little", signed=True)
    aux_offset += 2
    mag_uT_x = int.from_bytes(payload[aux_offset : aux_offset + 2], byteorder="little", signed=True)
    aux_offset += 2
    mag_uT_y = int.from_bytes(payload[aux_offset : aux_offset + 2], byteorder="little", signed=True)
    aux_offset += 2
    mag_uT_z = int.from_bytes(payload[aux_offset : aux_offset + 2], byteorder="little", signed=True)
    aux_offset += 2
    aux_flags = int.from_bytes(payload[aux_offset : aux_offset + 2], byteorder="little", signed=False)

    return (
        version,
        record_num,
        timestamp,
        tuple(frames),
        {
            "aux_timestamp": aux_timestamp,
            "pressure_pa_x10": pressure_pa_x10,
            "pressure_temp_cC": pressure_temp_cC,
            "ir_object_temp_cC": ir_object_temp_cC,
            "ir_ambient_temp_cC": ir_ambient_temp_cC,
            "mag_uT_x": mag_uT_x,
            "mag_uT_y": mag_uT_y,
            "mag_uT_z": mag_uT_z,
            "aux_flags": aux_flags,
        },
    )


def detect_input_format(path: Path) -> str:
    """Return 'binary' or 'base64' depending on the file contents."""
    with path.open("rb") as infile:
        sample_bytes = infile.read(max(RECORD_FRAME_LENGTH, 128))
    if not sample_bytes:
        raise ValueError(f"{path} is empty")

    stripped = sample_bytes.lstrip()
    if not stripped:
        raise ValueError(f"{path} only contains whitespace")

    if stripped[0] == SOH:
        return "binary"

    first_line = None
    try:
        with path.open("r", encoding="utf-8") as infile:
            for line in infile:
                candidate = line.strip()
                if candidate:
                    first_line = candidate
                    break
    except UnicodeDecodeError:
        first_line = None

    if first_line:
        try:
            decoded = base64.b64decode(first_line, validate=True)
        except base64.binascii.Error:
            pass
        else:
            if decoded and decoded[0] == SOH:
                return "base64"

    total_size = path.stat().st_size
    if total_size % RECORD_FRAME_LENGTH == 0:
        return "binary"

    raise ValueError(f"Unable to determine format for {path}")


def iter_records_base64(path: Path) -> Iterator[Tuple[int, int, int, Tuple[Tuple[int, ...], ...], Dict[str, int]]]:
    with path.open("r", encoding="utf-8") as infile:
        for line_number, line in enumerate(infile, start=1):
            stripped = line.strip()
            if not stripped:
                continue
            try:
                frame = base64.b64decode(stripped)
            except base64.binascii.Error as exc:  # noqa: B904
                raise ValueError(f"Line {line_number}: invalid base64 data") from exc
            if len(frame) != RECORD_FRAME_LENGTH:
                if skip_or_raise_non_record(frame, f"Line {line_number}"):
                    continue
            try:
                yield parse_record(frame)
            except ValueError as exc:
                raise ValueError(f"Line {line_number}: {exc}") from exc


def is_ble_response_frame(frame: bytes) -> bool:
    """Return True when the frame matches a BLE response (ACK/details/etc.)."""
    if len(frame) < 5:
        return False
    if frame[0] != SOH:
        return False
    frame_length = frame[1]
    if frame_length != len(frame):
        return False
    if frame[2] != FRAME_TYPE_RESPONSE:
        return False
    if frame[3] not in BLE_RESPONSE_COMMAND_IDS:
        return False
    return True


def skip_or_raise_non_record(frame: bytes, context: str) -> bool:
    """Return True if a non-record frame was skipped; raise for malformed payloads."""
    if is_ble_response_frame(frame):
        command = frame[3]
        print(
            f"{context}: skipping BLE response frame for command 0x{command:02X} ({len(frame)} bytes)",
            file=sys.stderr,
        )
        return True

    raise ValueError(f"{context}: unexpected frame length {len(frame)} (wanted {RECORD_FRAME_LENGTH})")


def iter_records_binary(path: Path) -> Iterator[Tuple[int, int, int, Tuple[Tuple[int, ...], ...], Dict[str, int]]]:
    with path.open("rb") as infile:
        for index in itertools.count():
            frame = infile.read(RECORD_FRAME_LENGTH)
            if not frame:
                break
            if len(frame) != RECORD_FRAME_LENGTH:
                raise ValueError(
                    f"Record {index}: expected {RECORD_FRAME_LENGTH} bytes, got {len(frame)}"
                )
            yield parse_record(frame)


def iter_records(path: Path) -> Iterator[Tuple[int, int, int, Tuple[Tuple[int, ...], ...], Dict[str, int]]]:
    input_format = detect_input_format(path)
    if input_format == "binary":
        yield from iter_records_binary(path)
    else:
        yield from iter_records_base64(path)


def sensor_output_paths(base_path: Path) -> Dict[str, Path]:
    return {
        "imu": base_path.with_name(f"{base_path.name}_imu.csv"),
        "pressure": base_path.with_name(f"{base_path.name}_pressure.csv"),
        "magnetometer": base_path.with_name(f"{base_path.name}_magnetometer.csv"),
        "infrared": base_path.with_name(f"{base_path.name}_infrared.csv"),
    }


def convert_to_csvs(input_path: Path, output_base_path: Path, annotate: bool) -> Dict[str, Path]:
    output_base_path.parent.mkdir(parents=True, exist_ok=True)
    output_paths = sensor_output_paths(output_base_path)

    with ExitStack() as stack:
        imu_writer = csv.writer(
            stack.enter_context(output_paths["imu"].open("w", encoding="utf-8", newline=""))
        )
        pressure_writer = csv.writer(
            stack.enter_context(output_paths["pressure"].open("w", encoding="utf-8", newline=""))
        )
        magnetometer_writer = csv.writer(
            stack.enter_context(output_paths["magnetometer"].open("w", encoding="utf-8", newline=""))
        )
        infrared_writer = csv.writer(
            stack.enter_context(output_paths["infrared"].open("w", encoding="utf-8", newline=""))
        )

        imu_header = [
            "record_version",
            "record_number",
            "record_timestamp",
            "sample_index",
            "ax",
            "ay",
            "az",
            "gx",
            "gy",
            "gz",
            "aux_flags",
        ]
        if annotate:
            imu_header.extend(["record_timestamp_iso", "record_delta_ms"])

        pressure_header = [
            "record_version",
            "record_number",
            "record_timestamp",
            "aux_timestamp",
            "pressure_pa_x10",
            "pressure_temp_cC",
            "aux_flags",
        ]
        if annotate:
            pressure_header.extend(["record_timestamp_iso", "record_delta_ms", "aux_timestamp_iso"])

        magnetometer_header = [
            "record_version",
            "record_number",
            "record_timestamp",
            "aux_timestamp",
            "mag_uT_x",
            "mag_uT_y",
            "mag_uT_z",
            "aux_flags",
        ]
        if annotate:
            magnetometer_header.extend(["record_timestamp_iso", "record_delta_ms", "aux_timestamp_iso"])

        infrared_header = [
            "record_version",
            "record_number",
            "record_timestamp",
            "aux_timestamp",
            "ir_object_temp_cC",
            "ir_ambient_temp_cC",
            "aux_flags",
        ]
        if annotate:
            infrared_header.extend(["record_timestamp_iso", "record_delta_ms", "aux_timestamp_iso"])

        imu_writer.writerow(imu_header)
        pressure_writer.writerow(pressure_header)
        magnetometer_writer.writerow(magnetometer_header)
        infrared_writer.writerow(infrared_header)

        last_timestamp = None

        for version, record_number, timestamp, frames, aux in iter_records(input_path):
            record_iso = ""
            delta_ms = 0
            aux_iso = ""

            if annotate:
                record_iso = datetime.fromtimestamp(timestamp / 1000, tz=timezone.utc).isoformat()
                if last_timestamp is not None:
                    delta_ms = timestamp - last_timestamp
                if aux["aux_timestamp"]:
                    aux_iso = datetime.fromtimestamp(
                        aux["aux_timestamp"] / 1000, tz=timezone.utc
                    ).isoformat()
            last_timestamp = timestamp

            for sample_index, (ax, ay, az, gx, gy, gz) in enumerate(frames):
                imu_row = [
                    version,
                    record_number,
                    timestamp,
                    sample_index,
                    ax,
                    ay,
                    az,
                    gx,
                    gy,
                    gz,
                    aux["aux_flags"],
                ]
                if annotate:
                    imu_row.extend([record_iso, delta_ms])
                imu_writer.writerow(imu_row)

            pressure_row = [
                version,
                record_number,
                timestamp,
                aux["aux_timestamp"],
                aux["pressure_pa_x10"],
                aux["pressure_temp_cC"],
                aux["aux_flags"],
            ]
            if annotate:
                pressure_row.extend([record_iso, delta_ms, aux_iso])
            pressure_writer.writerow(pressure_row)

            magnetometer_row = [
                version,
                record_number,
                timestamp,
                aux["aux_timestamp"],
                aux["mag_uT_x"],
                aux["mag_uT_y"],
                aux["mag_uT_z"],
                aux["aux_flags"],
            ]
            if annotate:
                magnetometer_row.extend([record_iso, delta_ms, aux_iso])
            magnetometer_writer.writerow(magnetometer_row)

            infrared_row = [
                version,
                record_number,
                timestamp,
                aux["aux_timestamp"],
                aux["ir_object_temp_cC"],
                aux["ir_ambient_temp_cC"],
                aux["aux_flags"],
            ]
            if annotate:
                infrared_row.extend([record_iso, delta_ms, aux_iso])
            infrared_writer.writerow(infrared_row)

    return output_paths


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert captured IMU record dumps into per-sensor CSV files."
    )
    parser.add_argument(
        "input",
        type=Path,
        help="Path to the .bin file produced by start_stop_rec_ses.py",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Base path for generated CSV files (defaults to <input> without suffix)",
    )
    parser.add_argument(
        "--annotate",
        action="store_true",
        help="Include ISO8601 timestamps and inter-record delta (ms) columns",
    )
    args = parser.parse_args()

    if args.output:
        output_base = args.output
    else:
        output_base = args.input.with_suffix("")

    if output_base.suffix:
        output_base = output_base.with_suffix("")

    output_paths = convert_to_csvs(args.input, output_base, args.annotate)
    print("Wrote:")
    for sensor, path in output_paths.items():
        print(f"  {sensor}: {path}")


if __name__ == "__main__":
    main()
