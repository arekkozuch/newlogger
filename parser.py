#!/usr/bin/env python3
"""
GPS BIN Parser

This script reads a binary GPS log file produced by the ESP32 GPS logger.
It skips the text header, parses each 38-byte record according to the packed
`GPSPacket` structure, verifies the CRC16, and can output parsed data and
statistics or save to CSV.
"""
import struct
import datetime
import sys

# Constants matching the C struct layout
RECORD_SIZE = 38           # bytes per record (including CRC)
HEADER_LINE = b'GPS_LOG_V1.0\n'

# CRC16 parameters match the embedded implementation (poly 0x1021, init 0x0000)
def crc16(data: bytes) -> int:
    """Compute CRC-16 (poly=0x1021) over the given data."""
    crc = 0x0000
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) & 0xFFFF) ^ 0x1021
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

# Struct format: little-endian
# Fields: timestamp, lat, lon, alt, speed, heading,
#         fixType, satellites, battery_mv, battery_pct,
#         accel_x, accel_y, accel_z, gyro_x, gyro_y,
#         reserved1, crc
FMT = '<IiiiHHBBHBhhhhhBH'

def parse_record(chunk: bytes) -> dict:
    """Unpack a single record and verify its CRC."""
    if len(chunk) != RECORD_SIZE:
        raise ValueError(f"Invalid record size: expected {RECORD_SIZE}, got {len(chunk)}")
    # Unpack all fields including CRC
    fields = struct.unpack(FMT, chunk)
    (timestamp, lat, lon, alt,
     speed, heading,
     fixType, sats,
     batt_mv, batt_pct,
     accel_x, accel_y, accel_z, gyro_x, gyro_y,
     reserved1, crc_stored) = fields

    # Verify CRC over payload (all but last 2 bytes)
    payload = chunk[:-2]
    crc_calc = crc16(payload)

    return {
        'timestamp': timestamp,
        'datetime_utc': datetime.datetime.utcfromtimestamp(timestamp),
        'latitude': lat * 1e-7,
        'longitude': lon * 1e-7,
        'altitude_m': alt / 1000.0,
        'speed_m_s': speed / 1000.0,
        'speed_kmh': speed * 3.6 / 1000.0,
        'heading_deg': heading / 100.0,
        'fix_type': fixType,
        'satellites': sats,
        'battery_mv': batt_mv,
        'battery_v': batt_mv / 1000.0,
        'battery_pct': batt_pct,
        'accel_x_g': accel_x / 1000.0,
        'accel_y_g': accel_y / 1000.0,
        'accel_z_g': accel_z / 1000.0,
        'gyro_x_dps': gyro_x / 100.0,
        'gyro_y_dps': gyro_y / 100.0,
        'crc_stored': crc_stored,
        'crc_calc': crc_calc,
        'crc_ok': crc_stored == crc_calc,
    }


def parse_file(path: str) -> list:
    """Read and parse all records from a GPS log file."""
    records = []
    with open(path, 'rb') as f:
        # Skip the text header line
        header = f.readline()
        if header != HEADER_LINE:
            print(f"Warning: unexpected header: {header!r}", file=sys.stderr)
        # Read records
        idx = 0
        while True:
            chunk = f.read(RECORD_SIZE)
            if not chunk:
                break
            if len(chunk) != RECORD_SIZE:
                print(f"Warning: incomplete record at index {idx}", file=sys.stderr)
                break
            rec = parse_record(chunk)
            records.append(rec)
            idx += 1
    return records


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <input.bin> [output.csv]")
        sys.exit(1)

    in_path = sys.argv[1]
    out_csv = sys.argv[2] if len(sys.argv) > 2 else None

    recs = parse_file(in_path)
    print(f"Total records parsed: {len(recs)}")
    if recs:
        t0 = recs[0]['datetime_utc']
        t1 = recs[-1]['datetime_utc']
        delta = (t1 - t0).total_seconds()
        print(f"Time span: {t0} to {t1} ({delta:.2f} s)")
        if len(recs) > 1 and delta > 0:
            print(f"Average rate: {len(recs)/delta:.2f} Hz")

    if out_csv and recs:
        import csv
        fieldnames = list(recs[0].keys())
        with open(out_csv, 'w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            for r in recs:
                writer.writerow(r)
        print(f"Data written to {out_csv}")

if __name__ == '__main__':
    main()
