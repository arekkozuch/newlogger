import socket
"""
Enhanced UDP Listener for GPS Logger Packets with IMU Data

This script listens for UDP packets from an enhanced GPS logger device that includes
both GPS data and IMU (accelerometer/gyroscope) data in a 40-byte packet format.

Updated Packet Structure (40 bytes total):
- 38 bytes payload:
  * GPS data: timestamp(4), latitude(4), longitude(4), altitude(4), speed(2), heading(4), fix_type(1), satellites(1)
  * Battery data: voltage_mv(2), percentage(1)
  * IMU data: accel_x(2), accel_y(2), accel_z(2), gyro_x(2), gyro_y(2)
  * Reserved byte(1)
- 2 bytes CRC16 (little-endian, calculated over the 38-byte payload)

Key Functions:
- crc16_xmodem(data: bytes) -> int: Computes CRC16-XMODEM checksum for given data.

Main Loop:
- Receives UDP packets.
- Validates packet length (40 bytes).
- Extracts and verifies CRC.
- Unpacks payload using struct format for GPS + IMU data.
- Converts and prints all fields including motion data.

Usage:
- Run the script to start listening for enhanced packets.
- Parsed GPS and IMU data is printed to the console for each valid packet.

Dependencies:
- socket, struct, binascii, datetime, math

Note:
- Updated for GPS logger v2.2 with MPU6000/MPU9250 IMU support
- Now handles 40-byte packets with proper heading precision (uint32_t)
- Handles both stationary and motion data
- Calculates total acceleration magnitude for motion detection
"""
import struct
import binascii
import math
from datetime import datetime

UDP_IP = "0.0.0.0"
UDP_PORT = 9000

# Updated structure for 40-byte packets with uint32_t heading
# Format: timestamp(4), lat(4), lon(4), alt(4), speed(2), heading(4), fix_type(1), sats(1),
#         batt_mv(2), batt_pct(1), accel_x(2), accel_y(2), accel_z(2), gyro_x(2), gyro_y(2), reserved(1)
STRUCT_FORMAT = "<IiiiHIBBHBhhhhhB"
PAYLOAD_SIZE = 38  # Actual payload size
TOTAL_PACKET_SIZE = 40  # Total packet size including CRC

print(f"🚀 Enhanced GPS Logger UDP Listener v2.3")
print(f"📡 Listening on UDP {UDP_IP}:{UDP_PORT}...")
print(f"🧪 Using struct format: {STRUCT_FORMAT}")
print(f"🧪 Expected payload size: {PAYLOAD_SIZE} bytes")
print(f"🧪 Expected total packet size: {TOTAL_PACKET_SIZE} bytes")
print(f"🔄 Enhanced features: GPS + IMU data + Motion detection")
print(f"🎯 Fixed: 32-bit heading precision (degrees × 1e5)")
print("=" * 80)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

def crc16_xmodem(data: bytes) -> int:
    """Calculate CRC16-XMODEM checksum"""
    crc = 0x0000
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if (crc & 0x8000):
                crc = ((crc << 1) ^ 0x1021)
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc

def classify_motion(magnitude):
    """Classify motion based on acceleration magnitude"""
    if magnitude < 0.8:
        return "🔻 Very Low"
    elif magnitude < 1.2:
        return "😴 Stationary"
    elif magnitude < 1.8:
        return "🚶 Walking"
    elif magnitude < 3.0:
        return "🏃 Running"
    elif magnitude < 5.0:
        return "🚗 Vehicle"
    else:
        return "💥 High Impact"

def format_fix_type(fix_type):
    """Convert fix type number to descriptive string"""
    fix_types = {
        0: "No Fix",
        1: "Dead Reckoning",
        2: "2D Fix",
        3: "3D Fix", 
        4: "GNSS + Dead Reckoning",
        5: "Time Only"
    }
    return fix_types.get(fix_type, f"Unknown ({fix_type})")

def format_heading(heading_raw):
    """Convert heading from degrees × 1e5 to readable format"""
    heading_deg = heading_raw / 100000.0
    # Normalize to 0-360 range
    heading_deg = heading_deg % 360.0
    
    # Add compass direction
    if heading_deg < 11.25 or heading_deg >= 348.75:
        direction = "N"
    elif heading_deg < 33.75:
        direction = "NNE"
    elif heading_deg < 56.25:
        direction = "NE"
    elif heading_deg < 78.75:
        direction = "ENE"
    elif heading_deg < 101.25:
        direction = "E"
    elif heading_deg < 123.75:
        direction = "ESE"
    elif heading_deg < 146.25:
        direction = "SE"
    elif heading_deg < 168.75:
        direction = "SSE"
    elif heading_deg < 191.25:
        direction = "S"
    elif heading_deg < 213.75:
        direction = "SSW"
    elif heading_deg < 236.25:
        direction = "SW"
    elif heading_deg < 258.75:
        direction = "WSW"
    elif heading_deg < 281.25:
        direction = "W"
    elif heading_deg < 303.75:
        direction = "WNW"
    elif heading_deg < 326.25:
        direction = "NW"
    else:
        direction = "NNW"
    
    return heading_deg, direction

packet_count = 0
last_heading = None

while True:
    data, addr = sock.recvfrom(1024)
    packet_count += 1
    
    print(f"\n📦 Packet #{packet_count} from {addr[0]}:{addr[1]}")
    print(f"📏 Received {len(data)} bytes")
    
    if len(data) != TOTAL_PACKET_SIZE:
        print(f"❌ Invalid packet size: {len(data)} bytes (expected {TOTAL_PACKET_SIZE})")
        print(f"🔹 Raw HEX: {binascii.hexlify(data)}")
        continue

    # Split payload and CRC
    payload = data[:PAYLOAD_SIZE]  # First 38 bytes
    crc_data = data[PAYLOAD_SIZE:TOTAL_PACKET_SIZE]  # Last 2 bytes

    # Verify CRC
    try:
        received_crc = struct.unpack("<H", crc_data)[0]
        computed_crc = crc16_xmodem(payload)
        
        crc_valid = received_crc == computed_crc
        
        if not crc_valid:
            print(f"❌ CRC mismatch! Received: {received_crc:04X}, Computed: {computed_crc:04X}")
            # Continue anyway to see if we can parse the data
            
    except Exception as e:
        print(f"❌ CRC processing error: {e}")
        continue

    # Unpack the enhanced payload
    try:
        expected_size = struct.calcsize(STRUCT_FORMAT)
        if len(payload) != expected_size:
            print(f"⚠️ Size mismatch: payload={len(payload)}, format expects={expected_size}")
            continue
        
        fields = struct.unpack(STRUCT_FORMAT, payload)
        
        (timestamp, lat, lon, alt, speed, heading_raw, fix_type, sats, 
         batt_mv, batt_pct, accel_x, accel_y, accel_z, gyro_x, gyro_y, reserved) = fields

        # Convert GPS data to human-readable units
        lat_deg = lat / 1e7
        lon_deg = lon / 1e7
        alt_m = alt / 1000.0
        speed_kmh = (speed / 1000.0) * 3.6  # mm/s to km/h
        heading_deg, compass_dir = format_heading(heading_raw)
        batt_v = batt_mv / 1000.0

        # Convert IMU data to human-readable units
        accel_x_g = accel_x / 1000.0  # mg to g
        accel_y_g = accel_y / 1000.0
        accel_z_g = accel_z / 1000.0
        gyro_x_dps = gyro_x / 100.0   # (deg/s * 100) to deg/s
        gyro_y_dps = gyro_y / 100.0

        # Calculate total acceleration magnitude
        accel_magnitude = math.sqrt(accel_x_g**2 + accel_y_g**2 + accel_z_g**2)
        
        # Format timestamp
        time_str = datetime.utcfromtimestamp(timestamp).strftime('%Y-%m-%d %H:%M:%S')
        
        # Check heading stability
        heading_status = ""
        if last_heading is not None:
            heading_diff = abs(heading_deg - last_heading)
            if heading_diff > 180:  # Handle 360° wraparound
                heading_diff = 360 - heading_diff
            if heading_diff > 5:
                heading_status = f" (Δ{heading_diff:.1f}°)"
        last_heading = heading_deg
        
        # Print parsed data in organized sections
        print("=" * 60)
        print("🕐 TIMESTAMP & LOCATION")
        print(f"   Time        : {time_str} UTC")
        print(f"   Latitude    : {lat_deg:.7f}°")
        print(f"   Longitude   : {lon_deg:.7f}°")
        print(f"   Altitude    : {alt_m:.2f} m")
        
        print("\n🛰️ GPS STATUS")
        print(f"   Fix Type    : {format_fix_type(fix_type)}")
        print(f"   Satellites  : {sats}")
        print(f"   Speed       : {speed_kmh:.1f} km/h")
        print(f"   Heading     : {heading_deg:.2f}° ({compass_dir}){heading_status}")
        print(f"   Raw Heading : {heading_raw} (×1e5)")
        
        print("\n🔋 POWER STATUS")
        print(f"   Battery     : {batt_v:.2f}V ({batt_pct}%)")
        
        print("\n🔄 MOTION DATA (IMU)")
        print(f"   Accelerometer: X={accel_x_g:+.1f}g, Y={accel_y_g:+.1f}g, Z={accel_z_g:+.1f}g")
        print(f"   Gyroscope    : X={gyro_x_dps:+.0f}°/s, Y={gyro_y_dps:+.0f}°/s")
        print(f"   Total Accel  : {accel_magnitude:.1f}g")
        print(f"   Motion Class : {classify_motion(accel_magnitude)}")
        
        # Motion alerts
        if abs(gyro_x_dps) > 15 or abs(gyro_y_dps) > 15:
            print(f"   🌀 Significant rotation detected!")
            
        if accel_magnitude > 2.5:
            print(f"   💥 HIGH IMPACT DETECTED! ({accel_magnitude:.1f}g)")
        elif accel_magnitude > 1.5 and speed_kmh > 5:
            print(f"   ⚡ Vehicle acceleration/braking detected")
        elif accel_magnitude < 0.5 and speed_kmh < 1:
            print(f"   🛑 Device appears stationary")
            
        print(f"\n📊 PACKET INFO")
        print(f"   CRC Check   : {'✅' if crc_valid else '❌'} (RX:{received_crc:04X} vs CALC:{computed_crc:04X})")
        print(f"   Reserved    : 0x{reserved:02X}")
        print(f"   Packet Rate : ~{1000/max(1, (datetime.now().timestamp()*1000) % 1000):.1f} Hz (estimated)")

        # Racing-style summary for high-speed scenarios
        if speed_kmh > 30:
            lateral_g = math.sqrt(accel_x_g**2 + accel_y_g**2)
            print(f"\n🏁 RACING METRICS")
            print(f"   Speed       : {speed_kmh:.1f} km/h ({speed_kmh/3.6:.1f} m/s)")
            print(f"   Heading     : {heading_deg:.1f}° {compass_dir}")
            print(f"   Lateral G   : {lateral_g:.1f}g")
            print(f"   Braking/Accel: {accel_y_g:+.1f}g")
            if lateral_g > 0.3:
                print(f"   🏁 Cornering detected! ({lateral_g:.1f}g)")

    except struct.error as e:
        print(f"❌ Packet parsing error: {e}")
        print(f"🔹 Raw HEX: {binascii.hexlify(data)}")
        print(f"🔹 Expected format: {STRUCT_FORMAT}")
        print(f"🔹 Expected size: {struct.calcsize(STRUCT_FORMAT)} bytes")
        
        # Manual hex analysis for debugging
        hex_str = binascii.hexlify(data).decode()
        print(f"🔍 Manual analysis of {len(data)} bytes:")
        for i in range(0, min(len(hex_str), 80), 8):  # Show first 40 bytes
            chunk = hex_str[i:i+8]
            byte_offset = i // 2
            print(f"   Bytes {byte_offset:2d}-{byte_offset+3:2d}: {chunk}")
            
        continue
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        continue