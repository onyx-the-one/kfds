import os
import sys
import time
import select
import serial
from polysense_proto import (
    FrameParser, build_frame,
    PS_MSG_ENV_TELEM, PS_MSG_IMU_TELEM, PS_MSG_QKD_INITIATE, PS_MSG_QKD_RESPONSE,
    PS_MSG_QKD_SIFTED, PS_MSG_QKD_VERIFY,
    QkdResponse, QkdVerify,
    encode_qkd_initiate, encode_qkd_sifted,
    bit_get, bit_set, pack_bits, unpack_bits,
)

NODE_ID = 0x01
NUM_BITS = 100


def run_bb84(ser, parser):
    """Run a full BB84 simulated QKD exchange."""
    print("\n" + "=" * 60)
    print("BB84 Quantum Key Distribution — Simulated Exchange")
    print("=" * 60)

    # Step 1: Generate random bits and bases
    raw_bits_list = [int(b) for b in bin(int.from_bytes(os.urandom(13), 'big'))[2:].zfill(104)[:NUM_BITS]]
    gs_bases_list = [int(b) for b in bin(int.from_bytes(os.urandom(13), 'big'))[2:].zfill(104)[:NUM_BITS]]

    raw_bits = pack_bits(raw_bits_list)
    gs_bases = pack_bits(gs_bases_list)

    print(f"\n[GS] Generated {NUM_BITS} random bits and bases")
    print(f"[GS] First 20 raw bits:  {raw_bits_list[:20]}")
    print(f"[GS] First 20 GS bases:  {gs_bases_list[:20]}")

    # Send QKD_INITIATE
    frame = encode_qkd_initiate(NODE_ID, raw_bits, gs_bases, NUM_BITS)
    ser.write(frame)
    print(f"\n[GS -> SAT] QKD_INITIATE sent ({len(frame)} bytes)")

    # Step 2: Wait for QKD_RESPONSE
    print("[GS] Waiting for QKD_RESPONSE...")
    response = None
    deadline = time.time() + 10.0
    while time.time() < deadline:
        data = ser.read(256)
        if not data:
            continue
        for f in parser.feed(data):
            if f.msg_type == PS_MSG_QKD_RESPONSE and isinstance(f.payload, QkdResponse):
                response = f.payload
                break
        if response:
            break

    if not response:
        print("[GS] ERROR: No QKD_RESPONSE received (timeout)")
        return

    sat_bases_list = unpack_bits(response.sat_bases, NUM_BITS)
    print(f"\n[SAT -> GS] QKD_RESPONSE received")
    print(f"[GS] First 20 SAT bases: {sat_bases_list[:20]}")

    # Step 3: Compute match mask and select check bits
    match_list = [1 if gs_bases_list[i] == sat_bases_list[i] else 0 for i in range(NUM_BITS)]
    matched_positions = [i for i in range(NUM_BITS) if match_list[i]]
    num_matched = len(matched_positions)

    print(f"\n[GS] Bases matched at {num_matched}/{NUM_BITS} positions")

    # Sacrifice ~half of matched positions for error checking
    check_positions = set(matched_positions[:num_matched // 2])
    check_list = [0] * NUM_BITS
    check_bits_list = [0] * NUM_BITS
    for i in range(NUM_BITS):
        if i in check_positions:
            check_list[i] = 1
            check_bits_list[i] = raw_bits_list[i]

    match_mask = pack_bits(match_list)
    check_mask = pack_bits(check_list)
    check_bits = pack_bits(check_bits_list)

    num_check = len(check_positions)
    num_key = num_matched - num_check
    print(f"[GS] Sacrificing {num_check} bits for error check, {num_key} bits for key")

    # Send QKD_SIFTED
    frame = encode_qkd_sifted(NODE_ID, match_mask, check_mask, check_bits, NUM_BITS)
    ser.write(frame)
    print(f"\n[GS -> SAT] QKD_SIFTED sent ({len(frame)} bytes)")

    # Extract local key (same logic as satellite)
    key_bits_list = []
    for i in range(NUM_BITS):
        if match_list[i] and not check_list[i]:
            key_bits_list.append(raw_bits_list[i])

    key_bytes = pack_bits(key_bits_list)
    key_len = (len(key_bits_list) + 7) // 8

    print(f"[GS] Local key: {len(key_bits_list)} bits = {key_len} bytes")
    print(f"[GS] Key bytes (hex): {key_bytes[:key_len].hex()}")

    # Step 4: Wait for QKD_VERIFY
    print("\n[GS] Waiting for QKD_VERIFY...")
    verify = None
    deadline = time.time() + 10.0
    while time.time() < deadline:
        data = ser.read(256)
        if not data:
            continue
        for f in parser.feed(data):
            if f.msg_type == PS_MSG_QKD_VERIFY and isinstance(f.payload, QkdVerify):
                verify = f.payload
                break
        if verify:
            break

    if not verify:
        print("[GS] ERROR: No QKD_VERIFY received (timeout)")
        return

    print(f"\n[SAT -> GS] QKD_VERIFY received")
    print(f"  Status:      {'OK' if verify.status == 0 else 'ERROR'}")
    print(f"  Errors:      {verify.error_count}/{verify.check_count}")
    print(f"  Key bits:    {verify.key_len_bits}")
    print(f"  Encrypted:   {verify.encrypted_msg.hex()}")

    # Decrypt the proof message
    if key_len > 0:
        decrypted = bytes(
            verify.encrypted_msg[i] ^ key_bytes[i % key_len]
            for i in range(verify.encrypted_len)
        )
    else:
        decrypted = verify.encrypted_msg[:verify.encrypted_len]

    try:
        decrypted_str = decrypted.decode('utf-8')
    except UnicodeDecodeError:
        decrypted_str = decrypted.hex()

    print(f"  Decrypted:   \"{decrypted_str}\"")
    print("\n" + "=" * 60)
    if decrypted_str == "KFDS26 su najlepsi":
        print("BB84 exchange SUCCESSFUL — shared key verified!")
    else:
        print("BB84 exchange COMPLETED — decryption mismatch (check key derivation)")
    print("=" * 60 + "\n")


def main():
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    parser = FrameParser()

    print("PolySense Ground Station Console")
    print("Press 'q' for BB84 QKD exchange, Ctrl+C to exit")
    print("-" * 40)

    try:
        while True:
            # Check for keyboard input (non-blocking on Unix)
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                if key == 'q':
                    run_bb84(ser, parser)
                    continue

            data = ser.read(256)
            if not data:
                continue
            for frame in parser.feed(data):
                if frame.msg_type == PS_MSG_ENV_TELEM:
                    p = frame.payload
                    T = p.temperature_c_x100 / 100.0
                    P = p.pressure_pa
                    print(f"ENV: T={T:.2f} C, P={P} Pa")
                elif frame.msg_type == PS_MSG_IMU_TELEM:
                    p = frame.payload
                    ax = p.ax_mps2_x1000 / 1000.0
                    ay = p.ay_mps2_x1000 / 1000.0
                    az = p.az_mps2_x1000 / 1000.0
                    gx = p.gx_dps_x1000 / 1000.0
                    gy = p.gy_dps_x1000 / 1000.0
                    gz = p.gz_dps_x1000 / 1000.0
                    print(
                        f"IMU: ax={ax:.3f} ay={ay:.3f} az={az:.3f}  "
                        f"gx={gx:.3f} gy={gy:.3f} gz={gz:.3f}"
                    )
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
