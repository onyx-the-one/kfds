from __future__ import annotations
import os
import struct
from dataclasses import dataclass
from typing import Optional, Tuple, Union, Iterator, List

PS_SYNC_LO = 0x55
PS_SYNC_HI = 0xAA
PS_PROTO_VER = 0x01

# Message types
PS_MSG_ENV_TELEM  = 0x01
PS_MSG_IMU_TELEM  = 0x02
PS_MSG_GPS_TELEM  = 0x03
PS_MSG_STATUS     = 0x04
PS_MSG_BEACON     = 0x05
PS_MSG_CMD        = 0x10
PS_MSG_CMD_ACK    = 0x11

PS_MSG_QKD_INITIATE = 0x20
PS_MSG_QKD_RESPONSE = 0x21
PS_MSG_QKD_SIFTED   = 0x22
PS_MSG_QKD_VERIFY   = 0x23

QKD_PROOF_MSG_MAX = 32

# Command classes
PS_CMD_CLASS_RECOVERY = 0
PS_CMD_CLASS_MOTOR    = 1
PS_CMD_CLASS_MISSION  = 2
PS_CMD_CLASS_CONFIG   = 3

# Mission states
PS_MSTATE_IDLE      = 0
PS_MSTATE_PRELAUNCH = 1
PS_MSTATE_ASCENT    = 2
PS_MSTATE_DESCENT   = 3
PS_MSTATE_LANDED    = 4
PS_MSTATE_RECOVERY  = 5

# CMD_ACK status
PS_CMD_STATUS_RECEIVED         = 0
PS_CMD_STATUS_ACCEPTED         = 1
PS_CMD_STATUS_DONE             = 2
PS_CMD_STATUS_REJECTED_INVALID = 3
PS_CMD_STATUS_REJECTED_STATE   = 4
PS_CMD_STATUS_ERROR_INTERNAL   = 5


def crc16_ccitt(data: bytes, init: int = 0xFFFF) -> int:
    crc = init
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


# -------------------------
# Dataclasses for payloads
# -------------------------

@dataclass
class EnvTelem:
    uptime_ms: int
    sample_id: int
    temperature_c_x100: int
    pressure_pa: int
    humidity_rh_x100: int
    env_flags: int


@dataclass
class ImuTelem:
    uptime_ms: int
    sample_id: int
    ax_mps2_x1000: int
    ay_mps2_x1000: int
    az_mps2_x1000: int
    gx_dps_x1000: int
    gy_dps_x1000: int
    gz_dps_x1000: int
    imu_flags: int
    qw_x10000: int
    qx_x10000: int
    qy_x10000: int
    qz_x10000: int


@dataclass
class GpsTelem:
    uptime_ms: int
    sample_id: int
    lat_deg_x1e7: int
    lon_deg_x1e7: int
    alt_m_x100: int
    ground_speed_cmps: int
    course_deg_x100: int
    hdop_x100: int
    sat_count: int
    fix_type: int
    gps_flags: int


@dataclass
class Status:
    uptime_ms: int
    battery_mv: int
    battery_pct_x100: int
    storage_free_kb: int
    mission_state: int
    recovery_flags: int
    link_rssi_dbm: int
    link_snr_db_x10: int
    status_flags: int


@dataclass
class Beacon:
    uptime_ms: int
    last_fix_age_ms: int
    lat_deg_x1e7: int
    lon_deg_x1e7: int
    alt_m_x100: int
    battery_mv: int
    recovery_flags: int
    beacon_seq: int


@dataclass
class CmdHeader:
    cmd_id: int
    cmd_class: int
    cmd_code: int
    param_len: int
    params: bytes


@dataclass
class CmdAck:
    cmd_id: int
    status: int
    detail: int
    timestamp_ms: int


@dataclass
class QkdInitiate:
    raw_bits: bytes   # 13 bytes, 100 bits packed MSB first
    gs_bases: bytes   # 13 bytes
    num_bits: int     # = 100


@dataclass
class QkdResponse:
    sat_bases: bytes  # 13 bytes
    num_bits: int     # = 100


@dataclass
class QkdSifted:
    match_mask: bytes  # 13 bytes
    check_mask: bytes  # 13 bytes
    check_bits: bytes  # 13 bytes
    num_bits: int      # = 100


@dataclass
class QkdVerify:
    error_count: int
    check_count: int
    key_len_bits: int
    encrypted_msg: bytes
    encrypted_len: int
    status: int


@dataclass
class Frame:
    ver: int
    msg_type: int
    node_id: int
    payload: Union[EnvTelem, ImuTelem, GpsTelem, Status, Beacon, CmdHeader, CmdAck,
                   QkdInitiate, QkdResponse, QkdSifted, QkdVerify, bytes]


# -------------------------
# Encode helpers
# -------------------------

def build_frame(node_id: int, msg_type: int, payload_bytes: bytes) -> bytes:
    hdr = struct.pack(
        "<BBBH",
        PS_PROTO_VER,
        msg_type,
        node_id & 0xFF,
        len(payload_bytes),
    )
    crc = crc16_ccitt(hdr + payload_bytes)
    return bytes([PS_SYNC_LO, PS_SYNC_HI]) + hdr + payload_bytes + struct.pack("<H", crc)


def encode_env_telem(node_id: int, p: EnvTelem) -> bytes:
    payload = struct.pack(
        "<IIhIHH6s",
        p.uptime_ms,
        p.sample_id,
        p.temperature_c_x100,
        p.pressure_pa,
        p.humidity_rh_x100,
        p.env_flags,
        b"\x00" * 6,
    )
    return build_frame(node_id, PS_MSG_ENV_TELEM, payload)


def encode_imu_telem(node_id: int, p: ImuTelem) -> bytes:
    payload = struct.pack(
        "<IIhhhhhhHhhhh10s",
        p.uptime_ms,
        p.sample_id,
        p.ax_mps2_x1000,
        p.ay_mps2_x1000,
        p.az_mps2_x1000,
        p.gx_dps_x1000,
        p.gy_dps_x1000,
        p.gz_dps_x1000,
        p.imu_flags,
        p.qw_x10000,
        p.qx_x10000,
        p.qy_x10000,
        p.qz_x10000,
        b"\x00" * 10,
    )
    return build_frame(node_id, PS_MSG_IMU_TELEM, payload)


def encode_cmd(node_id: int, h: CmdHeader) -> bytes:
    hdr = struct.pack("<HBBH", h.cmd_id, h.cmd_class, h.cmd_code, len(h.params))
    payload = hdr + (h.params or b"")
    return build_frame(node_id, PS_MSG_CMD, payload)


# -------------------------
# Bit packing helpers
# -------------------------

def bit_get(arr: bytes, i: int) -> int:
    return (arr[i // 8] >> (7 - (i % 8))) & 1


def bit_set(arr: bytearray, i: int) -> None:
    arr[i // 8] |= (0x80 >> (i % 8))


def pack_bits(bit_list: List[int]) -> bytes:
    n_bytes = (len(bit_list) + 7) // 8
    buf = bytearray(n_bytes)
    for i, b in enumerate(bit_list):
        if b:
            bit_set(buf, i)
    return bytes(buf)


def unpack_bits(data: bytes, count: int) -> List[int]:
    return [bit_get(data, i) for i in range(count)]


# -------------------------
# QKD encode helpers
# -------------------------

def encode_qkd_initiate(node_id: int, raw_bits: bytes, gs_bases: bytes,
                        num_bits: int = 100) -> bytes:
    payload = raw_bits[:13] + gs_bases[:13] + struct.pack("B", num_bits)
    return build_frame(node_id, PS_MSG_QKD_INITIATE, payload)


def encode_qkd_sifted(node_id: int, match_mask: bytes, check_mask: bytes,
                       check_bits: bytes, num_bits: int = 100) -> bytes:
    payload = (match_mask[:13] + check_mask[:13] + check_bits[:13] +
               struct.pack("B", num_bits))
    return build_frame(node_id, PS_MSG_QKD_SIFTED, payload)


# -------------------------
# Decode helpers
# -------------------------

def _decode_env(payload: bytes) -> EnvTelem:
    if len(payload) < 24:  # 4+4+2+4+2+2+6 = 24
        raise ValueError("ENV_TELEM too short")
    uptime_ms, sample_id, temp, press, hum, flags, _ = struct.unpack("<IIhIHH6s", payload[:24])  # now struct size is 24 bytes
    return EnvTelem(uptime_ms, sample_id, temp, press, hum, flags)



def _decode_imu(payload: bytes) -> ImuTelem:
    if len(payload) < 40:
        raise ValueError("IMU_TELEM too short")
    # 4+4 + 6*2 + 2 + 4*2 + 10 = 40
    vals = struct.unpack("<IIhhhhhhHhhhh10s", payload[:40])
    return ImuTelem(
        uptime_ms=vals[0],
        sample_id=vals[1],
        ax_mps2_x1000=vals[2],
        ay_mps2_x1000=vals[3],
        az_mps2_x1000=vals[4],
        gx_dps_x1000=vals[5],
        gy_dps_x1000=vals[6],
        gz_dps_x1000=vals[7],
        imu_flags=vals[8],
        qw_x10000=vals[9],
        qx_x10000=vals[10],
        qy_x10000=vals[11],
        qz_x10000=vals[12],
    )


def _decode_gps(payload: bytes) -> GpsTelem:
    if len(payload) < 36:
        raise ValueError("GPS_TELEM too short")
    vals = struct.unpack("<IIiiiHHHBBH", payload[:36])
    return GpsTelem(*vals)


def _decode_status(payload: bytes) -> Status:
    if len(payload) < 24:
        raise ValueError("STATUS too short")
    vals = struct.unpack("<IHHIBBhhH2s", payload[:24])
    return Status(
        uptime_ms=vals[0],
        battery_mv=vals[1],
        battery_pct_x100=vals[2],
        storage_free_kb=vals[3],
        mission_state=vals[4],
        recovery_flags=vals[5],
        link_rssi_dbm=vals[6],
        link_snr_db_x10=vals[7],
        status_flags=vals[8],
    )


def _decode_beacon(payload: bytes) -> Beacon:
    if len(payload) < 24:
        raise ValueError("BEACON too short")
    uptime_ms, last_age, lat, lon, alt, batt, rflags, seq, _ = struct.unpack("<IHiiiHBB4s", payload[:24])
    return Beacon(uptime_ms, last_age, lat, lon, alt, batt, rflags, seq)


def _decode_cmd(payload: bytes) -> CmdHeader:
    if len(payload) < 6:
        raise ValueError("CMD too short")
    cmd_id, cmd_class, cmd_code, param_len = struct.unpack("<HBBH", payload[:6])
    params = payload[6:6 + param_len]
    return CmdHeader(cmd_id, cmd_class, cmd_code, param_len, params)


def _decode_cmd_ack(payload: bytes) -> CmdAck:
    if len(payload) < 8:
        raise ValueError("CMD_ACK too short")
    cmd_id, status, detail, ts = struct.unpack("<HBBI", payload[:8])
    return CmdAck(cmd_id, status, detail, ts)


def _decode_qkd_response(payload: bytes) -> QkdResponse:
    if len(payload) < 14:
        raise ValueError("QKD_RESPONSE too short")
    sat_bases = payload[:13]
    num_bits = payload[13]
    return QkdResponse(sat_bases=sat_bases, num_bits=num_bits)


def _decode_qkd_verify(payload: bytes) -> QkdVerify:
    # Packed struct: 1+1+1+32+1+1 = 37 bytes
    if len(payload) < 37:
        raise ValueError("QKD_VERIFY too short")
    error_count = payload[0]
    check_count = payload[1]
    key_len_bits = payload[2]
    encrypted_msg = payload[3:35]     # 32 bytes
    encrypted_len = payload[35]
    status = payload[36]
    return QkdVerify(
        error_count=error_count,
        check_count=check_count,
        key_len_bits=key_len_bits,
        encrypted_msg=bytes(encrypted_msg[:encrypted_len]),
        encrypted_len=encrypted_len,
        status=status,
    )


def decode_payload(msg_type: int, payload: bytes) -> Union[EnvTelem, ImuTelem, GpsTelem, Status, Beacon, CmdHeader, CmdAck, QkdResponse, QkdVerify, bytes]:
    if msg_type == PS_MSG_ENV_TELEM:
        return _decode_env(payload)
    if msg_type == PS_MSG_IMU_TELEM:
        return _decode_imu(payload)
    if msg_type == PS_MSG_GPS_TELEM:
        return _decode_gps(payload)
    if msg_type == PS_MSG_STATUS:
        return _decode_status(payload)
    if msg_type == PS_MSG_BEACON:
        return _decode_beacon(payload)
    if msg_type == PS_MSG_CMD:
        return _decode_cmd(payload)
    if msg_type == PS_MSG_CMD_ACK:
        return _decode_cmd_ack(payload)
    if msg_type == PS_MSG_QKD_RESPONSE:
        return _decode_qkd_response(payload)
    if msg_type == PS_MSG_QKD_VERIFY:
        return _decode_qkd_verify(payload)
    return payload  # raw bytes for unknown types


# -------------------------
# Streaming parser
# -------------------------

class FrameParser:
    def __init__(self):
        self._buf = bytearray()
        self._state = "sync"

    def feed(self, data: bytes) -> Iterator[Frame]:
        self._buf.extend(data)
        out: list[Frame] = []

        while True:
            # find sync
            if self._state == "sync":
                i = self._buf.find(bytes([PS_SYNC_LO, PS_SYNC_HI]))
                if i < 0:
                    self._buf.clear()
                    break
                if i > 0:
                    del self._buf[:i]
                if len(self._buf) < 2:
                    break
                # consume sync
                del self._buf[:2]
                self._state = "header"

            if self._state == "header":
                if len(self._buf) < 5:
                    break
                ver, msg_type, node_id, payload_len = struct.unpack("<BBBH", self._buf[:5])
                del self._buf[:5]
                if ver != PS_PROTO_VER:
                    # discard and resync
                    self._state = "sync"
                    continue
                self._current_header = (ver, msg_type, node_id, payload_len)
                self._state = "payload"

            if self._state == "payload":
                ver, msg_type, node_id, payload_len = self._current_header
                need = payload_len + 2  # payload + crc16
                if len(self._buf) < need:
                    break
                payload = bytes(self._buf[:payload_len])
                crc_recv = struct.unpack("<H", self._buf[payload_len:payload_len + 2])[0]
                del self._buf[:need]

                crc_calc = crc16_ccitt(struct.pack("<BBBH", ver, msg_type, node_id, payload_len) + payload)
                if crc_calc != crc_recv:
                    # CRC fail: resync
                    self._state = "sync"
                    continue

                parsed = decode_payload(msg_type, payload)
                out.append(Frame(ver=ver, msg_type=msg_type, node_id=node_id, payload=parsed))
                self._state = "sync"

        return iter(out)
