import serial
from polysense_proto import FrameParser, PS_MSG_ENV_TELEM, PS_MSG_IMU_TELEM

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
parser = FrameParser()

while True:
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

