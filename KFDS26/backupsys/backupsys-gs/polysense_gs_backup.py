#!/usr/bin/env python3
import sys
import csv
import time
from datetime import datetime

import serial
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg

from polysense_proto import (
    FrameParser,
    PS_MSG_ENV_TELEM,
    PS_MSG_IMU_TELEM,
)


class SerialThread(QtCore.QThread):
    frame_received = QtCore.pyqtSignal(object)

    def __init__(self, port, baud, parent=None):
        super().__init__(parent)
        self.port = port
        self.baud = baud
        self._stop = False

    def run(self):
        try:
            ser = serial.Serial(self.port, self.baud, timeout=1)
        except Exception as e:
            print(f"[Serial] open failed: {e}")
            return

        parser = FrameParser()
        print(f"[Serial] Listening on {self.port} @ {self.baud}")

        while not self._stop:
            try:
                data = ser.read(256)
                if not data:
                    continue
                for frame in parser.feed(data):
                    self.frame_received.emit(frame)
            except Exception as e:
                print(f"[Serial] error: {e}")
                time.sleep(0.5)

        ser.close()
        print("[Serial] stopped")

    def stop(self):
        self._stop = True


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, port="/dev/ttyUSB0", baud=115200):
        super().__init__()
        self.setWindowTitle("KFDS26 Node Interface")
        self.resize(1500, 900)

        self.port = port
        self.baud = baud

        # logging state
        self.csv_fh = None
        self.csv_w = None
        self.logging = False

        # data buffers
        self.max_points = 300

        self.env_index = 0
        self.imu_index = 0

        self.x_T = []
        self.T_vals = []
        self.x_P = []
        self.P_vals = []
        self.x_H = []
        self.H_vals = []

        self.x_IMU = []
        self.ax_vals = []
        self.ay_vals = []
        self.az_vals = []
        self.gx_vals = []
        self.gy_vals = []
        self.gz_vals = []

        self._setup_style()
        self._build_ui()

        # serial + timer
        self.serial_thread = SerialThread(port, baud)
        self.serial_thread.frame_received.connect(self.on_frame)
        self.serial_thread.start()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(120)  # ~8 Hz

    # ---------- Style ----------

    def _setup_style(self):
        app = QtWidgets.QApplication.instance()
        palette = QtGui.QPalette()
        palette.setColor(QtGui.QPalette.Window, QtGui.QColor(5, 5, 18))        # deep space
        palette.setColor(QtGui.QPalette.Base, QtGui.QColor(8, 8, 24))
        palette.setColor(QtGui.QPalette.AlternateBase, QtGui.QColor(14, 18, 40))
        palette.setColor(QtGui.QPalette.Text, QtGui.QColor(220, 240, 255))
        palette.setColor(QtGui.QPalette.WindowText, QtGui.QColor(220, 240, 255))
        palette.setColor(QtGui.QPalette.Button, QtGui.QColor(16, 20, 40))
        palette.setColor(QtGui.QPalette.ButtonText, QtGui.QColor(230, 240, 255))
        palette.setColor(QtGui.QPalette.Highlight, QtGui.QColor(0, 220, 200))  # neon cyan
        palette.setColor(QtGui.QPalette.HighlightedText, QtCore.Qt.black)
        app.setPalette(palette)

        app.setStyleSheet("""
            QMainWindow {
                background-color: #050512;
            }
            QLabel {
                color: #E0F4FF;
            }
            QPushButton {
                background-color: #141830;
                color: #E0F4FF;
                border: 1px solid #00D0C0;
                padding: 4px 10px;
                border-radius: 3px;
            }
            QPushButton:hover {
                background-color: #1C2140;
                border-color: #FF4FD8;
            }
            QPushButton:pressed {
                background-color: #101326;
            }
        """)

        pg.setConfigOption("background", (5, 5, 18))
        pg.setConfigOption("foreground", (210, 240, 255))

    def _style_axes(self, pw, color="#00D0C0"):
        ax_font = QtGui.QFont("IBM Plex Mono", 9)
        pw.getAxis("left").setStyle(tickFont=ax_font)
        pw.getAxis("bottom").setStyle(tickFont=ax_font)
        pw.getAxis("left").setPen(pg.mkPen(color, width=1))
        pw.getAxis("bottom").setPen(pg.mkPen(color, width=1))
        pw.getPlotItem().getViewBox().setBorder(pg.mkPen(color, width=1))

    # ---------- UI ----------

    def _build_ui(self):
        cw = QtWidgets.QWidget()
        self.setCentralWidget(cw)

        main_layout = QtWidgets.QVBoxLayout(cw)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(6)

        # Header
        header_layout = QtWidgets.QHBoxLayout()
        main_layout.addLayout(header_layout)

        title_label = QtWidgets.QLabel("KFDS26 NODE INTERFACE // BACKUP LINK")
        title_font = QtGui.QFont("IBM Plex Mono", 18, QtGui.QFont.Bold)
        title_label.setFont(title_font)

        self.label_status = QtWidgets.QLabel("Status: Connected")
        info_font = QtGui.QFont("IBM Plex Mono", 10)
        self.label_status.setFont(info_font)

        self.label_T = QtWidgets.QLabel("T: --.- C")
        self.label_P = QtWidgets.QLabel("P: ----- Pa")
        self.label_H = QtWidgets.QLabel("H: --.- %")
        self.label_T.setFont(info_font)
        self.label_P.setFont(info_font)
        self.label_H.setFont(info_font)

        header_layout.addWidget(title_label)
        header_layout.addStretch(1)
        header_layout.addWidget(self.label_status)
        header_layout.addSpacing(16)
        header_layout.addWidget(self.label_T)
        header_layout.addSpacing(8)
        header_layout.addWidget(self.label_P)
        header_layout.addSpacing(8)
        header_layout.addWidget(self.label_H)

        # Plots
        plots_layout = QtWidgets.QVBoxLayout()
        main_layout.addLayout(plots_layout, 1)

        # Temperature plot
        self.plot_T = pg.PlotWidget(title="TEMP CHANNEL 01  [°C]")
        self.plot_T.showGrid(x=True, y=True, alpha=0.25)
        self._style_axes(self.plot_T, "#00FFC8")
        self.curve_T = self.plot_T.plot(
            pen=pg.mkPen("#00FFC8", width=2.0, style=QtCore.Qt.SolidLine)
        )
        plots_layout.addWidget(self.plot_T, 1)

        # Pressure plot
        self.plot_P = pg.PlotWidget(title="PRESSURE CHANNEL 02  [Pa]")
        self.plot_P.showGrid(x=True, y=True, alpha=0.25)
        self._style_axes(self.plot_P, "#7CFF42")
        self.curve_P = self.plot_P.plot(
            pen=pg.mkPen("#7CFF42", width=1.8, style=QtCore.Qt.SolidLine)
        )
        plots_layout.addWidget(self.plot_P, 1)

        # Humidity plot
        self.plot_H = pg.PlotWidget(title="HUMIDITY CHANNEL 03  [%RH]")
        self.plot_H.showGrid(x=True, y=True, alpha=0.25)
        self._style_axes(self.plot_H, "#FF54F8")
        self.curve_H = self.plot_H.plot(
            pen=pg.mkPen("#FF54F8", width=1.8, style=QtCore.Qt.SolidLine)
        )
        plots_layout.addWidget(self.plot_H, 1)

        # IMU plot
        self.plot_IMU = pg.PlotWidget(
            title="IMU VECTOR SCOPE  [ACC: solid  /  GYRO: dotted]"
        )
        self.plot_IMU.showGrid(x=True, y=True, alpha=0.25)
        self._style_axes(self.plot_IMU, "#B0C8FF")
        # accel: solid RGB
        self.curve_ax = self.plot_IMU.plot(
            pen=pg.mkPen("#FF4F4F", width=1.4)
        )
        self.curve_ay = self.plot_IMU.plot(
            pen=pg.mkPen("#4FFF4F", width=1.4)
        )
        self.curve_az = self.plot_IMU.plot(
            pen=pg.mkPen("#4FC3FF", width=1.4)
        )
        # gyro: dotted pastel RGB
        self.curve_gx = self.plot_IMU.plot(
            pen=pg.mkPen("#FF9AD9", width=1.2, style=QtCore.Qt.DotLine)
        )
        self.curve_gy = self.plot_IMU.plot(
            pen=pg.mkPen("#AFFF9A", width=1.2, style=QtCore.Qt.DotLine)
        )
        self.curve_gz = self.plot_IMU.plot(
            pen=pg.mkPen("#9AD5FF", width=1.2, style=QtCore.Qt.DotLine)
        )
        plots_layout.addWidget(self.plot_IMU, 1)

        # Controls
        ctrl_layout = QtWidgets.QHBoxLayout()
        main_layout.addLayout(ctrl_layout)

        self.btn_log = QtWidgets.QPushButton("Start Log")
        self.btn_log.clicked.connect(self.toggle_logging)
        self.btn_quit = QtWidgets.QPushButton("Quit")
        self.btn_quit.clicked.connect(self.close)

        ctrl_layout.addWidget(self.btn_log)
        ctrl_layout.addStretch(1)
        ctrl_layout.addWidget(self.btn_quit)

    # ---------- Logging ----------

    def toggle_logging(self):
        if not self.logging:
            fname = datetime.utcnow().strftime("polysense_backup_%Y%m%dT%H%M%SZ.csv")
            try:
                self.csv_fh = open(fname, "w", newline="")
                self.csv_w = csv.writer(self.csv_fh)
                self.csv_w.writerow([
                    "ts_host", "node_id",
                    "T_C", "P_Pa", "H_rh",
                    "ax_mps2", "ay_mps2", "az_mps2",
                    "gx_dps", "gy_dps", "gz_dps",
                    "type",
                ])
                self.logging = True
                self.btn_log.setText("Stop Log")
                self.label_status.setText(f"Status: Logging to {fname}")
            except Exception as e:
                self.label_status.setText(f"Status: Log error: {e}")
        else:
            self.logging = False
            self.btn_log.setText("Start Log")
            if self.csv_fh:
                self.csv_fh.close()
                self.csv_fh = None
                self.csv_w = None
            self.label_status.setText("Status: Connected")

    def closeEvent(self, event):
        if self.serial_thread.isRunning():
            self.serial_thread.stop()
            self.serial_thread.wait(1000)
        if self.csv_fh:
            self.csv_fh.close()
        event.accept()

    # ---------- Frame handling ----------

    @QtCore.pyqtSlot(object)
    def on_frame(self, frame):
        ts = time.time()
        if frame.msg_type == PS_MSG_ENV_TELEM:
            p = frame.payload
            T = p.temperature_c_x100 / 100.0
            P = float(p.pressure_pa)
            H = p.humidity_rh_x100 / 100.0

            self.label_T.setText(f"T: {T:.2f} C")
            self.label_P.setText(f"P: {P:.1f} Pa")
            self.label_H.setText(f"H: {H:.1f} %")

            self.env_index += 1
            idx = self.env_index

            self.x_T.append(idx)
            self.T_vals.append(T)
            if len(self.x_T) > self.max_points:
                self.x_T = self.x_T[-self.max_points:]
                self.T_vals = self.T_vals[-self.max_points:]

            self.x_P.append(idx)
            self.P_vals.append(P)
            if len(self.x_P) > self.max_points:
                self.x_P = self.x_P[-self.max_points:]
                self.P_vals = self.P_vals[-self.max_points:]

            self.x_H.append(idx)
            self.H_vals.append(H)
            if len(self.x_H) > self.max_points:
                self.x_H = self.x_H[-self.max_points:]
                self.H_vals = self.H_vals[-self.max_points:]

            if self.logging and self.csv_w:
                self.csv_w.writerow([
                    ts, frame.node_id,
                    T, P, H,
                    "", "", "",
                    "", "", "",
                    "ENV",
                ])

        elif frame.msg_type == PS_MSG_IMU_TELEM:
            p = frame.payload
            ax = p.ax_mps2_x1000 / 1000.0
            ay = p.ay_mps2_x1000 / 1000.0
            az = p.az_mps2_x1000 / 1000.0
            gx = p.gx_dps_x1000 / 1000.0
            gy = p.gy_dps_x1000 / 1000.0
            gz = p.gz_dps_x1000 / 1000.0

            self.imu_index += 1
            idx = self.imu_index

            self.x_IMU.append(idx)
            self.ax_vals.append(ax)
            self.ay_vals.append(ay)
            self.az_vals.append(az)
            self.gx_vals.append(gx)
            self.gy_vals.append(gy)
            self.gz_vals.append(gz)

            if len(self.x_IMU) > self.max_points:
                self.x_IMU = self.x_IMU[-self.max_points:]
                self.ax_vals = self.ax_vals[-self.max_points:]
                self.ay_vals = self.ay_vals[-self.max_points:]
                self.az_vals = self.az_vals[-self.max_points:]
                self.gx_vals = self.gx_vals[-self.max_points:]
                self.gy_vals = self.gy_vals[-self.max_points:]
                self.gz_vals = self.gz_vals[-self.max_points:]

            if self.logging and self.csv_w:
                self.csv_w.writerow([
                    ts, frame.node_id,
                    "", "", "",
                    ax, ay, az,
                    gx, gy, gz,
                    "IMU",
                ])

    # ---------- Plot update ----------

    def update_plots(self):
        if self.x_T:
            self.curve_T.setData(self.x_T, self.T_vals)
        if self.x_P:
            self.curve_P.setData(self.x_P, self.P_vals)
        if self.x_H:
            self.curve_H.setData(self.x_H, self.H_vals)

        if self.x_IMU:
            x = self.x_IMU
            self.curve_ax.setData(x, self.ax_vals)
            self.curve_ay.setData(x, self.ay_vals)
            self.curve_az.setData(x, self.az_vals)
            self.curve_gx.setData(x, self.gx_vals)
            self.curve_gy.setData(x, self.gy_vals)
            self.curve_gz.setData(x, self.gz_vals)


def main():
    import argparse
    ap = argparse.ArgumentParser(description="KFDS26 Node Interface")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", default=115200, type=int)
    args = ap.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow(port=args.port, baud=args.baud)
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
