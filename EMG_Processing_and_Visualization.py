import serial, time, csv, sys, struct
import numpy as np
from collections import deque
from scipy.signal import butter, lfilter, lfilter_zi
import os
from datetime import datetime
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore

# ================= CONFIG =================
PORT = "COM11"
BAUD = 2000000
FS = 2000
DURATION = 110

LOWCUT = 10
HIGHCUT = 500
ENV_CUTOFF = 6

ENV_SEND_RATE = 100
ENV_INTERVAL = 1 / ENV_SEND_RATE

SAVE_DIR = r'C:\Alief\College\SKRIPSI\Program\New EMG\Data'
os.makedirs(SAVE_DIR, exist_ok=True)

filename = f"EMG_DATA_NAMA{datetime.now().strftime('%Y_%m_%d__%H_%M_%S')}.csv"
filepath = os.path.join(SAVE_DIR, filename)

WINDOW_SEC = 3
BUFFER_LEN = int(FS * WINDOW_SEC)

# ================= SERIAL =================
ser = serial.Serial(PORT, BAUD, timeout=0.001)
time.sleep(2)
ser.reset_input_buffer()
print("Serial connected")

# ================= FILTER =================
nyq = FS / 2
b_bp, a_bp = butter(4, [LOWCUT/nyq, HIGHCUT/nyq], btype='band')
b_env, a_env = butter(4, ENV_CUTOFF/nyq, btype='low')

zi_bp = lfilter_zi(b_bp, a_bp) * 0
zi_env = lfilter_zi(b_env, a_env) * 0

# ================= BUFFER =================
env_buffer = deque([0]*BUFFER_LEN, maxlen=BUFFER_LEN)
enc_buffer = deque([0]*BUFFER_LEN, maxlen=BUFFER_LEN)

data = []
encoder_angle = 0.0

start_time = time.time()
last_env_time = time.time()
last_print_time = time.time()

# ================= UI =================
app = QtWidgets.QApplication(sys.argv)

win = pg.GraphicsLayoutWidget(title="Realtime EMG Envelope & Encoder ")
win.resize(1000, 600)
win.show()

p1 = win.addPlot(title="EMG Envelope")
p1.showGrid(x=True, y=True)
curve_env = p1.plot(pen=pg.mkPen('y', width=2))

win.nextRow()
p2 = win.addPlot(title="Encoder Angle")
p2.showGrid(x=True, y=True)
curve_enc = p2.plot(pen=pg.mkPen('c', width=2))

# ================= UPDATE LOOP =================
def update():
    global zi_bp, zi_env, encoder_angle
    global last_env_time, last_print_time

    while ser.in_waiting >= 1:

        header = ser.read(1)

        # ===== EMG RAW =====
        if header == b'\xAA' and ser.in_waiting >= 2:

            packet = ser.read(2)
            emg_raw = struct.unpack('<H', packet)[0]
            t = time.time() - start_time

            bp, zi_bp = lfilter(b_bp, a_bp, [emg_raw], zi=zi_bp)
            rect = abs(bp[0])
            env, zi_env = lfilter(b_env, a_env, [rect], zi=zi_env)
            env_val = env[0]

            env_buffer.append(env_val)
            enc_buffer.append(encoder_angle)

            data.append([t, emg_raw, bp[0], env_val, encoder_angle])

            # ===== SEND ENVELOPE 100 Hz =====
            if time.time() - last_env_time >= ENV_INTERVAL:

            #   env_norm = min(env_val / 300.0, 1.0)
                env_norm = min(env_val / 300.0, 1.0)
                packet = b'\xCC' + struct.pack('<f', env_norm)
                ser.write(packet)

                last_env_time = time.time()

        # ===== ENCODER =====
        elif header == b'\xBB' and ser.in_waiting >= 4:

            packet = ser.read(4)
            encoder_angle = struct.unpack('<f', packet)[0] / 20

        else:
            pass

    # ===== UPDATE PLOT =====
    curve_env.setData(env_buffer)
    curve_enc.setData(enc_buffer)

    # ===== PRINT DETIK (OVERWRITE) =====
    current_time = time.time()
    if current_time - last_print_time >= 1.0:
        elapsed_sec = int(current_time - start_time)
        print(f"\rElapsed time: {elapsed_sec} s", end="", flush=True)
        last_print_time = current_time

    # ===== STOP CONDITION =====
    if current_time - start_time >= DURATION:

        timer.stop()
        ser.close()

        with open(filepath, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["time","emg_raw","emg_bandpass",
                             "emg_envelope","encoder_angle_deg"])
            writer.writerows(data)

        print("\nFinished:", data[-1][0], "seconds")
        print("CSV saved at:", filepath)

        QtWidgets.QApplication.quit()

# ================= TIMER =================
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(20)

sys.exit(app.exec())
