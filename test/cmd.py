import serial
import json
import math
import time
import threading
import sys
import signal
from time import sleep

# === ПАРАМЕТРЫ РОБОТА ===
PORT = '/dev/ttyACM1'
BAUD = 115200
WHEEL_RADIUS = 0.03725      # радиус колеса, м
WHEEL_BASE = 0.154        # база между колёсами, м
UPDATE_RATE = 0.1        # период обновления, с
SLEEP_TIME = 0.5

# === ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ===
odom = {"x": 0.0, "y": 0.0, "theta": 0.0}
last_odl = None
last_odr = None
lock = threading.Lock()
running = True

# === СОЕДИНЕНИЕ ===
try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
except Exception as e:
    print(e)
    ser = None

def send_command(cmd_dict):
    msg = json.dumps(cmd_dict) + "\r\n"
    print(f"|{msg}|")
    ser.write(msg.encode('utf-8'))
    sleep(SLEEP_TIME)

def set_velocity(v, omega):
    """Отправить команду движения с заданными линейной и угловой скоростями"""
    vL = v - (WHEEL_BASE / 2.0) * omega
    vR = v + (WHEEL_BASE / 2.0) * omega

    rpmL = vL / (2 * math.pi * WHEEL_RADIUS) * 60
    rpmR = vR / (2 * math.pi * WHEEL_RADIUS) * 60

    L = int(10 * rpmL)
    R = int(10 * rpmR)

    send_command({"T": 1, "L": L, "R": R})

def get_feedback():
    """Запросить одометрию и состояние"""
    send_command({"T": 130})
    line = ser.readline().decode('utf-8', errors='ignore').strip()
    if line:
        try:
            data = json.loads(line)
            if data.get("T") == 1001:
                return {
                    "odl": float(data.get("odl", 0.0)),   # см
                    "odr": float(data.get("odr", 0.0)),   # см
                    "v_bat": data.get("v", 0)/100.0
                }
        except json.JSONDecodeError:
            pass
    return None

def update_odometry(odl_cm, odr_cm):
    """Обновить положение робота (x, y, theta) по одометрии"""
    global last_odl, last_odr, odom

    odl = odl_cm / 100.0  # перевод в метры
    odr = odr_cm / 100.0

    if last_odl is None or last_odr is None:
        last_odl, last_odr = odl, odr
        return

    dL = odl - last_odl
    dR = odr - last_odr
    last_odl, last_odr = odl, odr

    dC = (dL + dR) / 2.0
    dTheta = (dR - dL) / WHEEL_BASE

    with lock:
        odom["theta"] += dTheta
        odom["x"] += dC * math.cos(odom["theta"])
        odom["y"] += dC * math.sin(odom["theta"])

def feedback_loop():
    """Фоновый поток для чтения одометрии"""
    while running:
        fb = get_feedback()
        if fb:
            update_odometry(fb["odl"], fb["odr"])
        time.sleep(UPDATE_RATE)

def signal_handler(sig, frame):
    global running
    running = False
    set_velocity(0, 0)
    send_command({"T": 131, "cmd": 0})
    ser.close()
    print("\nЗавершено.")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# === ЗАПУСК ===
send_command({"T": 131, "cmd": 1})  # включить режим обратной связи
send_command({"Т": 132, "IO1": 255, "IO2": 255})
print("Робот запущен. Управление: w/s/a/d, Enter — стоп, Ctrl+C — выход.\n")

threading.Thread(target=feedback_loop, daemon=True).start()

# === РУЧНОЕ УПРАВЛЕНИЕ ===
try:
    while running:
        cmd = input().strip()

        match cmd:
            case "w":
                set_velocity(0.2, 0.0)
            case "s":
                set_velocity(-0.2, 0.0)
            case "a":
                set_velocity(0.0, 1.0)
            case "d":
                set_velocity(0.0, -1.0)
            case "":
                set_velocity(0.0, 0.0)
            case "odom":
                with lock:
                    print(f"x={odom['x']:.3f} m, y={odom['y']:.3f} m, θ={math.degrees(odom['theta']):.1f}°")
            case "help":
                print("Команды: w/s/a/d — движение, Enter — стоп, odom — вывести одометрию")
except KeyboardInterrupt:
    pass
finally:
    signal_handler(None, None)
