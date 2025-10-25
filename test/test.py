# diagnostic_cobraflex.py
import serial, json, time
from time import sleep

PORT = '/dev/ttyACM1'
BAUD = 115200
ser = serial.Serial(PORT, BAUD, timeout=0.1)

def send_raw(s):
    ser.write(s.encode('utf-8'))

def send_command(cmd_dict):
    msg = json.dumps(cmd_dict) + "\r\n"
    msg = msg.replace(" ", "")
    print("SEND:", msg)
    # очистим входной буфер чтобы не читать старые ответы
    try:
        ser.reset_input_buffer()
    except:
        pass
    send_raw(msg)
    # короткая пауза чтобы устройство успело ответить
    time.sleep(0.05)

def read_jsons(timeout=1.0):
    """
    Читает из серила до timeout секунд, собирая все полные JSON объекты.
    Возвращает список распарсенных объектов и остаточный буфер (строка).
    """
    deadline = time.time() + timeout
    buf = ""
    results = []
    while time.time() < deadline:
        n = ser.in_waiting
        if n:
            chunk = ser.read(n).decode('utf-8', errors='replace')
            # покажем сырые байты для диагностики
            print("RAW_CHUNK:", repr(chunk))
            buf += chunk
            # извлекаем полные JSON-объекты по сбалансированным фигурным скобкам
            start = buf.find('{')
            while start != -1:
                depth = 0
                found = False
                for i in range(start, len(buf)):
                    if buf[i] == '{':
                        depth += 1
                    elif buf[i] == '}':
                        depth -= 1
                    if depth == 0:
                        candidate = buf[start:i+1]
                        try:
                            obj = json.loads(candidate)
                            results.append(obj)
                            print("PARSED_JSON:", obj)
                        except Exception as e:
                            print("JSON parse error for candidate:", repr(candidate), "err:", e)
                        buf = buf[i+1:]
                        found = True
                        break
                if not found:
                    break
                start = buf.find('{')
        else:
            time.sleep(0.02)
    return results, buf

# --- последовательность команд для теста ---
try:
    # 1) включаем continuous feedback / режим
    send_command({"T":131,"cmd":1})
    # прочитать ответы 1-2 сек
    read_jsons(1.5)
    sleep(0.5)

    # 2) включаем IO (LED / возможно питание моторной части)
    send_command({"T":132,"IO1":255,"IO2":255})
    read_jsons(1.5)
    sleep(0.5)

    # 3) запросить один feedback (T=130) чтобы увидеть текущее состояние
    send_command({"T":130})
    read_jsons(0.8)
    sleep(0.3)

    # 4) попытка запустить один мотор (M2) — чтобы проверить локально
    print("\n--- Test: single motor M2 -> 500 ---")
    send_command({"T":11,"M1":0,"M2":500,"M3":0,"M4":0})
    read_jsons(1.5)
    sleep(0.5)

    # 5) попытка запустить все моторы индивидуально
    print("\n--- Test: all M* -> 500 ---")
    send_command({"T":11,"M1":500,"M2":500,"M3":500,"M4":500})
    read_jsons(1.5)
    sleep(0.5)

    # 6) попытка общей скорости T=1 (L/R)
    print("\n--- Test: T=1 L/R -> 500 ---")
    send_command({"T":1,"L":500,"R":500})
    read_jsons(1.5)
    sleep(1.5)

    # 7) стоп
    send_command({"T":1,"L":0,"R":0})
    read_jsons(0.8)

    # финальный запрос статуса
    send_command({"T":130})
    read_jsons(1.0)

except KeyboardInterrupt:
    print("Interrupted")
finally:
    ser.close()
