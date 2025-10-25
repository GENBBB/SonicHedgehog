import serial
import threading

ser = serial.Serial(port='/dev/ttyACM1',baudrate=115200)

def read_loop():
    while True:
        print(ser.readline())


threading.Thread(target=read_loop).start()


while True:
    cmd = input()
    match cmd:
        case "w":
            ser.write(b'{"T":1,"L":100,"R":100}\r\n')
        case "s":
            ser.write(b'{"T":1,"L":-100,"R":-100}\r\n')
        case "a":
            ser.write(b'{"T":1,"L":-100,"R":100}\r\n')
        case "d":
            ser.write(b'{"T":1,"L":100,"R":-100}\r\n')
        case "":
            ser.write(b'{"T":1,"L":0,"R":0}\r\n')
        case "start":
            ser.write(b'{"T":131,"cmd":1}\r\n')
        case "stop":
            ser.write(b'{"T":131,"cmd":0}\r\n')
