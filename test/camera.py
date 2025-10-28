import cv2
from time import sleep

def find_video_device():
    for i in range(5):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            print(f"Камера найдена: /dev/video{i}")
            return cap
        cap.release()
    print("Камера не найдена.")
    return None

cap = find_video_device()
if not cap:
    exit()

while True:
    ret, frame = cap.read()
    if ret:
        print("Кадр получен, сохраняю в snapshot.jpg")
        cv2.imwrite("snapshot.jpg", frame)
    else:
        print("Not удалось получить кадр.")
    sleep(0.1)

cap.release()