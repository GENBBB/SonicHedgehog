import math

class LidarData:
    def __init__(self, FSA, LSA, CS, Speed, TimeStamp,
                 Degree_angle, Confidence_i, Angle_i, Distance_i):
        self.FSA = FSA
        self.LSA = LSA
        self.CS = CS
        self.Speed = Speed
        self.TimeStamp = TimeStamp
        self.Degree_angle = Degree_angle
        self.Confidence_i = Confidence_i
        self.Angle_i = Angle_i
        self.Distance_i = Distance_i


def CalcLidarData(data_str):
    data_str = data_str.replace(' ', '')

    Speed = int(data_str[2:4] + data_str[0:2], 16) / 100
    FSA = float(int(data_str[6:8] + data_str[4:6], 16)) / 100
    LSA = float(int(data_str[-8:-6] + data_str[-10:-8], 16)) / 100
    TimeStamp = int(data_str[-4:-2] + data_str[-6:-4], 16)
    CS = int(data_str[-2:], 16)

    Confidence_i, Angle_i, Distance_i, Degree_angle = [], [], [], []

    # шаг между точками
    if LSA - FSA > 0:
        angle_step = (LSA - FSA) / 12
    else:
        angle_step = ((LSA + 360) - FSA) / 12

    circle = lambda deg: deg - 360 if deg >= 360 else deg

    for i in range(0, 6 * 12, 6):
        dist = int(data_str[8 + i + 2:8 + i + 4] + data_str[8 + i:8 + i + 2], 16) / 1000
        conf = int(data_str[8 + i + 4:8 + i + 6], 16)
        angle_deg = circle(angle_step * (i // 6) + FSA)
        Distance_i.append(dist)
        Confidence_i.append(conf)
        Degree_angle.append(angle_deg)
        Angle_i.append(angle_deg * math.pi / 180.0)

    return LidarData(FSA, LSA, CS, Speed, TimeStamp,
                     Degree_angle, Confidence_i, Angle_i, Distance_i)
