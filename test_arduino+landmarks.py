import serial
import time
import landmarks, kalman_filter

from landmarks import ransac_algorithm, add_to_DB, add_slam_id, remove_bad_landmarks, landmarkDB, DBSize
from kalman_filter import main as kalman_main
# out - range to object, x, y, theta
# catch out, push it to laserdata and robot_position

laserdata = []
pobot_position = []

def main():
    # get data
    a = ser.readline()
    try:
        laser, x, y, theta = str(a)[2:-5].split(' ')
    except ValueError:
        return False
    laserdata.append(float(laser))
    robot_position = [float(x), float(y), float(theta)]

    # get landmarks
    temp = ransac_algorithm(laserdata, robot_position)
    new_landmark = True
    global robot_position
    for i in temp:
        for k in range(DBSize):
            if get_association(i, landmarkDB[k]) != -1:
                new_landmark = False
        if new_landmark:
            add_to_DB(i)

    remove_bad_landmarks(laserdata,robot_position)

    # update robot_position

    x, y, theta = kalman_main(robot_position, landmarkDB, DBSize)


if __name__ == "__main__":
    ser = serial.Serial('/dev/ttyACM0', 9600)
    time.sleep(1)
    a = ser.readline()
    while True:
        main()
        if DBSize:
            for i in range(DBSize):                                 # FIXME
                print(landmarkDB[i].pos, end=' ')                   # just test
        else:
            global robot_position
            print(robot_position)
