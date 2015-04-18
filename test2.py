import serial
import landmarks

from landmarks import ransac_algorithm, add_to_DB, add_slam_id, remove_bad_landmarks, landmarkDB, DBSize
# out - range to object, x, y, theta
# catch out, push it to laserdata and robot_position

laserdata = []
pobot_position = []

def main():
    a = ser.readline()
    laser, x, y, theta = str(a)[2:-5].split(sep=" ")
    laserdata.append(laser)
    robot_position = [x, y, theta]
    temp = ransac_algorithm(laserdata)
    new_landmark = True

    for i in temp:
        for k in range(DBSize):
            if get_association(i, landmarkDB[k]) != -1:
                new_landmark = False
        if new_landmark:
            add_to_DB(i)
            add_slam_id(landmarkDB[DBSize], landmarkDB[DBSize]*5)       # FIXME why landmarkDB[DBSize]*5? because i want
    remove_bad_landmarks(laserdata,robot_position)                      # NB! find function for slam_id


if __name__ == "__main__":
    ser = serial.Serial('/dev/ttyACM0', 9600)
    while True:
        main()
        for i in range(DBSize):                                        # FIXME
            print(landmarkDB[i].pos, end=' ')                          # just test
""" That's all.
    One question left: so what? we have database with landmarks and so what?
    FIXME
    FIXME
    FIXME
"""
