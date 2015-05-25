import serial   # catch data
import time     # small delay
import tkinter as tk    # simple
import landmarks, kalman_filter  # correct x,y

from PIL import Image, ImageDraw, ImageTk
from modules.landmarks import ransac_algorithm, add_to_DB, add_slam_id, \
                      remove_bad_landmarks, landmarkDB, DBSize
from modules.kalman_filter import main as kalman_main
# out:
# r1, r2, theta_i, range_i
# catch out, push it to laserdata and robot_position


# angles = [-90, -45, 0, 45, 90]
laserdata = []
pobot_position = []
x = y = theta = 0
l1 = l2 = 0
r1 = r2 = 0
l = 0.132   # m, FIXME
r = 0.035   # m, FIXME
color = "Red"
dot = [0, 0]


def tol(r1, r2):
    s1 = 2 * np.pi * r * r1 / 40
    s2 = 2 * np.pi * r * r2 / 40
    return s1, s2


def toxy(l1, l2):
    a = (l1-l2) / l
    dy = np.sin(a) * (l/2+r)
    dx = (np.cos(a)-np.sin(a)) * (l/2+r) / np.tan(a)
    return dx, dy


def get_coords():
    # get data
    a = ser.readline()
    try:
        r1, r2, theta, laser = str(a)[2:-5].split(' ')
    except ValueError:
        return False
    if laser>200:
        laser = float(inf)
    l1, l2 = tol(r1, r2)
    dx, dy = toxy(l1, l2)
    theta = (theta/180) * np.pi  # to radians
    laserdata.append(float(laser))
    robot_position = [float(dx), float(dy), float(theta)]

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


def image_():
    r_ = 10
    x, y = dot
    draw.circle((x-r_, y-r_, x+r_, y+r_), fill=color)


def show_():
    image_()
    photo = ImageTk.PhotoImage(im)
    label['image'] = photo
    label.image = photo


def main():
    get_coords()
    if laserdata[-1] < 201:
        dot[0] = laserdata[-1] * cos(theta)
        dot[1] = -laserdata[-1] * sin(theta)
    show_(dot)
    label.after(1, main)


if __name__ == "__main__":
    ports = ['dev/ttyACM0', 
             'dev/ttyACM1',
             'dev/ttyACM2',
             'dev/ttyACM3',
             'dev/ttyACM4',
             'dev/ttyACM5',
             'dev/ttyACM6',
             'dev/ttyACM7',
             'dev/ttyACM8']
    for port in ports:
        try:
            ser = serial.Serial(port, 9600)
        except Exception:
            pass
        else:
            break

    time.sleep(1)
    try:
        a = ser.readline()
    except:
        raise IOError    # FIXME

        
    root = tk.Tk()
    root.geometry('+300+200')
    im = Image.new("RGBA", (WIDTH+1, HEIGHT+1), "LightGray")
    photo = ImageTk.PhotoImage(im)
    draw = ImageDraw.Draw(im)
    label = tk.Label()
    label['image'] = photo
    label.image = photo
    label.pack()

    label.after_idle(main)
    root.mainloop()
