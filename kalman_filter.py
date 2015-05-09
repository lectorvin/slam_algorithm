import numpy as np
# import cov_matrix
from landmarks import get_DB, get_DB_size

DBSize = get_DB_size()
landmarkDB = get_DB()

landmarks = landmarkDB[:DBSize]

C = 1.0    # const for gaussin sample
c = 0.01
d = 1.0
V = np.matrix([[1, 1],   # V is just 2 by 2 identity matrix
               [1, 1]],
              dtype='float')
A = np.matrix([[1, 0, 0],   # jacobian of the prediction model
               [0, 1, 0],
               [0, 0, 1]],
              dtype='float')
H = np.matrix([[0, 0, 0],   # jacobian of the measurement model
               [0, 0, 0]],
              dtype='float')
X = np.matrix([[0],   # system state
               [0],
               [0]],
              dtype='float')
P = np.matrix([[0, 0, 0],   # default - np.cov(x[:3],x[:3])
               [0, 0, 0],   # covariance matrix
               [0, 0, 0]],
              dtype='float')
R = np.matrix([[1, 0],
               [0, 1]],
              dtype='float')
Jxr = np.matrix([[1, 0, 0],
                 [0, 1, 0]],
                dtype='float')

Jz = np.matrix([[0, 0],
                [0, 0]],
               dtype='float')

x = y = th = 0


def main(robot_position, dposition, landmarkDB, DBSize):
    # A = [[1,0,-dy],[0,1,dx],[0,0,1]] - matrix
    #
    # Hus = [[(x-lx)/r,(y-ly)/r,0],[(ly-y)/r**2,(lx-x)/r**2,-1]]
    # H - matrix
    # ---------------------------
    # xr|yr|tr|x1|y1|x2|y2|x3|y3|
    # ---------------------------
    # A |B |C |  |  |-A|-B|  |  |
    # D |E |F |  |  |-D|-E|  |  |
    # ---------------------------
    # A - (x-lx)/r, B - (y-ly)/r, D - (ly-y)/r**2, E - (lx-x)/r**2
    #
    # x, y, z

    global x, y, th, V, A, H, P, X, c, C, d, R, Jxr, Jz

    landmarkDB = landmarkDB[:DBSize]
    """ step 1 """
    dx = dposition[0]
    dy = dposition[1]
    dth = dposition[2]

    x = robot_position[0]
    y = robot_position[1]
    th = robot_position[2]

    x += dx
    y += dy
    th += dth
    X[0], X[1], X[2] = x, y, th

    A[0, 2] = -dy
    A[1, 2] = dx
    Q = [[C*dth**2, C*dx*dy, C*dx*dth],
         [C*C*dy*dx, C*dy**2, C*dy*dth],
         [C*dth*dx, C*dth*dy, C*dth**2]]

    Prr = P[:3, :3]
    Prr = A*Prr*A + Q   # top left 3*3 of P
    P[:3, :3] = Prr

    Pri = P[:3, :]
    Pri = A*Pri         # top 3 rows of X
    P[:3, :] = Pri

    Jxr[0, 2] = -dth * np.sin(th)
    Jxr[0, 2] = dth * np.cos(th)

    Jz = np.matrix([[np.cos(th), -dth*np.cos(th)],  # not sure, maybe th+dth
                    [np.sin(th), dth*np.cos(th)]])

    """ step 2 """
    for i, landmark in enumerate(landmarks):
        lx, ly = landmark.pos
        lrange = landmark.range_
        lbearing = landmark.bearing

        range_ = math.sqrt((lx-x)**2 + (ly-y)**2) + lrange
        bearing = math.atan((ly-y) / (lx-x)) - th + lbearing

        h = np.matrix([[range_], [bearing]])

        H[0][0] = (x-lx) / range_
        H[0][1] = (y-ly) / range_
        H[1][0] = (y-ly) / bearing**2
        H[1][1] = (x-lx) / bearing**2
        try:
            H[1 + i*2] = -H[0][0]     # 3+i*2-1 -1
            H[2 + i*2] = -H[0][1]     # 3+i*2   -1
            H[1 + i*2] = -H[1][0]     # check it
            H[2 + i*2] = -H[1][1]
        except IndexError:
            temp = np.matrix([[-H[0][0], -H[0][1]], [-H[1][0], -H[1][1]]])
            np.append(H, temp)

        r = R[0, 0]
        b = R[1, 1]
        R = [[r*c, 0], [0, b*d]]

        try:
            K = P * H.T * (H*P*H.T + V*R*V.T)**(-1)
            S = H*P*H.T + V*R*V.T
        except np.linalg.linalg.LinAlgError:
            print('Nothing to correct')
            K = 1

        X = X + K*(h)  # z-h

        """ step 3 """
        land_pos = np.matrix([[lx], [ly]], dtype='float')
        # first we add the new landmark to the state vector X
        X = np.append(X, land_pos, axis=0)
        # we add the covariance for the new landmark in the cell C
        # P^(N+1,N+1) = Jxr*P*Jxr.T + Jz*R*Jz.T
        mass = []    # add the row of covariance to the P
        cov_C = np.cov(land_pos, land_pos, rowvar=0)    # , ddof=0)
        for i, lm in enumerate(landmarkDB):
            if i != DBSize:
                lm_pos = np.matrix([[lm.pos[0]], [lm.pos[1]]], dtype='float')
                cov_F = np.cov(land_pos, lm_pos, rowvar=0)
                mass.append(cov_F)
        P = np.append(P, mass, axis=0)

        column = mass.T
        column = np.append(column, cov_C, axis=0)
        P = np.append(P, column, axis=0)

    return round(X[0, 0], 6), round(X[1, 0], 6), round(X[2, 0], 6)
    # round doesn't work

if __name__ == "__main__":
    from landmarks import DBSize, landmarkDB
    print(main([5, 5, 0.5], [1, 1, 0.1], landmarkDB, DBSize))
