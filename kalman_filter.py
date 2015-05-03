import numpy as np
landmarks = landmarkDB[:DBSize]    # FIXME: DBSize, landmarkDB

C = 1    # const for gaussin sample
c = 0.01
d = 1
V = [[1,1],   # V is just 2 by 2 identity matrix
     [1,1]]
A = [[1,0,0],   # jacobian of the prediction model
     [0,1,0],
     [0,0,1]]
H = [[0,0,0],   # jacobian of the measurement model
     [0,0,0]]
X = np.matrix([[0],   # system state
               [0],
               [0]])
P = np.matrix([[0,0,0],   # default - np.cov(x[:3],x[:3])
               [0,0,0],   # covariance matrix
               [0,0,0]])


def main(dx,dy,dth,DBSize,landmarkDB):
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
    x += dx
    y += dy
    th += dth
    X[0], X[1], X[2] = x, y, th

    A[0][2] = -dy
    A[1][2] = dx
    Q = [[C*dt**2, C*dx*dy, C*dx*dt],
         [C*C*dy*dx, C*dy**2, C*dy*dt],
         [C*dt*dx, C*dt*dy, C*dt**2]]
    Prr = A*Prr*A + Q   # top left 3*3 of P
    Pri = A*Pri    # top 3 rows of X

    for i,landmark in enumerate(landmarks):
        lx, ly = landmark.pos
        lrange = landmark.range_
        lbearing = landmark.bearing

        range_ = math.sqrt((lx-x)**2 + (ly-y)**2) + lrange
        bearing = math.atan((ly-y) / (lx-x)) - th + lbearing

        H[0][0] = (x-lx) / range_
        H[0][1] = (y-ly) / range_
        H[1][0] = (y-ly) / bearing**2
        H[1][1] = (x-lx) / bearing**2
        try:
            H[1+i*2] = -H[0][0]     # 3+i*2-1 -1 
            H[2+i*2] = -H[0][1]     # 3+i*2   -1
            H[1+i*2] = -H[1][0]     # check it
            H[2+i*2] = -H[1][1]
        except IndexError:
            temp = np.matrix([[-H[0][0],-H[0][1]],[-H[1][0],-H[1][1]]])
            np.append(H,temp)    # check it

    R[0][0] *= c
    R[1][1] *= d
    # r = R[0][0]
    # b = R[1][1]
    # R = [[r*c, 0],[0, b*d]]

    K = P * H.T * (H*P*H.T + V*R*V.T)**(-1)
    S = H*P*H.T + V*R*V.T
