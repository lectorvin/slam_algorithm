import numpy as np


def mean(arr):
    return sum(arr)/(len(arr))


def mul(arr1, arr2):
    a = [arr1[i]*arr2[i] for i in range(len(arr1))]
    return a


def cov(arr1, arr2):
    m = len(mul(arr1, arr2))
    return (sum(mul(arr1, arr2)) - sum(arr1)*sum(arr2)/m)/(m-1)


def covMatrix(x, y):
    # column - variable, row - observation
    try:
        temp = [[0 for j in range(len(y[0]))] for i in range(len(x[0]))]
        for i in range(len(x[0])):
            for j in range(len(y[0])):
                temp[i][j] = round(cov([k[i] for k in x], [t[j] for t in y]),
                                   6)
    except TypeError:
        temp = [[0, 0], [0, 0]]
        temp[0][0] = cov(x, x)
        temp[0][1] = temp[1][0] = cov(x, y)
        temp[1][1] = cov(y, y)
    return temp


if __name__ == "__main__":
    x = [[-2.1, -1, 4.3]]
    y = [[3, 1.1, 0.12]]
    X = [[-2.1, -1, 4.3],
         [3, 1.1, 0.12]]
    XT = [[-2.1, 3],
          [-1, 1.1],
          [4.3, 0.12]]
    M = [[0.4706, 0.0588, 0.0882, 0.3824],
         [0.1471, 0.3235, 0.2941, 0.2353],
         [0.2647, 0.2059, 0.1765, 0.3529],
         [0.1176, 0.4118, 0.4412, 0.0294]]
    B = [[10, 5, 15],
         [15, 10, 16],
         [14, 5, 5],
         [13, 3, 2],
         [5, 10, -5],
         [-10, -5, -15],
         [-5, -10, 15],
         [3, 5, 20],
         [10, 8, 18],
         [15, 10, 15]]

    P = covMatrix(M, M)
    for l in P:
        for x in l:
            print(x, end=" ")
        print()
    print()

    M = np.matrix(M)
    P = np.cov(M, rowvar=0, ddof=1)
    for l in P:
        for x in l:
            print(round(x, 6), end=' ')
        print()
    print()

    U = covMatrix(B, B)
    for l in U:
        for x in l:
            print(x, end=' ')
        print()
    print()

    B = np.matrix(B)
    U = np.cov(B, rowvar=0, ddof=1)
    for l in U:
        for x in l:
            print(round(x, 6), end=' ')
        print()
    print()

    U = covMatrix(XT, XT)
    for l in U:
        for x in l:
            print(round(x, 6), end=' ')
        print()
