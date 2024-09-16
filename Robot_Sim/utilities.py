import numpy as np

def euler_to_quaternion(euler):
        yaw, pitch, roll = euler

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qw, qx, qy, qz]


def qproduct(a, b):
    c = [0, 0, 0, 0]
    c[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3]
    c[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2]
    c[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1]
    c[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]
    return c


def qconj(x):
    y = [0, 0, 0, 0]
    y[0] = x[0]
    y[1] = -x[1]
    y[2] = -x[2]
    y[3] = -x[3]
    return y


def semantically_similar(deltadrx, deltadry, tolerancea):

    # print('deltadrx')
    # print(deltadrx)
    # print('deltadry')
    # print(deltadry)


    i = 0
    j = 0
    deviation = 0
    similarity = 0 #added by me
    
    mappingpoint = np.zeros(len(deltadrx))
    
    while i < len(deltadrx):
        while j < len(deltadry):
            if i == 0:
                if min(np.linalg.norm(np.array(deltadrx[0]) - np.array(deltadry[0])), np.linalg.norm(np.array(deltadrx[0]) + np.array(deltadry[0]))) > tolerancea:
                    similarity = 0
                    mappingpoint[i] = 0
                    i = len(deltadrx)
                    break
                else:
                    mappingpoint[i] = 1
                    j += 1
                    break
            elif min(np.linalg.norm(np.array(deltadrx[i]) - np.array(deltadry[j])), np.linalg.norm(np.array(deltadrx[i]) + np.array(deltadry[j]))) > tolerancea:
                j += 1
                if j > len(deltadry):
                    similarity = 0
                    mappingpoint[i] = 0
                    i = len(deltadrx)
                    break
            else:
                mappingpoint[i] = j
                deviation += min(np.linalg.norm(np.array(deltadrx[i]) - np.array(deltadry[j])), np.linalg.norm(np.array(deltadrx[i]) + np.array(deltadry[j])))
                j += 1
                break
        
        if i == len(deltadrx)-1:
            similarity = 1
        
        i += 1
    
    return similarity, mappingpoint, deviation



