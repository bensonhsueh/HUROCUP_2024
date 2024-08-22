import numpy as np
import math
def relative_motion(pos,next_pos):

    pose_x = (next_pos[:,0]-pos[:,0])*np.cos(next_pos[:,2]-pos[:,2]) + (next_pos[:,1]-pos[:,1])*np.sin(next_pos[:,2]-pos[:,2])
    pose_y = (next_pos[:,1]-pos[:,1])*np.cos(next_pos[:,2]-pos[:,2]) - (next_pos[:,0]-pos[:,0])*np.sin(next_pos[:,2]-pos[:,2])
    rot = next_pos[:,2] - pos[:,2]



    # print(pose_x,pose_y,rot)

    pose_x = pose_x.reshape(-1,1)
    pose_y= pose_y.reshape(-1,1)
    rot = rot.reshape(-1,1)

    pose = np.concatenate((pose_x,pose_y,rot),axis=1)

    # print(pose)

    return pose




def move(pos,pose):

    next_pos_x = pose[:,0]*np.cos(pose[:,2]) - pose[:,1]*np.sin(pose[:,2]) + pos[:,0]
    next_pos_y = pose[:,1]*np.cos(pose[:,2]) + pose[:,0]*np.sin(pose[:,2]) + pos[:,1]
    next_orientation = pose[:,2] + pos[:,2]



    # print(pose_x,pose_y,rot)
    next_pos_x = next_pos_x.reshape(-1,1)
    next_pos_y = next_pos_y.reshape(-1,1)
    next_orientation = next_orientation.reshape(-1,1)

    next_pos = np.concatenate((next_pos_x,next_pos_y,next_orientation),axis=1)

    # print(pose)

    return next_pos



def relative_motion_single(pos,next_pos):

    pose_x = (next_pos[0]-pos[0])*np.cos(next_pos[2]-pos[2]) + (next_pos[1]-pos[1])*np.sin(next_pos[2]-pos[2])
    pose_y = (next_pos[1]-pos[1])*np.cos(next_pos[2]-pos[2]) - (next_pos[0]-pos[0])*np.sin(next_pos[2]-pos[2])
    rot = next_pos[2] - pos[2]



    pose = np.array([pose_x,pose_y,rot])


    return pose


def move_single(pos,pose):

    radian = pose[2]
    # print(radian)
    next_pos_x = pose[0]*np.cos(radian) - pose[1]*np.sin(radian) + pos[0]
    next_pos_y = pose[1]*np.cos(radian) + pose[0]*np.sin(radian) + pos[1]
    next_orientation = radian + pos[2]


    next_pos = np.array([next_pos_x,next_pos_y,next_orientation])


    return next_pos


if __name__ == "__main__":


    import matplotlib.pyplot as plt

    batch_size = 1


    pos = np.random.rand(batch_size,3)

    next_pos = np.random.rand(batch_size,3)



    pos = np.array([[1,2,0.5*math.pi]])

    next_pos = np.array([[1,3,0.5*math.pi]])

    pose = relative_motion(pos,next_pos)

    orientation_degrees = pos[0][2]



    next_pos_pred = move(pos,pose)







    dx,dy = np.cos(pos[0][2])*0.01,np.sin(pos[0][2])*0.01

    fig = plt.figure()

    ax = fig.add_subplot(1,1,1)

    ax.plot(pos[0][0],pos[0][1],"o",markerfacecolor='blue')

    ax.arrow(pos[0][0],pos[0][1],dx,dy,ec='blue')

    dx2,dy2 = np.cos(next_pos[0][2])*0.01,np.sin(next_pos[0][2])*0.01

    ax.plot(next_pos[0][0],next_pos[0][1],"o",markerfacecolor='red')

    ax.arrow(next_pos[0][0],next_pos[0][1],dx2,dy2,ec='red')


    dx3,dy3 = np.cos(next_pos_pred[0][2])*0.01,np.sin(next_pos_pred[0][2])*0.01

    ax.plot(next_pos_pred[0][0],next_pos_pred[0][1],"o",markerfacecolor='green')

    ax.arrow(next_pos_pred[0][0],next_pos_pred[0][1],dx3,dy3,ec='green')

    # print(next_pos)
    # print(next_pos_pred)


    plt.show()
    plt.close()




