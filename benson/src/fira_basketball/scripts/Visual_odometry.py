import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import torch
from CNNLstmModel import DeepVO
import cv2
from utils import relative_motion_single,move_single
import numpy as np
import matplotlib.pyplot as plt

model = DeepVO()

model.load_state_dict(torch.load("CNNLstmModelNice.pt",map_location=torch.device('cpu')))

cap = cv2.VideoCapture(0)


id = 0

model.eval()
fig = plt.figure()

# pos_label = np.array([0,0,0.5*np.pi])
pos_pred = np.array([0,0,0.5*np.pi])
ax3 = fig.add_subplot(1,3,1)
ax1 = fig.add_subplot(1,3,2)
ax2 = fig.add_subplot(1,3,3)
x = []
y = []
with torch.no_grad():

    while True:


        id += 1

        _,img = cap.read()


        img = cv2.resize(img,(64,64))

        img = cv2.cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        original_img = img.copy()

        img = np.transpose(img, (2, 0, 1)).astype(np.float32)

        img = torch.from_numpy(img/255)

        img = img.view(1,3,64,64)

        if id<2:

            old_img = img

        else:
            # print(old_img.shape)
            # print(img.shape)

            pose = model(old_img,img)

            # print(pose)
            pose = pose.view(-1,3)

            pose = pose.numpy()

            scale = 50


            pose[0][0] = pose[0][0]*640
            pose[0][1] = pose[0][1]*480
            pose[0][2] = pose[0][2]*np.pi

            ax1.cla()
            ax2.cla()
            ax3.cla()
            ax1.set_xlim(-1000,1000)
            ax1.set_ylim(-1000,1000)

            pos_pred = move_single(pos_pred,pose[0])
            
            dx,dy = np.cos(pos_pred[2]),np.sin(pos_pred[2])
            ax1.plot(pos_pred[1],pos_pred[0],"o",markerfacecolor='blue')
            ax1.arrow(pos_pred[1],pos_pred[0],dx*scale,dy*scale,fc='blue',ec='blue')
            ax3.imshow(original_img)

            old_img = img


            plt.pause(0.00001)
