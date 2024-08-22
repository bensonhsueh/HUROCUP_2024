import rospy
import cv2
import numpy as np
from collections import deque
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import random
import math
from scipy.signal import wiener


class VisionSystem:
    """
        To be used as a callback for a Subscriber to a camera topic, saves
        the images to a limited buffer. Can also run a sequence of functions
        on the image as soon as it is captured. Each function in the pipeline
        should return a tuple of its resulting value and success status. The
        first argument of the function should be an image.

        foo(img, *args) -> (result, success)

        Parameters:
            maxlen: The maximum size of the image buffer, old images are
                discarded.
            pipeline_funcs: A list of functions to be ran after reading a new
                image.
            pipeline_args: The arguments to each of the functions.

    """
    def __init__(self, maxlen=1, pipeline_funcs=[], pipeline_args=[], debug=False, verbose=1):
        self.verbose = verbose
        self.frame_count = 0
        self.img_buffer = deque(maxlen=maxlen)
        self.bridge = CvBridge()

        self.pipeline_funcs = pipeline_funcs
        self.pipeline_args = pipeline_args

        self.results = [None] * len(pipeline_funcs)
        self.status = [None] * len(pipeline_funcs)
        self.debug_img = [None] * len(pipeline_funcs)

        # self.results =  len(pipeline_funcs)
        # self.status =  len(pipeline_funcs)
        # self.debug_img =  len(pipeline_funcs)

        self.debug = debug

    def read(self, ros_msg=None):
        """ Acquires a new frame from a ROS message. This function is intended to
            be passed as callback when subscribing to a camera topic.

            Parameters:
                ros_msg: A ros message containing the image
        
        """
        try:
            img = self.bridge.imgmsg_to_cv2(ros_msg, "bgr8")
            
            # cv2.imwrite('/home/robotis/joe_ws/src/fira_basketball/config/output.jpg', img)
            self.img_buffer.append(img)
            self.frame_count += 1
        except CvBridgeError as e:
            rospy.loginfo(e)
        i_foo = 0
        # print(self.pipeline_funcs, self.pipeline_args)
        for func, args in zip(self.pipeline_funcs, self.pipeline_args):
            try:
                # The image passed is a copy so further functions are not affected
                copy_img = img.copy()
                result, success = func(copy_img, *args)
                if self.debug:
                    self.debug_img[i_foo] = copy_img

                self.status[i_foo] = success
                if success != False:
                    self.results[i_foo] = result

            except Exception as e:
                print(e)
                if self.verbose == 1:
                    rospy.loginfo("Failed to run function %d in vision pipeline" % i_foo)
                    rospy.loginfo(e)
                self.status[i_foo] = False

            i_foo += 1

def detect2Color(img, hsv_params1, hsv_params2):
    """ Detects a single color specified in HSV colorspace in the img_buffer
        
        Parameters:
            img 
            hsv_params: A tuple of two 3-dim numpy array specifing the HSV 
                range of the color.

        Return:
            (pos_x, pos_y): Returns the center position of the largest color
                blob normalized to the dimensions of the image.
            area: area of the largest color blob

    """
    lower1 = hsv_params1[0]
    upper1 = hsv_params1[1]
    lower2 = hsv_params2[0]
    upper2 = hsv_params2[1]

    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv_img, lower1, upper1)
    mask2 = cv2.inRange(hsv_img, lower2, upper2)

    # mask1 = cv2.morphologyEx(mask1, cv2.MORPH_OPEN, np.ones((5, 5)))
    # mask1 = cv2.morphologyEx(mask1, cv2.MORPH_CLOSE, np.ones((5, 5)))
    # mask2 = cv2.morphologyEx(mask2, cv2.MORPH_OPEN, np.ones((5, 5)))
    # mask2 = cv2.morphologyEx(mask2, cv2.MORPH_CLOSE, np.ones((5, 5)))

    mask = cv2.bitwise_or(mask1, mask2)

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5)))


    center_pos, area = findCenterOfLargestContour(mask)

    img[:, :, 2] = mask
    img[:, :, :2] = 0

    if center_pos is not None:
        # Normalize by image dimension
        c_x = center_pos[0] / float(img.shape[1])
        c_y = center_pos[1] / float(img.shape[0])

        center = (c_x, c_y)
        
        return (center, area), True
    else:
        return None, False

def get_contour_areas(contours):
    all_area = []

    for cnt in contours:
        area = cv2.contourArea(cnt)
        all_area.append(area)

    return all_area

def detectSingleColor(img, hsv_params):
    '''
    Detects a single color specified in HSV colorspace in the img_buffer
        
        Parameters:
            img 
            hsv_params: A tuple of one 3-dim numpy array specifing the HSV 
                range of the color.

        Return:
            (pos_x, pos_y): Returns the center position of the largest color
                blob normalized to the dimensions of the image.
            area: area of the largest color blob
    '''
    
    lower = hsv_params[0]
    upper = hsv_params[1]

    # print(hsv_params)

    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_img, lower, upper)
    
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5)))
    # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((10, 10)))
    # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((10, 10)))
    
    # print(1)
    center_pos, area = findCenterOfLargestContour(mask)

    img[:, :, 2] = mask
    img[:, :, :2] = 0
    
    if center_pos is not None:
        # Normalize by image dimension
        c_x = center_pos[0] / float(img.shape[1])
        c_y = center_pos[1] / float(img.shape[0])

        center = (c_x, c_y)
        
        return (center, area), True
    else:
        return None, False  

def detectAngleOfColorLine(img, hsv_params):
    '''
    Detects a single color specified in HSV colorspace in the img_buffer
        
        Parameters:
            img 
            hsv_params: A tuple of one 3-dim numpy array specifing the HSV 
                range of the color.

        Return:
            (pos_x, pos_y): Returns the center position of the largest color
                blob normalized to the dimensions of the image.
            area: area of the largest color blob
    '''
    
    lower = hsv_params[0]
    upper = hsv_params[1]

    # print(hsv_params)
    # img = cv2.resize(img, (0,0), fx=0.025, fy = 0.025)#change size
    h ,w , x = img.shape

    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_img, lower, upper)
    
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5)))
    # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((10, 10)))
    # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((10, 10)))
    
    # print(1)
    center_pos,xywh, area = findSquareOfLargestContour(mask)
    # center_pos, area = findCenterOfLargestContour(mask)

    img[:, :, 2] = mask
    img[:, :, :2] = 0
    
    if center_pos is not None:
        # Normalize by image dimension
        c_x = center_pos[0] / float(img.shape[1])
        c_y = center_pos[1] / float(img.shape[0])

        center = (c_x, c_y)
    # for i in range(w):
    #     if mask[][]
        
        return (center,xywh, area), True
    else:
        return None, False

def detectLabColor(img, lab_params):
    '''
    Detects a single color specified in HSV colorspace in the img_buffer
        
        Parameters:
            img 
            lab_params: A tuple of one 3-dim numpy array specifing the HSV 
                range of the color.

        Return:
            (pos_x, pos_y): Returns the center position of the largest color
                blob normalized to the dimensions of the image.
            area: area of the largest color blob
    '''
    
    lower = lab_params[0]
    upper = lab_params[1]


    lab_img = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    mask = cv2.inRange(lab_img, lower, upper)
    
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5)))
    
    center_pos, area = findCenterOfLargestContour(mask)

    img[:, :, 2] = mask
    img[:, :, :2] = 0
    
    if center_pos is not None:
        # Normalize by image dimension
        c_x = center_pos[0] / float(img.shape[1])
        c_y = center_pos[1] / float(img.shape[0])

        center = (c_x, c_y)
        
        return (center, area), True
    else:
        return None, False

def detectHlsColor(img, hls_params):
    lower = hls_params[0]
    upper = hls_params[1]

    # print(hsv_params)

    hls_img = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
    mask = cv2.inRange(hls_img, lower, upper)
    
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5)))
    
    # print(1)
    center_pos, area = findCenterOfLargestContour(mask)

    img[:, :, 2] = mask
    img[:, :, :2] = 0
    
    if center_pos is not None:
        # Normalize by image dimension
        c_x = center_pos[0] / float(img.shape[1])
        c_y = center_pos[1] / float(img.shape[0])

        center = (c_x, c_y)
        
        return (center, area), True
    else:
        return None, False
    
def findCenterOfLargestContour(binary_mask):
    """ Detects all contours in the image and returns the center position and 
        area of the largest contour.

        Parameters:
            binary_mask: A binary image, to detect the contours.

        Returns:
            (center_x, center_y), area: If no contours are detect it returns
                None, None.

    """
    _,contours,_ = cv2.findContours(binary_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) == 1:
        largest_cnt = 0
        largest_area = cv2.contourArea(contours[0])
    elif len(contours) > 1:
        largest_cnt = 0
        largest_area = cv2.contourArea(contours[0])
        for i, cnt in enumerate(contours[1:]):
            cnt_area = cv2.contourArea(cnt)
            if cnt_area > largest_area:
                largest_area = cnt_area
                largest_cnt = i+1 # Enumerate starts from 0, increment 1 here 
                                  # because we skip the first contour
    else: # No contours were found
        return None, None

    # Get moments of largest contour
    M = cv2.moments(contours[largest_cnt])
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    return (cx, cy), largest_area

def findSquareOfLargestContour(binary_mask):
    """ Detects all contours in the image and returns the center position and 
        area of the largest contour.

        Parameters:
            binary_mask: A binary image, to detect the contours.

        Returns:
            (center_x, center_y), area: If no contours are detect it returns
                None, None.

    """
    x = 0
    y = 0
    w = 0
    h = 0
    _,contours,_ = cv2.findContours(binary_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) == 1:
        largest_cnt = 0
        largest_area = cv2.contourArea(contours[0])
    elif len(contours) > 1:
        largest_cnt = 0
        largest_area = cv2.contourArea(contours[0])
        for i, cnt in enumerate(contours[1:]):
            cnt_area = cv2.contourArea(cnt)
            if cnt_area > largest_area:
                peri = cv2.arcLength(i,True)
                vertices = cv2.approxPolyDP(i, peri * 0.02, True)
                x , y , w , h = cv2.boundingRect(vertices)
                largest_area = cnt_area
                largest_cnt = i+1 # Enumerate starts from 0, increment 1 here (x, y, w, h) = cv2.boundingRect(c)
                                  # because we skip the first contour
    else: # No contours were found
        return None, None

    # Get moments of largest contour
    M = cv2.moments(contours[largest_cnt])
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    return (cx, cy) ,(x,y,w,h), largest_area

def follow_line_benson(img, lab_params):#garbage
    '''
    follow line in HSV colorspace in the img_buffer
        
        Parameters:
            img 
            hsv_params: A tuple of one 3-dim numpy array specifing the HSV 
                range of the color.

        Return:
            angle: Returns angle of the line.
            distance: return line distance from center -1 to 1
    '''
    y_res, x_res, channel = img.shape
    
    lower = lab_params[0]
    upper = lab_params[1]

    # print(hsv_params)

    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_img, lower, upper)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5)))
    # cv2.rectangle(mask, (0,0), (x_res,y_res/3*2), 0, -1)
    
    img[:, :, 2] = mask
    img[:, :, :2] = 0
    
    _,contours,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if len(contours) == 1:
        largest_cnt = 0
        largest_area = cv2.contourArea(contours[0])
    elif len(contours) > 1:
        largest_cnt = 0
        largest_area = cv2.contourArea(contours[0])
        for i, cnt in enumerate(contours[1:]):
            cnt_area = cv2.contourArea(cnt)
            if cnt_area > largest_area:
                largest_area = cnt_area
                largest_cnt = i+1 # Enumerate starts from 0, increment 1 here 
                                  # because we skip the first contour
    else: # No contours were found
        return None, None
    
    c = contours[largest_cnt]
    selection = cv2.minAreaRect(c)
    (x, y), (width, height), angle = selection
    angle = compute_angle(width, height, angle)
    
    #right-angle turns detection routine
    if width*height>0.2*x_res*y_res and not width*height>0.6*x_res*y_res:
        if abs(angle)>70:
            if angle<0 and x>x_res*0.5 and y>x_res*0.5:
                # print("Right!")
                angle=80
            elif angle>0 and x<x_res*0.5 and y>x_res*0.5:
                # print("Left!")
                angle=-80 
    
    middle = x_res // 2
    #get line distance from center and normalize
    distance = (x - middle)/middle 
    return (angle, distance), True

def follow_line_benson_V2(img, lab_params):#big garbage, too slow QQ
    y_res, x_res, channel = img.shape
    
    lower = lab_params[0]
    upper = lab_params[1]

    # print(hsv_params)

    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_img, lower, upper)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5)))
    # mask = skeletonization(mask)
    # cv2.rectangle(mask, (0,0), (x_res,y_res/3*2), 0, -1)
    
    img[:, :, 2] = mask
    img[:, :, :2] = 0
    
    _,contours,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if len(contours) == 1:
        pass
    elif len(contours) > 1:
        pass
    else: # No contours were found
        return None, None

    

    
    # middle = x_res // 2
    #get line distance from center and normalize
    # distance = (x - middle)/middle 
    # cv2.line(mask, (20,20), (50,250), 125, 5)
    # cv2.circle(mask, (50,50), 1, 200, 1)
    score_down = 0
    conx = (x_res*(x_res-1))/2
    # TODO:
    #   for row in mask:
    #      idxs_np_ay = np.array(row).where([row>255])
        #   
    #       
    #
    for i in range(0,x_res,10):
        # if (i % 5 != 0):
        #     continue
        score_down = score_down+ (mask[int(y_res*0.9)][i]/255)*(i-(x_res/2))
        # print(int(y_res*0.9),i,mask[int(y_res*0.9)][i],conx)
        # score_down = score_down / conx
        # cv2.circle(mask, (i,int(y_res*0.9)), 1, 200, 1)
    # score_down = score_down / 25

    
    img[:, :, 2] = mask
    img[:, :, :2] = 0
    return (score_down), True
    
def follow_line_benson_V3(img, lab_params):#epic garbage
    y_res, x_res, channel = img.shape
    
    lower = lab_params[0]
    upper = lab_params[1]

    # print(hsv_params)

    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_img, lower, upper)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5)))
    # mask = skeletonization(mask)
    # cv2.rectangle(mask, (0,0), (x_res,y_res/3*2), 0, -1)
    
    img[:, :, 2] = mask
    img[:, :, :2] = 0
    
    _, contours,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if len(contours) == 1:
        pass
    elif len(contours) > 1:
        pass
    else: # No contours were found
        return None, None

    

    
    # middle = x_res // 2
    #get line distance from center and normalize
    # distance = (x - middle)/middle 
    # cv2.line(mask, (20,20), (50,250), 125, 5)
    # cv2.circle(mask, (50,50), 1, 200, 1)
    score_down = 0
    conx = (x_res*(x_res-1))/2
    # TODO:
    #   for row in mask:
    #      idxs_np_ay = np.array(row).where([row>255])
        #   
    #       
    #
    for i in range(0,x_res,10):
        # if (i % 5 != 0):
        #     continue
        score_down = score_down+ (mask[int(y_res*0.9)][i]/255)*(i-(x_res/2))
        # print(int(y_res*0.9),i,mask[int(y_res*0.9)][i],conx)
        # score_down = score_down / conx
        # cv2.circle(mask, (i,int(y_res*0.9)), 1, 200, 1)
    # score_down = score_down / 25

    
    img[:, :, 2] = mask
    img[:, :, :2] = 0
    return (score_down), True

def follow_line_benson_V4(img, hsv_params1, hsv_params2, hsv_params3):
    '''
    follow line in HSV colorspace in the img_buffer
        
        Parameters:
            img 
            hsv_params: A tuple of one 3-dim numpy array specifing the HSV 
                range of the color.

        Return:
            angle: Returns angle of the line.
            distance: return line distance from center -1 to 1
    '''
    y_res, x_res, channel = img.shape
    
    lower1 = hsv_params1[0]
    upper1 = hsv_params1[1]
    lower2 = hsv_params2[0]
    upper2 = hsv_params2[1]
    lower3 = hsv_params3[0]
    upper3 = hsv_params3[1]

    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv_img, lower1, upper1)
    mask2 = cv2.inRange(hsv_img, lower2, upper2)
    mask3 = cv2.inRange(hsv_img, lower3, upper3)

    mask1 = cv2.morphologyEx(mask1, cv2.MORPH_OPEN, np.ones((5, 5)))
    mask1 = cv2.morphologyEx(mask1, cv2.MORPH_CLOSE, np.ones((5, 5)))
    mask2 = cv2.morphologyEx(mask2, cv2.MORPH_OPEN, np.ones((5, 5)))
    mask2 = cv2.morphologyEx(mask2, cv2.MORPH_CLOSE, np.ones((5, 5)))
    mask3 = cv2.morphologyEx(mask3, cv2.MORPH_OPEN, np.ones((5, 5)))
    mask3 = cv2.morphologyEx(mask3, cv2.MORPH_CLOSE, np.ones((5, 5)))

    mask = cv2.bitwise_or(mask1, mask2)
    mask = cv2.bitwise_or(mask, mask3)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5)))
    mask = cv2.dilate(mask , np.ones((5,5)),iterations =1)
    # cv2.rectangle(mask, (0,0), (x_res,y_res/3*2), 0, -1)
    
    # center_pos, area = findCenterOfLargestContour(mask)

    img[:, :, 2] = mask
    img[:, :, :2] = 0
    
    _,contours,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if len(contours) == 1:
        largest_cnt = 0
        largest_area = cv2.contourArea(contours[0])
    elif len(contours) > 1:
        largest_cnt = 0
        largest_area = cv2.contourArea(contours[0])
        for i, cnt in enumerate(contours[1:]):
            cnt_area = cv2.contourArea(cnt)
            if cnt_area > largest_area:
                largest_area = cnt_area
                largest_cnt = i+1 # Enumerate starts from 0, increment 1 here 
                                  # because we skip the first contour
    else: # No contours were found
        return None, None
    
    M = cv2.moments(contours[largest_cnt])
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    center_pos = (cx, cy)
    area = largest_area

    c = contours[largest_cnt]
    selection = cv2.minAreaRect(c)
    (x, y), (width, height), angle = selection
    angle = (-1)*compute_angle(width, height, angle)
    
    #right-angle turns detection routine
    if width*height>0.2*x_res*y_res and not width*height>0.6*x_res*y_res:
        if abs(angle)>70:
            if angle<0 and x>x_res*0.5 and y>x_res*0.5:
                # print("Right!")
                angle=80
            elif angle>0 and x<x_res*0.5 and y>x_res*0.5:
                # print("Left!")
                angle=-80 

    if center_pos is not None:
        # Normalize by image dimension
        c_x = center_pos[0] / float(img.shape[1])
        c_y = center_pos[1] / float(img.shape[0])

        center = (c_x, c_y)
        
        return (center, area, angle), True
    else:
        return None, False
    # middle = x_res // 2
    #get line distance from center and normalize
    # distance = (x - middle)/middle 
    # return (center, area, angle), True
    # return (angle, distance), True

def follow_line_benson_V5(img, hsv_params1, hsv_params2, hsv_params3):
    '''
    follow line in HSV colorspace in the img_buffer
        
        Parameters:
            img 
            hsv_params: A tuple of one 3-dim numpy array specifing the HSV 
                range of the color.

        Return:
            angle: Returns angle of the line.
            distance: return line distance from center -1 to 1
    '''
    y_res, x_res, channel = img.shape
    
    lower1 = hsv_params1[0]
    upper1 = hsv_params1[1]
    lower2 = hsv_params2[0]
    upper2 = hsv_params2[1]
    lower3 = hsv_params3[0]
    upper3 = hsv_params3[1]

    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv_img, lower1, upper1)
    mask2 = cv2.inRange(hsv_img, lower2, upper2)
    mask3 = cv2.inRange(hsv_img, lower3, upper3)

    mask1 = cv2.morphologyEx(mask1, cv2.MORPH_OPEN, np.ones((5, 5)))
    mask1 = cv2.morphologyEx(mask1, cv2.MORPH_CLOSE, np.ones((5, 5)))
    mask2 = cv2.morphologyEx(mask2, cv2.MORPH_OPEN, np.ones((5, 5)))
    mask2 = cv2.morphologyEx(mask2, cv2.MORPH_CLOSE, np.ones((5, 5)))
    mask3 = cv2.morphologyEx(mask3, cv2.MORPH_OPEN, np.ones((5, 5)))
    mask3 = cv2.morphologyEx(mask3, cv2.MORPH_CLOSE, np.ones((5, 5)))

    mask = cv2.bitwise_or(mask1, mask2)
    mask = cv2.bitwise_or(mask, mask3)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5)))
    mask = cv2.dilate(mask , np.ones((5,5)),iterations =1)
    # cv2.rectangle(mask, (0,0), (x_res,y_res/3*2), 0, -1)
    
    # center_pos, area = findCenterOfLargestContour(mask)

    img[:, :, 2] = mask
    img[:, :, :2] = 0

    min_contour_length = 100  # tune
    _,contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Filter contours based on area (which might be proportional to the thickness)
    # min_contour_area = 500  # tune
    # max_contour_area = 12000  # tune
    # filtered_contours = [cnt for cnt in contours if min_contour_area < cv2.contourArea(cnt) < max_contour_area]
    
    _,contours,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    min_contour_area = 500  # tune
    max_contour_area = 50000  # tune
    contours = [cnt for cnt in contours if min_contour_area < cv2.contourArea(cnt) < max_contour_area]

    longest_contour = None
    max_length = 0

    for cn in contours:

        rect = cv2.minAreaRect(cn)
        box = cv2.boxPoints(rect)
        box = np.intp(box)
       
        side1, side2 = rect[1]
        longest_side = max(side1, side2)  # we can calculate the lengths of the sides of the rectangle

        # pic_contours = cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)

        # Find the only the longest contour
        if longest_side > max_length and longest_side > min_contour_length:
            longest_contour = cn
            max_length = longest_side

    if longest_contour is not None:  # Draw the rectangle and display 
        area = cv2.contourArea(longest_contour)
        # area =0

        M = cv2.moments(longest_contour)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        center_pos = (cx, cy)
        c_x = center_pos[0] / float(img.shape[1])
        c_y = center_pos[1] / float(img.shape[0])
        center = (c_x, c_y)


        rect = cv2.minAreaRect(longest_contour)
        box = cv2.boxPoints(rect)
        box = np.intp(box)
        
        # print("rect")
        # print(rect)
        # print("----------------------------")
        # print("box")
        # print(box)
        # frame_with_box = cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)
        width  = rect[1][0]
        height = rect[1][1]
        angle = (-1)*compute_angle(width, height, rect[2])
        
        #right-angle turns detection routine
        if width*height>0.2*x_res*y_res and not width*height>0.6*x_res*y_res:
            if abs(angle)>70:
                if angle<0 and x>x_res*0.5 and y>x_res*0.5:
                    # print("Right!")
                    angle=80
                elif angle>0 and x<x_res*0.5 and y>x_res*0.5:
                    # print("Left!")
                    angle=-80 

        # angle = rect[2]
        # if rect[1][0] < rect[1][1]: # convert angles
        #     angle = (90 + angle) % 360
        # else:
        #     angle = (360 + angle) % 360

        # cv2.putText(frame_with_box, f"{angle:.1f}", (int(rect[0][0]), int(rect[0][1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (10, 40, 10), 2)
        return (center, area, angle), True
        # cv2.imshow('pic_contours', pic_contours)
    else:
        return None, False


    
def benson_arrow_detect_CV(img, angle_for_body, if_offset_camera_angle):
    '''
    Detects a arrow degree and desise the type of angle
        
        Parameters:
            img 
            hsv_params: Actually is useless,just for unite the form

        Return:
            (pos_x, pos_y): Returns the center position of the largest color
                blob normalized to the dimensions of the image.
            area: area of the largest color blob
    '''
    
    # lower = hsv_params[0]
    # upper = hsv_params[1]

    # print(hsv_params)
    
    
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray_img = cv2.GaussianBlur(gray_img, (3,3), 0)

    # unsharp_img = cv2.addWeighted(gray_img, 1.5, blur_img, -0.5, 0)
    # gray_unsharp_img = cv2.addWeighted(img, 1.5, blur_img, -0.5, 0)
   
    # blur_img = wiener(blur_img, (5, 5)) #deblur
    # gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# 
    # ret, process_img = cv2.threshold(unsharp_img, 80, 200, cv2.THRESH_BINARY)
    # ret, process_img = cv2.threshold(unsharp_img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # kernel = np.ones((3, 3), np.uint8)

    # process_img = cv2.erode(process_img, kernel, iterations = 1)

    # process_img = cv2.morphologyEx(process_img, cv2.MORPH_OPEN, np.ones((3, 3)))
    # process_img = cv2.morphologyEx(process_img, cv2.MORPH_CLOSE, np.ones((3, 3)))
    # process_img = cv2.morphologyEx(process_img, cv2.MORPH_OPEN, np.ones((3, 3)))
    # process_img = cv2.morphologyEx(process_img, cv2.MORPH_CLOSE, np.ones((3, 3)))

    # lower = hsv_params[0]
    # upper = hsv_params[1]

    # print(angle_for_body)

    # hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # process_img = cv2.inRange(hsv_img, lower, upper)
    # process_img = cv2.Canny()
    
    # process_img = cv2.morphologyEx(process_img, cv2.MORPH_OPEN, np.ones((3, 3)))
    # process_img = cv2.morphologyEx(process_img, cv2.MORPH_CLOSE, np.ones((3, 3)))
    # ret, process_img = cv2.threshold(gray_img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    # ret, process_img = cv2.threshold(gray_blur_img, 128, 255, cv2.THRESH_BINARY)

    process_img = cv2.Canny(gray_img,50,150)


    # ret, process_img = cv2.threshold(gray_img, 127, 255, cv2.THRESH_BINARY)
    # process_img = cv2.canny(blur_img, 100, 200)

    # process_img = cv2.morphologyEx(process_img, cv2.MORPH_OPEN, np.ones((3, 3)))
    process_img = cv2.morphologyEx(process_img, cv2.MORPH_CLOSE, np.ones((3, 3)))

    # process_img = cv2.GaussianBlur(process_img, (3,3), 0)


    # hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # mask = cv2.inRange(hsv_img, lower, upper)
    # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5)))
    # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5)))
    # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((10, 10)))
    # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((10, 10)))
    
    # print(1)
    # center_pos, area = findCenterOfLargestContour(mask)

    _,contours,_ = cv2.findContours(process_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # _,contours,_ = cv2.findContours(process_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    # _,contours,_ = cv2.findContours(process_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    ans = "none"
    offset = 45
    flag = 0
    not_90_angle_quantity = 0
    center_pos = [0,0]
    arrow_area = 0
    arrow_arclen = 0
    find_angle = False
    
    # print("len(contours) = ",len(contours))

    if len(contours) > 0:
        # contours = sorted(contours, key=cv2.contourArea, reverse=True)
        for contour in contours:
            # contour = cv2.convexHull(contour)
            contour = cv2.approxPolyDP(contour, 5,True)
            
            
            M = cv2.moments(contour)
            if M['m00'] != 0:
                centerX = int(M['m10'] / M['m00'])
                centerY = int(M['m01'] / M['m00'])
                # print("centor: ", centerX, centerY)
            # else:
            #     print("no area")
                
            # color = random.randint(0,255)

            contour_area = cv2.contourArea(contour)
            # print(contour_area)

            # cv2.drawContours(process_img, [contour], -1, (255), 2)

            if contour_area <=1800 or contour_area >= 5000:
                # cv2.putText(process_img, "A:"+str(contour_area) , (centerX, centerY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255), 1)
                # print("qq contour_area:", contour_area)
                continue
            # else:
            #     cv2.putText(process_img, "  good A:"+str(contour_area) , (centerX, centerY), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255), 1)
                # print("contour_area:", contour_area)



            if (len(contour) != 7) and (len(contour) != 9):
                # pass
                # cv2.putText(process_img, "L:"+str(len(contour))  , (centerX, centerY), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255), 1)
                # print("len(contour):", len(contour))
                continue
            # else:
            #     print("len(contour):", len(contour))
            #     print("under len area:", contour_area)
            #     cv2.putText(process_img, "L:"+str(len(contour))  , (centerX, centerY), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255), 1)

            arclen = cv2.arcLength(contour , True)
            if arclen >= 600 or arclen <= 160:
                # pass
                # cv2.putText(process_img, "AL:"+str(arclen) , (centerX, centerY), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255), 1)
                # print("arclen")
                continue
            x, y, w, h = cv2.boundingRect(contour)

            aspect_ratio = float(w) / h
            if aspect_ratio >= 3 or aspect_ratio <= (1/3):
                # cv2.putText(process_img, "AR:" , (centerX, centerY), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255), 1)
                # print("aspect_ratio")
                continue
            # print(aspect_ratio)

            # print(len(contour),contour_area,arclen)


            
            i = 0
            not_90_angle_quantity = 0
            too_short_length = 0
            angles = []
            lengths = []
            angle_is_clear = 1 
            for point in contour:
                # print("a")
                # print(len(point))
                x, y = point[0]
                # print("point",point)
                # print("contour",contour[i])
                # print("contour[(i-1) len(contour)",contour[(i-1)%len(contour)])
                # print("contour[(i+1) len(contour)]",contour[(i+1)%len(contour)])
                # print("len(contour)",len(contour))
                angle = calculate_angle_benson(contour[(i-1)%len(contour)][0], contour[i][0], contour[(i+1)%len(contour)][0])
                length = calculate_length_benson(contour[(i-1)%len(contour)][0], contour[i][0])
                if(abs(angle-90) >= 30):
                    not_90_angle_quantity = not_90_angle_quantity + 1
                if(length <= 20):
                    too_short_length = too_short_length + 1
                angles.append(angle)
                lengths.append(length)
                cv2.circle(process_img, (x, y), radius=3, color=128, thickness=-1)
                # cv2.putText(process_img, str(i) + "," + str(int(angle)) + "," + str(int(length)), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.3, 200, 1)
                # cv2.putText(process_img, str(i) + "," +  str(int(length)), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, 200, 1)
                # cv2.putText(process_img, str(int(angle)) + "," +  str(int(length)), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, 200, 1)
                # cv2.putText(process_img,str(int(length)), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, 200, 1)
                i = i+1
                
            # print(not_90_angle_quantity)
            if(not_90_angle_quantity > 4):
                # print("not_90_angle_quantity")
                angle_is_clear = 0
                continue
            # print(angles)
            if(too_short_length >= 4):
                # print("too_short_length")
                continue
            arrow_area = cv2.contourArea(contour)
            arrow_arclen = cv2.arcLength(contour , True)
            x = (contour[(i-1)%len(contour)][0][0] + contour[(i+1)%len(contour)][0][0])/2
            y = (contour[(i-1)%len(contour)][0][1] + contour[(i+1)%len(contour)][0][1])/2
            center_pos = (x,y)
            # print("aaaa")
            for i in range(len(angles)):
                # print("b")
                # if(angle_is_clear == 0):
                #     flag = 1
                #     ans = "unknow"
                #     direction_degree = 0
                    
                #     continue
                if angles[(i-1)%len(contour)] < 75 and angles[(i+1)%len(contour)] < 75:
                    # print("c")
                    arrow_area = cv2.contourArea(contour)
                    arrow_arclen = cv2.arcLength(contour , True)
                    # print("contour[i][0]",contour[i][0][0])
                    x = (contour[(i-1)%len(contour)][0][0] + contour[(i+1)%len(contour)][0][0])/2
                    y = (contour[(i-1)%len(contour)][0][1] + contour[(i+1)%len(contour)][0][1])/2
                    center_pos = (x,y)
                    # cv2.circle(process_img, (x, y), radius=3, color=128, thickness=-1)
                    cv2.line(process_img, (x, y), (contour[i][0][0], contour[i][0][1]), (128), 2)
                    w = contour[i][0][0] - x
                    h = y - contour[i][0][1]
                    if if_offset_camera_angle:
                        h = h/math.sin(math.radians(angle_for_body))
                    # print("w:",w,"  h:",h)
                    # try:
                    direction_degree = math.degrees(math.atan(float(h)/float(w)))
                    # except:
                    #     direction_degree = 0.0
                    if(w>0):
                        direction_degree = direction_degree - 90
                    elif(w<0):
                        direction_degree = direction_degree + 90
                    # if(w<0):
                    #     direction_degree = direction_degree + 180
                    # if(w>0 and h<0):
                    #     direction_degree = direction_degree + 360

                    # print(direction_degree)
                    if(not_90_angle_quantity > 2):
                        flag = 1
                        ans = "unknown"
                    elif(abs(direction_degree) < offset):
                        flag = 1
                        ans = "straight"
                        # print(ans)
                    elif(90+offset >= direction_degree >= 90-offset):
                        flag = 1
                        ans = "left"
                        # print(ans)
                    elif((-90)+offset >= direction_degree >= (-90)-offset):
                        flag = 1
                        ans = "right"
                    else:
                        flag = 1
                        ans = "back"
                    # print(ans)
                    
                    # if(90+offset >= direction_degree >= 90-offset):
                    #     flag = 1
                    #     ans = "straight"
                    #     # print(ans)
                    # elif(180+offset >= direction_degree >= 180-offset):
                    #     flag = 1
                    #     ans = "left"
                    #     # print(ans)
                    # elif(0+offset >= direction_degree >= 0 or 360 >= direction_degree >= 360-offset):
                    #     flag = 1
                    #     ans = "right"
                    #     # print(ans)
                        
                    # cv2.putText(img, ans+","+str(int(direction_degree)), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 1)
                    cv2.putText(process_img, ans+","+str('%.2f' % direction_degree), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (128), 1)
                    # print("find angle!")
                    # find_angle = True
                    # print(x,y)
                    # print(contour[i][0][0], contour[i][0][1])
                    # print(w,h)
                    # print(float(h)/float(w))
                    # print(direction_degree)
                    break
            if find_angle:
                break

        

        # print(len(contours))
        # sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)
        # cv2.drawContours(process_img, contours, -1, (128), 2)
        
        # print("------------------------")
        



    # img[:, :, 2] = mask
    # img[:, :, :2] = 0
    # print("c")
    img[:, :, 0] = process_img[:, :]
    img[:, :, 1] = process_img[:, :]
    img[:, :, 2] = process_img[:, :]
    # print("d")
    #     c_y = center_pos[1] / float(img.shape[0])

    #     center = (c_x, c_y)
        
    #     return (center, area), True
    # else:
    #     return ((10,10),0), True#False
    # print("flag",flag)
    
    if flag:
        # print("(direction_degree,ans)",(direction_degree,ans))
        c_x = center_pos[0] / float(img.shape[1])
        c_y = center_pos[1] / float(img.shape[0])
        center = (c_x, c_y)
        return (center,direction_degree,ans,arrow_area,arrow_arclen), True#False
    else:
        return None, False

def calculate_angle_benson(a, b, c):
    # a = a[0]
    # b = b[0]
    # c = c[0]
    # print("a:",a)
    # print("b:",b)
    # print("c:",c)
    # print("a[0]:",a[0])
    # print("b[0]",b[0])
    # print("a[0] - b[0]",a[0] - b[0])
    # ba a-b
    # bc c-b
    ba = [a[0] - b[0], a[1] - b[1]]
    bc = [c[0] - b[0], c[1] - b[1]]
    # print("ba:",ba)
    # print("bc:",bc)


    dot_product = ba[0] * bc[0] + ba[1] * bc[1]
    # print("dot_product:",dot_product)

    length_ba = math.sqrt(ba[0]**2 + ba[1]**2)
    length_bc = math.sqrt(bc[0]**2 + bc[1]**2)
    # print("length_ba:",length_ba)
    # print("length_bc:",length_bc)

    cos_angle = dot_product / (length_ba * length_bc)

    angle = math.acos(cos_angle)
    angle_degree = math.degrees(angle)
    # print("angle_degree:",angle_degree)

    return angle_degree

def calculate_length_benson(a, b):
    # a = a[0]
    # b = b[0]
    # print("a:",a)
    # print("b:",b)
    # print("a[0]:",a[0])
    # print("b[0]",b[0])
    # print("a[1]:",a[1])
    # print("b[1]",b[1])

    len = math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
    # print("len",len)
    return len

def follow_line(img, hsv_params_lower, hsv_params_upper):
    '''
    follow line in HSV colorspace in the img_buffer
        
        Parameters:
            img 
            hsv_params: A tuple of one 3-dim numpy array specifing the HSV 
                range of the color.

        Return:
            angle: Returns angle of the line.
            distance: return line distance from center -1 to 1
    '''
    y_res, x_res, channel = img.shape
    
    lower = hsv_params_lower
    upper = hsv_params_upper

    # print(hsv_params)

    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_img, lower, upper)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5)))
    
    img[:, :, 2] = mask
    img[:, :, :2] = 0
    
    __,contours,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if len(contours) == 1:
        largest_cnt = 0
        largest_area = cv2.contourArea(contours[0])
    elif len(contours) > 1:
        largest_cnt = 0
        largest_area = cv2.contourArea(contours[0])
        for i, cnt in enumerate(contours[1:]):
            cnt_area = cv2.contourArea(cnt)
            if cnt_area > largest_area:
                largest_area = cnt_area
                largest_cnt = i+1 # Enumerate starts from 0, increment 1 here 
                                  # because we skip the first contour
    else: # No contours were found
        return None, None
    
    c = contours[largest_cnt]
    selection = cv2.minAreaRect(c)
    (x, y), (width, height), angle = selection
    angle = compute_angle(width, height, angle)
    
    #right-angle turns detection routine
    if width*height>0.2*x_res*y_res and not width*height>0.6*x_res*y_res:
        if abs(angle)>70:
            if angle<0 and x>x_res*0.5 and y>x_res*0.5:
                print("Right!")
                angle=80
            elif angle>0 and x<x_res*0.5 and y>x_res*0.5:
                print("Left!")
                angle=-80 
    
    middle = x_res // 2
    #get line distance from center and normalize
    distance = (x - middle)/middle 
    return (angle, distance), True
    
    # if center_pos is not None:
    #     # Normalize by image dimension
    #     c_x = center_pos[0] / float(img.shape[1])
    #     c_y = center_pos[1] / float(img.shape[0])

    #     center = (c_x, c_y)
        
    #     return (center, area), True
    # else:
    #     return None, False

def compute_angle(width, height, angle):
    if angle < -90 or (width > height and angle < 0):
        return 90 + angle
    if width < height and angle > 0:
        return (90 - angle) * -1 
    return angle

def print_numbers(frame, angle, distance,intersection):
    y_res, x_res, channel = frame.shape
    angle_pos=(0,97*y_res//100)
    error_pos=(0,12*y_res//100)
    inters_pos=(93*x_res//100,97*y_res//100)
    cv2.putText(frame, str(angle), angle_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 1)
    cv2.putText(frame, str(distance), error_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,0,0), 1)


def detect3Color(img, params1, params2, params3):
    """ Detects a single color specified in HSV colorspace in the img_buffer
        
        Parameters:
            img 
            hsv_params: A tuple of two 3-dim numpy array specifing the HSV 
                range of the color.

        Return:
            (pos_x, pos_y): Returns the center position of the largest color
                blob normalized to the dimensions of the image.
            area: area of the largest color blob

    """
    lower1 = params1[0]
    upper1 = params1[1]
    lower2 = params2[0]
    upper2 = params2[1]
    lower3 = params3[0]
    upper3 = params3[1]


    hsv_img = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2HSV)
    hls_img = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2HLS)
    lab_img = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2LAB)

    mask1 = cv2.inRange(hsv_img, lower1, upper1)
    mask2 = cv2.inRange(hls_img, lower2, upper2)
    mask3 = cv2.inRange(lab_img, lower3, upper3)

    mask1 = open_close(mask1)
    mask2 = open_close(mask2)
    mask3 = open_close(mask3)

    mask = cv2.bitwise_or(mask1, mask2, mask3)
    mask3 = open_close(mask)

    center_pos, area = findCenterOfLargestContour(mask)

    img[:, :, 2] = mask
    img[:, :, :2] = 0

    if center_pos is not None:
        # Normalize by image dimension
        c_x = center_pos[0] / float(img.shape[1])
        c_y = center_pos[1] / float(img.shape[0])

        center = (c_x, c_y)
        
        return (center, area), True
    else:
        return None, False

def open_close(img):
    mask = cv2.morphologyEx(img, cv2.MORPH_OPEN, np.ones((5, 5)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5)))
    return mask

def skeletonization(img):
    # kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, [3,3])
    # kernel = np.array([[0,1,0],[1,1,1],[0,1,0]],np.uint8)
    kernel = np.array([[0,0,1,0,0],[0,0,1,0,0],[1,1,1,1,1],[0,0,1,0,0],[0,0,1,0,0]],np.uint8)
    # # kernel = cv2.getStructuringElement(cv2.MORPH_RECT , [3,3])
    # print(kernel)
    h ,w  = img.shape
    ans = np.zeros([h,w],np.uint8) 
    small = img.copy()
    img2 = img.copy()
    small = cv2.erode(small, kernel) 
    big = cv2.dilate(small, kernel) 
    sub = img2 - big
    ans = ans + sub
    img2 = small
    while(big.any()!=0):
        small = cv2.erode(small, kernel) 
        big = cv2.dilate(small, kernel) 
        sub = img2 - big
        ans = ans + sub
        img2 = small
    # print(big.any())
    # cv2.imshow("ans",ans)#show

    return ans