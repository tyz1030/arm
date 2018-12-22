import cv2
import numpy as np
from PyQt4.QtGui import QImage
import freenect
import csv
from rexarm import Rexarm
import time

class Kinect():
    def __init__(self,rexarm):
        self.rexarm = rexarm
        self.currentVideoFrame = np.array([])
        self.currentDepthFrame = np.array([])
        self.currentDetectFrame = np.array([])
        self.depth_detect_frame = np.array([])
        self.bin_detect_frame = np.array([])
        self.hsv_frame = np.array([])
        if(freenect.sync_get_depth() == None):
            self.kinectConnected = False
        else:
            self.kinectConnected = True
        
        """mouse clicks & calibration variables"""
        self.depth2rgb_affine = np.float32([[1,0,0],[0,1,0]])
        self.rgb2w_affine = np.zeros((3,3))
        self.w2rgb_affine = np.zeros((3,3))
        self.kinectCalibrated = False
        self.last_click = np.array([0,0])
        self.new_click = False
        self.rgb_click_points = np.zeros((5,2),int)
        self.depth_click_points = np.zeros((5,2),int)
        self.real_coord = np.zeros((640,480,2))
        self.pose = np.zeros(4)
        
        """Real world points"""
        self.real_points = np.array([[-270,270],[-270,-270],[270,-270],[270,270],[0,120]])

        """ Extra arrays for colormaping the depth image"""
        self.DepthHSV = np.zeros((480,640,3)).astype(np.uint8)
        self.DepthCM=np.array([])

        """ block info """
        self.block_contours = np.array([])
        self.block_contours_2 = np.array([])
        self.block_contours_3 = np.array([])
        self.block_pos = np.zeros((50,3))
        self.block_ori = np.zeros((50,3))
        self.block_color = np.zeros(50)
        self.hsv = np.zeros(3)
        self.hsv_cali_data = np.zeros((100,43))
        self.hsv_row = 0
        self.hsv_col = 0
        # it is actually the index for block_pos, which starts from 0 and is not relavant to actual block numbers
        self.block_num = 0
        self.block_veri = np.zeros((50,3))
        self.block_veri_num = 0
       

    def captureVideoFrame(self):
        """                      
        Capture frame from Kinect, format is 24bit RGB    
        """
        if(self.kinectConnected):
            self.currentVideoFrame = freenect.sync_get_video()[0]
        else:
            self.loadVideoFrame()
        #self.processVideoFrame()

    #def processVideoFrame(self):
    #    cv2.drawContours(self.currentVideoFrame,self.block_contours,-1,(255,0,255),3)

    def captureDepthFrame(self):
        """                      
        Capture depth frame from Kinect, format is 16bit Grey, 10bit resolution.
        """
        if(self.kinectConnected):
            if(self.kinectCalibrated):
                self.currentDepthFrame = self.registerDepthFrame(freenect.sync_get_depth()[0])
            else:
                self.currentDepthFrame = freenect.sync_get_depth()[0]
        else:
            self.loadDepthFrame()

    def processDetectFrame(self):
        # load current video frame
        if(self.kinectConnected):
            self.currentDetectFrame = self.currentVideoFrame.copy()
        else:
            self.loadVideoFrame()
        # load detect function
        self.currentDetectFrame = self.blockDetector(self.currentDetectFrame)

    def loadVideoFrame(self):
        self.currentVideoFrame = cv2.cvtColor(
            cv2.imread("data/ex0_bgr.png",cv2.IMREAD_UNCHANGED),
            cv2.COLOR_BGR2RGB
            )

    def loadDepthFrame(self):
        self.currentDepthFrame = cv2.imread("data/ex0_depth16.png",0)

    def convertFrame(self):
        """ Converts frame to format suitable for Qt  """
        try:
            img = QImage(self.currentVideoFrame,
                             self.currentVideoFrame.shape[1],
                             self.currentVideoFrame.shape[0],
                             QImage.Format_RGB888
                             )
            return img
        except:
            return None

    def convertDepthFrame(self):
        """ Converts frame to a colormaped format suitable for Qt  
            Note: this cycles the spectrum over the lowest 8 bits
        """
        try:
            """ 
            Convert Depth frame to rudimentary colormap
            """
            self.DepthHSV[...,0] = self.currentDepthFrame
            self.DepthHSV[...,1] = 0x9F
            self.DepthHSV[...,2] = 0xFF
            self.DepthCM = cv2.cvtColor(self.DepthHSV,cv2.COLOR_HSV2RGB)
            cv2.drawContours(self.DepthCM,self.block_contours,-1,(0,0,0),3)
            cv2.drawContours(self.DepthCM,self.block_contours_2,-1,(0,0,0),3)
            cv2.drawContours(self.DepthCM,self.block_contours_3,-1,(0,0,0),3)

            img = QImage(self.DepthCM,
                             self.DepthCM.shape[1],
                             self.DepthCM.shape[0],
                             QImage.Format_RGB888
                             )
            return img
        except:
            return None

    def convertDetectFrame(self):
        """ Converts detect frame to format suitable for Qt  """
       
        self.processDetectFrame()
        try:
            img = QImage(self.currentDetectFrame,
                             self.currentDetectFrame.shape[1],
                             self.currentDetectFrame.shape[0],
                             QImage.Format_RGB888
                             )
            return img
        except:
            return None

    def getAffineTransform(self, coord1, coord2):
        
        # coord1 and 2 should be 2*10 matices
        coord1 = np.array(coord1)
        coord2 = np.array(coord2)
        # Should be 1*6 matrix
        affine_par = np.zeros((6,1))
        affine_matrix = np.zeros((3,3))
        A = np.zeros((10,6))
        b = np.zeros((10,1))
        # forming A
        i = 0
        for row in coord1:
            for j in range(2):
                if j == 0:
                    tmpA = np.array([row[0],row[1],1,0,0,0])
                else:
                    tmpA = np.array([0,0,0,row[0],row[1],1])
                A[i] = tmpA
                tmpA = []
                i += 1
        # forming b
        i = 0
        for row in coord2:
            for j in range(2):
                if j == 0:
                    b[i] = np.array([row[0]])
                    i += 1
                else:
                    b[i] = np.array([row[1]])
                    i += 1
        # Find affine parameters: x=((A^T*A)^(-1))*A^T*b
        affine_par = np.dot(np.dot(np.linalg.inv(np.dot(np.transpose(A),A)),np.transpose(A)),b)
        # form 3*3 affine matrix
        k=0
        for i in range(2):
            for j in range(3): 
                affine_matrix[i][j] = affine_par[k] 
                k += 1
        affine_matrix[2][0] = 0
        affine_matrix[2][1] = 0
        affine_matrix[2][2] = 1

        return affine_matrix

    def registerDepthFrame(self, frame):
        """
        Using an Affine transformation, transform the depth frame to match the RGB frame
        """
        h, w = frame.shape[:2]
        frame = cv2.warpAffine(frame,self.depth2rgb_affine,(w,h))

        return frame

    def convertDepthtomm(self,depth):
        """This is only for display"""

        depth = 2.968*10**-05*depth+0.02079*depth+0.5146
        
        return depth

    def loadCameraCalibration(self):
        """
        Load camera intrinsic matrix from file.
        """

        # Read calibration.csv
        with open("util/calibration.csv", 'rb') as csvfile:
            csvreader = csv.reader(csvfile, delimiter=",", quotechar="|")
            tmp = []
            intrinsic_matrix = []
            distort_coef = []
            i = 0
            for row in csvreader:
                for col in row:
                    try:
                        tmp.append(float(col))
                    except:
                        print("ERROR in calibration.csv intrinsic matrix")
                if(i!=3):
                    intrinsic_matrix.append(tmp)
                    i += 1
                    tmp = []
                if(i==3):
                    distort_coef = tmp
                    tmp = []
            
        return intrinsic_matrix, distort_coef

    def saveCalibrationPoints(self):
        """ Save current calibration points into csv"""

        if self.kinectCalibrated == True:
            with open('cali_points.csv', 'wb') as csvfile:
                csvwriter = csv.writer(csvfile, delimiter=',')
                for row in range(5):
                    csvwriter.writerow(self.rgb_click_points[row])
                for row in range(5):    
                    csvwriter.writerow(self.depth_click_points[row])
        pass

    def loadCalibrationPoints(self):
        """load previous calibration points and calibrate the camera"""

        with open('cali_points.csv', 'rb') as csvfile:
            csvreader = csv.reader(csvfile, delimiter=",", quotechar="|")
            i = 0

            for row in csvreader:
                j = 0
                for col in row:
                    
                    if i < 5:
                        self.rgb_click_points[i][j] = int(col)
                        j += 1
                        if j == 2:
                            j = 0
                    elif i > 4 :
                        self.depth_click_points[i-5][j] = int(col)
                        j += 1
                        if j ==2:
                            j = 0
                i+=1
            self.cameraCalibration()
        pass
        
    def cameraCalibration(self):
        # get affine transform matrix
        self.rgb2w_affine = self.getAffineTransform(self.rgb_click_points,self.real_points)
        self.w2rgb_affine = np.linalg.inv(self.rgb2w_affine)
        # use affine transform to get realworld coordinate of each pixal, and store in real_coord
        # real_coord(Xpixal,Ypixal,real_world_coord)
        rgb_coord = np.zeros((3,1))
        for i in range(640):
            for j in range(480):
                rgb_coord = np.array([[i],[j],[1]])
                for k in range(2):
                    for f in range(3):
                        self.real_coord[i][j][k] += self.rgb2w_affine[k][f] * rgb_coord[f]
        # get affine matrix for depth frame
        depth2rgb_affine_3by3 = self.getAffineTransform(self.depth_click_points,self.rgb_click_points)
        self.depth2rgb_affine = depth2rgb_affine_3by3[:2][:] 
        self.kinectCalibrated = True
        pass

    def blockDetector(self, frame):
        """
        find block centers from contours and store it in block_pos
        """
        self.detectBlocksInDepthImage()
        self.block_pos = np.zeros((50,3))
        self.block_num = 0
        pixal_arm = np.zeros(2)
        if self.kinectCalibrated == True:
            # get current arm position
            real_arm_x, real_arm_y,_,_ = self.rexarm.get_wrist_pose()
            real_arm_x *= -1000
            real_arm_y *= 1000
            real_arm = np.array(([real_arm_x],[real_arm_y],[1]))
            
            # normalize arm_line vector
            arm_length = np.sqrt(real_arm_x**2 + real_arm_y**2)
            l = np.array([real_arm_x,real_arm_y])

            # find center of block_1_height and put them into block_position
            for cnt in self.block_contours:
                M = cv2.moments(cnt)
                if M['m00']>0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    c = np.array([[cx],[cy]])
                    # Eliminate Arm itself
                    real_x = self.real_coord[cx][cy][0]
                    real_y = self.real_coord[cx][cy][1]
                    d = np.linalg.norm(np.cross(l, np.array([real_x,real_y])))/np.linalg.norm(l)
                    
                    d_to_arm = np.sqrt((real_x-real_arm_x)**2+(real_y-real_arm_y)**2)
                    d_to_ori = np.sqrt(real_x**2 + real_y**2)

                    
                    if d > 2 and not(d_to_ori<arm_length and d_to_arm<arm_length):
                        # Check if its in our ROI
                        if self.real_coord[cx][cy][0]>self.real_points[0][0] and self.real_coord[cx][cy][0]<self.real_points[2][0]:
                            if self.real_coord[cx][cy][1]<self.real_points[0][1] and self.real_coord[cx][cy][1]>self.real_points[2][1]:
                                # points
                                self.block_pos[self.block_num][0] = self.real_coord[cx][cy][0]
                                self.block_pos[self.block_num][1] = self.real_coord[cx][cy][1]
                                self.block_pos[self.block_num][2] = 1
                                # orientation
                                rect = cv2.minAreaRect(cnt)
                                self.block_ori[self.block_num][0] = -rect[2]
                                box = cv2.boxPoints(rect)
                                box = np.int0(box)
                                # detect color
                                self.block_color[self.block_num] = self.colorDetector(cx,cy)
                                # draw contours
                                cv2.drawContours(self.currentDetectFrame,[box],0,(30,145,86),3)
                                cv2.circle(self.currentDetectFrame,(cx,cy),5,(30,145,86),-1)
                                self.block_num += 1

            # find centers of 2 blocks
            for cnt in self.block_contours_2:
                M = cv2.moments(cnt)
                if M['m00']>0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    c = np.array([[cx],[cy]])
                    # Eliminate Arm itself
                    real_x = self.real_coord[cx][cy][0]
                    real_y = self.real_coord[cx][cy][1]
                    d = np.linalg.norm(np.cross(l, np.array([real_x,real_y])))/np.linalg.norm(l)
                   
                    d_to_arm = np.sqrt((real_x-real_arm_x)**2+(real_y-real_arm_y)**2)
                    d_to_ori = np.sqrt(real_x**2 + real_y**2)
                    
                    
                    if d > 2 and not(d_to_ori<arm_length and d_to_arm<arm_length):
                        # Check if its in our ROI
                        if self.real_coord[cx][cy][0]>self.real_points[0][0] and self.real_coord[cx][cy][0]<self.real_points[2][0]:
                            if self.real_coord[cx][cy][1]<self.real_points[0][1] and self.real_coord[cx][cy][1]>self.real_points[2][1]:
                                # points
                                self.block_pos[self.block_num][0] = self.real_coord[cx][cy][0]
                                self.block_pos[self.block_num][1] = self.real_coord[cx][cy][1]
                                self.block_pos[self.block_num][2] = 2
                                # orientation
                                rect = cv2.minAreaRect(cnt)
                                self.block_ori[self.block_num][0] = -rect[2]
                                box = cv2.boxPoints(rect)
                                box = np.int0(box)
                                # detect color
                                self.block_color[self.block_num] = self.colorDetector(cx,cy)
                                # draw contours
                                cv2.drawContours(self.currentDetectFrame,[box],0,(30,87,137),3)
                                cv2.circle(self.currentDetectFrame,(cx,cy),5,(30,87,137),-1)
                                self.block_num += 1   

            # find centers of 3 blocks
            for cnt in self.block_contours_3:
                M = cv2.moments(cnt)
                if M['m00']>0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    c = np.array([[cx],[cy]])
                    # Eliminate Arm itself
                    real_x = self.real_coord[cx][cy][0]
                    real_y = self.real_coord[cx][cy][1]
                    d = np.linalg.norm(np.cross(l, np.array([real_x,real_y])))/np.linalg.norm(l)
                    
                    d_to_arm = np.sqrt((real_x-real_arm_x)**2+(real_y-real_arm_y)**2)
                    d_to_ori = np.sqrt(real_x**2 + real_y**2)
                    
                    
                    if d > 2 and not(d_to_ori<arm_length and d_to_arm<arm_length):
                        # Check if its in our ROI
                        if self.real_coord[cx][cy][0]>self.real_points[0][0] and self.real_coord[cx][cy][0]<self.real_points[2][0]:
                            if self.real_coord[cx][cy][1]<self.real_points[0][1] and self.real_coord[cx][cy][1]>self.real_points[2][1]:
                                # points
                                self.block_pos[self.block_num][0] = self.real_coord[cx][cy][0]
                                self.block_pos[self.block_num][1] = self.real_coord[cx][cy][1]
                                self.block_pos[self.block_num][2] = 3
                                # orientation
                                rect = cv2.minAreaRect(cnt)
                                self.block_ori[self.block_num][0] = -rect[2]
                                box = cv2.boxPoints(rect)
                                box = np.int0(box)
                                # detect color
                                self.block_color[self.block_num] = self.colorDetector(cx,cy)
                                # draw contours
                                cv2.drawContours(self.currentDetectFrame,[box],0,(204,6,6),3)
                                cv2.circle(self.currentDetectFrame,(cx,cy),5,(204,6,6),-1)
                                self.block_num += 1                           
            self.block_pos[self.block_num:50] = 0
            self.block_ori[self.block_num:50] = 0

        return frame

    def detectBlocksInDepthImage(self):
        """
        find contours in depth frame
        """
        self.depth_detect_frame = self.currentDepthFrame

        # 1 block
        self.bin_detect_frame_1 = cv2.inRange(self.depth_detect_frame,700,710)
        kernel = np.ones((6,6),np.uint8)
        self.bin_detect_frame_1 = cv2.erode(self.bin_detect_frame_1,kernel,iterations = 1)
        _, self.block_contours, _ = cv2.findContours(self.bin_detect_frame_1,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        
        # 2 blocks
        self.bin_detect_frame_2 = cv2.inRange(self.depth_detect_frame,680,699)
        kernel = np.ones((6,6),np.uint8)
        self.bin_detect_frame_2 = cv2.erode(self.bin_detect_frame_2,kernel,iterations = 1)
        _, self.block_contours_2, _ = cv2.findContours(self.bin_detect_frame_2,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        
        # 3 blocks
        self.bin_detect_frame_3 = cv2.inRange(self.depth_detect_frame,660,679)
        kernel = np.ones((6,6),np.uint8)
        self.bin_detect_frame_3 = cv2.erode(self.bin_detect_frame_3,kernel,iterations = 1)
        _, self.block_contours_3, _ = cv2.findContours(self.bin_detect_frame_3,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        
        pass

    def colorCalibration(self,x,y):
        """
        Color Calibration
        """
        if self.new_click == True:
            for self.hsv_row in range(100):
                color = self.colorDetector(x,y)
                self.hsv_cali_data[self.hsv_row][self.hsv_col]=self.hsv[0]
                self.hsv_cali_data[self.hsv_row][self.hsv_col+1]=self.hsv[1]
                self.hsv_cali_data[self.hsv_row][self.hsv_col+2]=self.hsv[2]
                    
            self.hsv_col+=3
            self.new_click = False
            print("finish record this pos!")
            print(self.hsv_col/3)
            
        if self.hsv_col == 3*14:
            with open('blue_cali.csv', 'wb') as csvfile:
                csvwriter = csv.writer(csvfile, delimiter=',')
                for row in range(100):
                    csvwriter.writerow(self.hsv_cali_data[row])
                    time.sleep(0.01)
            print("color daya saved to csv!")   
        pass

    def colorDetector(self,x,y):
        
        # (x,y) are in pixal frame
        self.hsv_frame = self.currentVideoFrame.copy()
        try:
            self.hsv_frame = cv2.cvtColor(self.hsv_frame, cv2.COLOR_RGB2HSV)
        except:
            return
        b = self.currentVideoFrame[y][x][2]
        g = self.currentVideoFrame[y][x][1]
        r = self.currentVideoFrame[y][x][0]

        h = self.hsv_frame[y][x][0]
        s = self.hsv_frame[y][x][1]
        v = self.hsv_frame[y][x][2]

        roi = self.hsv_frame[y-2:y+2,x-2:x+2]

        h,s,v,_ = np.uint8(cv2.mean(roi))       
        
        self.hsv[0]=h
        self.hsv[1]=s
        self.hsv[2]=v
        
        """ start color segmentation """

        # black
        if h>135 and h<180 and s>11 and s<48 and v>69 and v<109:
            block_color = 1  
            #print("black")  
        # red
        elif h>163 and h<187 and s>60 and s<231 and v>154 and v<246:
            block_color = 2
            #print("red")
        # orange
        elif h>0 and h<20 and s>7 and s<250 and v>225 and v<253:
            block_color = 3
            #print("orange")
        # yellow
        elif h>=0 and h<40 and s>=0 and s<130 and v>250:
            block_color = 4
            #print("yellow")
        # green
        elif h>49 and h<88 and s>6 and s<103 and v>136 and v<237:
            block_color = 5
            #print("green")
        # blue
        elif h>103 and h<127 and s>66 and s<139 and v>158 and v<231:
            block_color = 6
            #print("blue")
        # violet
        elif h>138 and h<165 and s>56 and s<93 and v>125 and v<162:
            block_color = 7
            #print("violet")
        # pink
        elif h>154 and h<184 and s>37 and s<178 and v>233:
            block_color = 8
            #print("pink")
        # can't tell
        else:
            block_color = 3
            #print("I am stupid. I can't tell the color")


        

        
        return block_color

    def click_to_world(self, click):
        [x, y] = click

        print('clicked pos')
        
        # Get click pos in world
        x_world = self.real_coord[x,y,0]
        y_world = self.real_coord[x,y,1]

        print(x_world, y_world)
        z = self.currentDepthFrame[y,x]
        z_world = self.convertDepthtomm(z)
        
        return np.array([x_world, y_world, z_world])

    def find_nearest_block(self, world):
        # Only need x,y position from world 
        pos = world[:2]
        print(pos)

        block_xy = self.block_pos[:, :2].copy()
        print(block_xy)
        print(block_xy - np.array(pos))
        block_dist = np.sum(np.abs(pos - block_xy),axis=1)

        print('block distances')
        print(block_dist)
        if np.min(block_dist) > 100:
            print('No block nearby')
            return [], 0

        idx = np.where(block_dist == np.min(block_dist))
        nearest_block = self.block_pos[idx, :]
        nearest_block_ori = self.block_ori[idx,0]

        print('nearest block')
        print(nearest_block)
        print('orientation', nearest_block_ori)
        return nearest_block, nearest_block_ori

    def get_clicked_block(self, click):
        
        world = self.click_to_world(click)
        # find the nearest block
        return self.find_nearest_block(world)
    def set_end_effector(self,pose):

        self.pose = pose
        pass
 
    def find_nearest(self, array, dim, value):
        
        idx = (np.abs(array[:,:,dim] - value)).argmin()
        return idx

    def find_empty_space(self):

        current_pos = np.array([210,210,1])

        while True:
            # end of this row
            if current_pos[0] == -195:
                current_pos[0] = 210
                current_pos[1] -= 45
                # if can't find empty space
                if current_pos[1] < 0:
                    return np.array([0,240,1])
            # start looking for empty space
            for i in range(self.block_num):
                if self.block_pos[i][0]<current_pos[0]+45 or self.block_pos[i][0]>current_pos[0]-45 \
                   or self.block_pos[i][1]<current_pos[1]+45 or self.block_pos[i][0]>current_pos[1]-45:
                    current_pos[0] -= 45
                    break
                elif i == self.block_num-1:
                    return current_pos    
    
    def record_block_position(self):

        """
        Block detection verification
        """
        """
        if self.new_click == True:
            world = np.array([x,y,1])
            self.block_veri[self.block_veri_num],_ = self.find_nearest_block(world)
            self.block_veri_num += 1
            print("finish record this pos!")
            self.new_click = False
            
        if self.block_veri_num == 14:
            with open('block_veri_1.csv', 'wb') as csvfile:
                csvwriter = csv.writer(csvfile, delimiter=',')
                for row in range(16):
                    csvwriter.writerow(self.block_veri[row])
                    time.sleep(0.01)
            print("block data saved to csv!")   
        pass"""
        print(self.block_pos[:16])
        with open('block_veri_1.csv', 'wb') as csvfile:
            csvwriter = csv.writer(csvfile, delimiter=',')
            for row in range(50):
                csvwriter.writerow(self.block_pos[row])
                   
            print("block data saved to csv!")   
            