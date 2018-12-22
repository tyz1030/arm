import time
import numpy as np
import copy
import cv2
import freenect
from kinematics import IK
"""
TODO: Add states and state functions to this class
        to implement all of the required logic for the armlab
"""

class StateMachine():
    def __init__(self, rexarm, planner, kinect):
        self.rexarm = rexarm
        self.tp = planner
        self.kinect = kinect
        self.status_message = "State: Idle"
        self.current_state = "idle"
        self.next_state = "idle"
        self.verbose = False

    def set_next_state(self, state):
        self.next_state = state

    """ This function is run continuously in a thread"""

    def run(self):
        if(self.current_state == "manual"):
            if (self.next_state == "manual"):
                self.manual()
            if (self.next_state == "add_wp"):
                self.add_wp()
            if (self.next_state == "clear_wp"):
                self.clear_wp()
            if(self.next_state == "idle"):
                self.idle()                
            if(self.next_state == "estop"):
                self.estop()

        if(self.current_state == "idle"):
            if(self.next_state == "manual"):
                self.manual()
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "estop"):
                self.estop()
            if(self.next_state == "calibrate"):
                self.calibrate()
            if(self.next_state == "execute"):
                self.execute()
            if (self.next_state == "clear_wp"):
                self.clear_wp()
            if (self.next_state == "click_and_pick"):
                self.click_and_pick()
            if (self.next_state == "save_calibration_points"):
                self.save_calibration_points()
            if (self.next_state == "load_previous_calibration"):
                self.load_previous_calibration()
            if (self.next_state == "record_block_position"):
                self.record_block_position()
            if (self.next_state == "mirror"):
                self.mirror()
            if (self.next_state == "stack_3"):
                self.stack_3()
            if (self.next_state == "line_em_up"):
                self.line_em_up()
            if (self.next_state == "stack_em_high"):
                self.stack_em_high()
            if (self.next_state == "pyramid"):
                self.pyramid()   
            if (self.next_state == "pyramid4"):
                self.pyramid4()   
            if (self.next_state == "pyramid5"):
                self.pyramid5()   
            

        if(self.current_state == "estop"):
            self.next_state = "estop"
            self.estop()  

        if(self.current_state == "calibrate"):
            if(self.next_state == "idle"):
                self.idle()
               
        if(self.current_state == "execute"):
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "estop"):
                self.estop()

        if(self.current_state == "add_wp"):
            if(self.next_state == "manual"):
                self.manual()
        
        if(self.current_state == "clear_wp"):
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "manual"):
                self.manual()

        if(self.current_state == "click_and_pick"):
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "manual"):
                self.manual()

        if(self.current_state == "save_calibration_points"):
            if(self.next_state == "idle"):
                self.idle()

        if(self.current_state == "load_previous_calibration"):
            if(self.next_state == "idle"):
                self.idle()

        if(self.current_state == "record_block_position"):
            if(self.next_state == "idle"):
                self.idle()
        
        if(self.current_state == "mirror"):
            if(self.next_state == "idle"):
                self.idle()
        
        if(self.current_state == "stack_3"):
            if(self.next_state == "idle"):
                self.idle()

        if(self.current_state == "line_em_up"):
            if(self.next_state == "idle"):
                self.idle()
        if(self.current_state == "stack_em_high"):
            if(self.next_state == "idle"):
                self.idle()

        if(self.current_state == "pyramid"):
            if(self.next_state == "idle"):
                self.idle()

        if(self.current_state == "pyramid4"):
            if(self.next_state == "idle"):
                self.idle()

        if(self.current_state == "pyramid5"):
            if(self.next_state == "idle"):
                self.idle()

    """Functions run for each state"""


    def manual(self):
        self.status_message = "State: Manual - Use sliders to control arm"
        self.current_state = "manual"

    def idle(self):
        self.status_message = "State: Idle - Waiting for input"
        self.current_state = "idle"

    def estop(self):
        self.status_message = "EMERGENCY STOP - Check Rexarm and restart program"
        self.current_state = "estop"
        self.rexarm.disable_torque()
        
    def calibrate(self):
        self.current_state = "calibrate"
        self.next_state = "idle"
        
        location_strings = ["lower left corner of board",
                            "upper left corner of board",
                            "upper right corner of board",
                            "lower right corner of board",
                            "center of shoulder motor"]

        i = 0
        for j in range(5):
            self.status_message = "Calibration - Click %s in RGB image" % location_strings[j]
            while (i <= j):
                if(self.kinect.new_click == True):
                    self.kinect.rgb_click_points[i] = self.kinect.last_click.copy()
                    i = i + 1
                    self.kinect.new_click = False        
        
        i = 0
        for j in range(5):
            self.status_message = "Calibration - Click %s in depth image" % location_strings[j]
            while (i <= j):
                if(self.kinect.new_click == True):
                    self.kinect.depth_click_points[i] = self.kinect.last_click.copy()
                    i = i + 1
                    self.kinect.new_click = False
        print(self.kinect.rgb_click_points)
        """camera calibration"""
        self.kinect.cameraCalibration()
        
        # finish calibration
        self.status_message = "Calibration - Completed Calibration"
        time.sleep(1)


    def add_wp(self):
        self.current_state = "add_wp"
        self.next_state = "manual"

        # we need to check that we are at zero torque -- this is our criteria for adding waypoints so arm is directed
        if self.rexarm.max_torque.any():
            # torque non zero --- err
            self.status_message =  "err: please set torque to zero"
            return

        # otherwise we get feedback
        waypoint = copy.copy(self.rexarm.joint_angles_fb)

        # add to trajectory planner
        self.tp.add_waypoint(waypoint)

    def clear_wp(self):
        self.next_state = self.current_state
        self.current_state = "clear_wp"
        # clear waypoints
        self.tp.clear_waypoints()
    
    def click_and_pick(self):
        '''
            Takes two clicked pixel locations from screen
            maps to world coordinates and attempts to pick up block at first
            location and drop off at second location
        '''

        self.current_state = "click_and_pick"
        self.next_state = "idle"
        
        self.kinect.new_click = False
        
        # wait for click
        while not self.kinect.new_click:
            time.sleep(0.05)

        # GET BLOCK
        block, ori = self.kinect.get_clicked_block(self.kinect.last_click.copy())

        ori *= np.pi/180.0

        if not len(block):
            print('ERROR: No block at clicked location')
            return
        else:
            block = block[0][0]
            ori = ori[0][0]
    
        if self.verbose:
            print('BLOCK ORIENTATION', ori)

        # convert to m
        pos = np.zeros(3)
        pos[:2] = block[:2] / 1000.0
        pos[2] = block[2] * 0.038
        
        # pick up block
        self.tp.pick_up_block(pos, ori)

        self.kinect.new_click = False
        # wait for next click
        while not self.kinect.new_click:
            time.sleep(0.05)

        world = self.kinect.click_to_world(self.kinect.last_click.copy())
        world /= 1000.0
        world[2] += 0.038

        if self.verbose:
            print('DROP LOCATION', world)

        # put down block
        self.tp.put_block(world, 0.0)
        
        self.kinect.new_click = False
        self.status_message = "Done pick and place"

    def save_calibration_points(self):
        self.current_state = "save_calibration_points"
        self.next_state = "idle"
        self.kinect.saveCalibrationPoints()
        self.status_message = "Calibration Saved"
        time.sleep(1)    

    def load_previous_calibration(self):
        self.current_state = "load_previous_calibration"
        self.next_state = "idle"
        self.kinect.loadCalibrationPoints()
        self.status_message = "Calibration Loaded"
        time.sleep(1)
    
    def execute(self):
        self.current_state = "execute"
        self.next_state = "idle"
        self.status_message = "Executing plan"
        self.tp.execute_plan(plan=0)
        self.status_message = "Finished waypoints"

    def mirror(self):
        self.current_state = "mirror"
        self.next_state = "idle"
        self.status_message = "Performing Mirror Task"
        
        # variables
        pick_pos = np.zeros(3)
        pick_ori = 0.0
        put_pos = np.zeros(3)
        put_ori = 0.0
        approach_method = ""
        i = 0
        block_picked = 0
        while True:
            if i >= self.kinect.block_num-1:
                i = 0
            # check if block is in the right side
            if self.kinect.block_pos[i][0]<-30 or self.kinect.block_pos[i][0]>30 \
                or self.kinect.block_pos[i][1]<-30 or self.kinect.block_pos[i][1]>30:
                if self.kinect.block_pos[i][0] >= 0:
                    pick_pos = self.kinect.block_pos[i][:]
                    pick_ori = self.kinect.block_ori[i][0]
                    pick_ori *= np.pi/180.0

                    pick_pos[:2] /= 1000.0
                    pick_pos[2] *= 0.038
                    approach_method = "above"
                    # pick up the block
                    self.tp.pick_up_block(pick_pos,pick_ori,approach_method)
                    time.sleep(1)
                    # mirror
                    put_pos[0] = -pick_pos[0]
                    put_pos[1] = pick_pos[1]
                    put_pos[2] = pick_pos[2]
                    print("put_pos", put_pos)
                    put_ori = -pick_ori
                    self.tp.put_block(put_pos,put_ori,approach_method)
                    block_picked+=1

            if block_picked == 3:
                return
            i+=1

                
    def stack_3(self):
        self.current_state = "stack_3"
        self.next_state = "idle"
        self.status_message = "Performing Stack 3 Task"
        
        # variables
        # hard code put_pos to (-210,0) first
        put_pos = np.zeros(3)
        put_pos[1] = 0
        put_pos[0] = -0.21
        put_ori = 0.0
        pick_pos = np.zeros(3)
        pick_ori = 0.0
        d_to_origin = 0.0
        approach_method = ""

        i = 0
        block_picked=0
        while True:
            if i >= self.kinect.block_num-1:
                i = 0
            # check if block is in the right side
            if self.kinect.block_pos[i][0]<-30 or self.kinect.block_pos[i][0]>30 \
                or self.kinect.block_pos[i][1]<-30 or self.kinect.block_pos[i][1]>30:
                if self.kinect.block_pos[i][0] >= 0:
                    pick_pos = copy.copy(self.kinect.block_pos[i][:])
                    pick_ori = copy.copy(self.kinect.block_ori[i][0])
                    pick_ori *= np.pi/180.0
                    self.kinect.block_pos[i] = np.array([0,0,0])
                    self.kinect.block_ori[i][0] = 0
                    pick_pos[:2] /= 1000.0
                    pick_pos[2] *= 0.038

                    approach_method = "above"
                    # decide approach method
                    #d_to_origin = (pick_pos[0]**2+pick_pos[1]**2)**0.5
                    
                    # pick up the block
                    self.tp.pick_up_block(pick_pos,pick_ori,approach_method)
                    # put blocks
                    # set stack height
                    put_pos[2] = (block_picked+1)*0.038
                    self.tp.put_block(put_pos,put_ori,"above")
                    block_picked+=1
                    if block_picked == 1:
                        put_pos[0] -= 0.03
                    elif block_picked == 2:
                        put_pos[0] += 0.03

            if block_picked == 3:
                return
            i+=1
                
    def line_em_up(self):
        self.current_state == "line_em_up"
        self.next_state == "idle"
        self.status_message = "Performing Line 'em up Task"
        
       
        # variables
        pick_pos = np.zeros(3)
        pick_ori = 0.0
        target_color = 1
        put_pos = np.zeros((10,3))
        put_ori = 0.0
        final_pos = np.zeros((8,3))
        """
        put_pos[0] = [-0.096, 0, 0.038]
        put_pos[1] = [-0.126, 0, 0.038]
        put_pos[2] = [-0.16, 0, 0.038]
        put_pos[3] = [-0.20, 0, 0.038]
        put_pos[4] = [-0.24, 0, 0.038]
        put_pos[5] = [-0.28, 0, 0.038]
        put_pos[6] = [-0.32, 0, 0.038]
        put_pos[7] = [-0.096, 0, 0.076]
        put_pos[8] = [-0.126, 0, 0.076]
        """
        put_pos[0] = [-0.180, -0.180, 0.038]
        put_pos[1] =[-0.180, -0.180, 0.038]
        put_pos[2] = [-0.180, -0.180, 0.038]
        put_pos[3] = [-0.180, -0.180, 0.038]
        put_pos[4] = [-0.180, -0.180, 0.038]
        put_pos[5] = [-0.180, -0.180, 0.038]
        put_pos[6] = [-0.180, -0.180, 0.038]
        put_pos[7] = [-0.180, -0.180, 0.038]
        put_pos[8] = [-0.180, -0.180, 0.038]
        final_ori = 0.0
        blocks_left = 8
        current_block = np.zeros(3)
        current_color = 0
        flatten = False
        i = 0
        end = False

        while True:
            current_block[:2] = copy.copy(self.kinect.block_pos[i][:2]/1000.0)
            current_block[2] = copy.copy(self.kinect.block_pos[i][2] * 0.038)
            current_ori = copy.copy(self.kinect.block_ori[i][0]*np.pi/180.0)
            current_color = copy.copy(self.kinect.block_color[i])
            #print(current_block)
            #print("current_color", current_color)
            #print("target_color", target_color)
            # If it loops through blocks on the image, loop again
            if i >= self.kinect.block_num - 1:
                i = 0
                flatten = True
            # find blocks in ROI
            if not(current_block[1]<0.06 and current_block[0]<0.06):
                if current_block[0]<-0.030 or current_block[0]>0.030 \
                    or current_block[1]<-0.030 or current_block[1]>0.030:
                    # check color
                    print("current",current_color)
                    print("target",target_color)
                    if target_color == 4:
                        if current_color == 3:
                            current_color = 4
                    if current_color == target_color:
                        #print("current",current_color)
                        #print("target",target_color)
                        # pick the block
                        pick_pos = current_block
                        pick_ori = current_ori
                        self.tp.pick_up_block(pick_pos,pick_ori,"above")
                        
                        #print(final_pos)
                        #print("put_pos", put_pos)
                        put_ori = final_ori
                        self.tp.put_block(put_pos[8-blocks_left],put_ori,"above")
                        
                        target_color += 1
                        blocks_left -= 1
                        flatten = False

                    # if loop through all, flatten high towers
                    elif flatten == True:
                        if current_block[1] > 0.06:
                            pick_pos = current_block
                            pick_ori = current_ori
                            self.tp.pick_up_block(pick_pos,pick_ori,"above")
                            #put_pos= self.kinect.find_empty_space()
                            
                            flat_pos = np.array([150.0,-150.0,1.0])
                            flat_pos[:2] /= 1000.0
                            flat_pos[2] *= 0.038
                            put_ori = 0
                            #print(flat_pos)
                            self.tp.put_block(flat_pos,put_ori,"above")
                            i=0
                    """
                    elif flatten == True:
                
                        current_block_id = np.where(self.kinect.block_pos[:][2] > 1.0)
                        print(current_block_id)
                        if len(current_block_id[0]) > 0:
                            current_block_id = current_block_id[0]
                            current_block[:2] = copy.copy(self.kinect.block_pos[current_block_id][:2]/1000.0)
                            current_block[2] = copy.copy(self.kinect.block_pos[current_block_id][2] * 0.038)
                            current_ori = copy.copy(self.kinect.block_ori[current_block_id][0]*np.pi/180.0)
                            current_color = copy.copy(self.kinect.block_color[current_block_id])

                            if current_block[1]>=0.06:
                                pick_pos = current_block
                                pick_ori = current_ori
                                self.tp.pick_up_block(pick_pos,pick_ori,"above")
                                #put_pos= self.kinect.find_empty_space()
                                
                                flat_pos = np.array([150.0,-150.0,1.0])
                                flat_pos[:2] /= 1000.0
                                flat_pos[2] *= 0.038
                                put_ori = 0
                                print(flat_pos)
                                self.tp.put_block(flat_pos,put_ori,"above")
                                i=0
                        else:
                            if not(current_block[1]<0.06 and current_block[0]<0.06):
                                pick_pos = current_block
                                pick_ori = current_ori
                                self.tp.pick_up_block(pick_pos,pick_ori,"above")
                        
                                #print(final_pos)
                                #print("put_pos", put_pos)
                                put_ori = final_ori
                                self.tp.put_block(put_pos[8-blocks_left],put_ori,"above")
                                target_color += 1
                                blocks_left -= 1
                                flatten = False
                        """

            # check if all blocks are picked
            if blocks_left == 0:
                return
            i+=1


    def stack_em_high(self):
        self.current_state == "stack_em_high"
        self.next_state == "idle"
        self.status_message = "Performing Stack 'em High Task"
        
        # variables
        pick_pos = np.zeros(3)
        pick_ori = 0.0
        target_color = 1
        put_pos = np.zeros(3)
        put_ori = 0.0
        final_pos = np.array([-210,0,1])
        
        final_ori = 0.0
        blocks_left = 8
        current_block = np.zeros(3)
        current_color = 0
        flatten = False
        i = 0
        end = False
        while True:

            current_block[:2] = copy.copy(self.kinect.block_pos[i][:2]/1000.0)
            current_block[2] = copy.copy(self.kinect.block_pos[i][2] * 0.038)
            current_ori = copy.copy(self.kinect.block_ori[i][0]*np.pi/180.0)
            current_color = copy.copy(self.kinect.block_color[i])
            print(current_block)
            #print("current_color", current_color)
            #print("target_color", target_color)
            # If it loops through blocks on the image, loop again
            if i >= self.kinect.block_num - 1:
                i = 0
                flatten = True
            # find blocks in ROI
            if not(current_block[1]<0.06 and current_block[0]<0.06):
                if current_block[0]<-0.030 or current_block[0]>0.030 \
                    or current_block[1]<-0.030 or current_block[1]>0.030:
                    # check color
                    if target_color == 4:
                        if current_color == 3:
                            current_color = 4
                    if current_color == target_color:
                        print("current",current_color)
                        print("target",target_color)
                        # pick the block
                        pick_pos = current_block
                        pick_ori = current_ori
                        self.tp.pick_up_block(pick_pos,pick_ori,"above")
                        put_pos[:2] = final_pos[:2]/1000.0
                        put_pos[2] = final_pos[2]*0.038
                        #print(final_pos)
                        #print("put_pos", put_pos)
                        put_ori = final_ori
                        self.tp.put_block(put_pos,put_ori,"above")
                        final_pos[2] += 1 
                        blocks_left -= 1
                        target_color += 1
                        flatten = False
                    # if loop through all, flatten high towers
                    

                    elif flatten == True:
                        if current_block[1] > 0.06:
                            pick_pos = current_block
                            pick_ori = current_ori
                            self.tp.pick_up_block(pick_pos,pick_ori,"above")
                            #put_pos= self.kinect.find_empty_space()
                            
                            flat_pos = np.array([150.0,-150.0,1.0])
                            flat_pos[:2] /= 1000.0
                            flat_pos[2] *= 0.038
                            put_ori = 0
                            #print(flat_pos)
                            self.tp.put_block(flat_pos,put_ori,"above")
                            i=0
            # check if all blocks are picked
            if blocks_left == 0:
                return
            i+=1



    def pyramid(self):

        self.current_state == "pyramid"
        self.next_state == "idle"
        self.status_message = "Performing Pyramid Task"
        approach_from = "left"
        pick_pos = np.zeros(3)
        pick_ori = 0
        block_found = False

        #gap between blocks
        put_block_gap = 0.004
        #block length
        block_length = 0.038
        # x of the beginning side of the pyramid
        start_point = ((put_block_gap+block_length)*5+block_length)*0.5

        put_pos = np.zeros(3)
        put_pos[1] = -0.12
        put_ori = 0

        i = 0
    # plan a 6-block high pyramid, width = height = 6 blocks
        for width in range(1, 7):
            for layer in range(1, width+1):
                block_found = False
                i=0
                while not block_found:
                    if self.kinect.block_pos[i][0]<-30 or self.kinect.block_pos[i][0]>30 \
                    or self.kinect.block_pos[i][1]<-30 or self.kinect.block_pos[i][1]>30:
                        if (self.kinect.block_pos[i][1] > 0):
                            pick_pos[:2] = self.kinect.block_pos[i][:2]/1000.0
                            pick_pos[2] = block_length * self.kinect.block_pos[i][2]
                            block_found = True
                        i = i+1
                        if i >= self.kinect.block_num-1:
                            i = 0

                print('PICK POS', pick_pos)
                i = 0
                self.tp.pick_up_block(pick_pos,put_ori,'above')

                put_pos[0] = start_point-block_length/2-(layer-1)*(block_length+put_block_gap)
                put_pos[2] = layer*block_length
                self.tp.put_block(put_pos,put_ori,approach_from)
                time.sleep(1)
                

    def pyramid2(self):
        self.current_state = "pyramid2"
        self.next_state = "idle"
        # variables
        pick_pos = np.zeros(3)
        pick_ori = 0.0
        put_pos = np.zeros((3,3))
        put_ori = 0.0
        approach_method = "above"
        block_picked = 0
        i = 0
        put_pos[0] = [-0.096, 0, 0.038]
        put_pos[1] = [-0.134, 0, 0.038]
        put_pos[2] = [-0.11, 0, 0.076]
        num = 0

        while True:
            if i >= self.kinect.block_num-1:
                i = 0
            if num == 3:
                return
            # check if block is in the right side
            if self.kinect.block_pos[i][0]<-30 or self.kinect.block_pos[i][0]>30 \
                or self.kinect.block_pos[i][1]<-30 or self.kinect.block_pos[i][1]>30:
                if self.kinect.block_pos[i][0] >= 0:
                    pick_pos = self.kinect.block_pos[i][:]
                    pick_ori = self.kinect.block_ori[i][0]
                    pick_ori *= np.pi/180.0

                    pick_pos[:2] /= 1000.0
                    pick_pos[2] *= 0.038
                    # pick up the block
                    self.tp.pick_up_block(pick_pos,pick_ori,approach_method)
                    time.sleep(1)
                    # mirror

                    print("put_pos", put_pos[num])
                    self.tp.put_block(put_pos[num],put_ori,approach_method)
                    num+=1
            i+=1

    def pyramid3(self):
        self.current_state = "pyramid3"
        self.next_state = "idle"
        # variables
        pick_pos = np.zeros(3)
        pick_ori = 0.0
        put_pos = np.zeros((6,3))
        put_ori = 0.0
        approach_method = "above"
        i = 0
        put_pos[0] = [-0.096, 0, 0.038]
        put_pos[1] = [-0.126, 0, 0.038]
        put_pos[2] = [-0.16, 0, 0.038]
        put_pos[3] = [-0.104, 0, 0.076]
        put_pos[4] = [-0.148, 0, 0.076]
        put_pos[5] = [-0.126, 0, 0.114]
        num = 0
        while True:
            if i >= self.kinect.block_num-1:
                i = 0
            if num == 6:
                return
            # check if block is in the right side
            if self.kinect.block_pos[i][0]<-30 or self.kinect.block_pos[i][0]>30 \
                or self.kinect.block_pos[i][1]<-30 or self.kinect.block_pos[i][1]>30:
                if self.kinect.block_pos[i][0] >= 0:
                    pick_pos = self.kinect.block_pos[i][:]
                    pick_ori = self.kinect.block_ori[i][0]
                    pick_ori *= np.pi/180.0

                    pick_pos[:2] /= 1000.0
                    pick_pos[2] *= 0.038
                    # pick up the block
                    self.tp.pick_up_block(pick_pos,pick_ori,approach_method)
                    time.sleep(1)
                    # mirror

                    print("put_pos", put_pos[num])
                    self.tp.put_block(put_pos[num],put_ori,approach_method)
                    num+=1
            i+=1

    def pyramid4(self):
        self.current_state = "pyramid3"
        self.next_state = "idle"
        # variables
        pick_pos = np.zeros(3)
        pick_ori = 0.0
        put_pos = np.zeros((10,3))
        put_ori = 0.0
        approach_method = "above"
        i = 0
        put_pos[0] = [-0.096, 0, 0.038]
        put_pos[1] = [-0.126, 0, 0.038]
        put_pos[2] = [-0.16, 0, 0.038]
        put_pos[3] = [-0.20, 0, 0.038]
        put_pos[4] = [-0.104, 0, 0.076]
        put_pos[5] = [-0.148, 0, 0.076]
        put_pos[6] = [-0.175, 0, 0.076]
        put_pos[7] = [-0.112, 0, 0.114]
        put_pos[8] = [-0.152, 0, 0.114]
        put_pos[9] = [-0.134, 0, 0.152]
        num = 0
        while True:
            if i >= self.kinect.block_num-1:
                i = 0
            if num == 10:
                return
            # check if block is in the right side
            if self.kinect.block_pos[i][0]<-30 or self.kinect.block_pos[i][0]>30 \
                or self.kinect.block_pos[i][1]<-30 or self.kinect.block_pos[i][1]>30:
                if self.kinect.block_pos[i][0] >= 60:
                    pick_pos = self.kinect.block_pos[i][:]
                    pick_ori = self.kinect.block_ori[i][0]
                    pick_ori *= np.pi/180.0

                    pick_pos[:2] /= 1000.0
                    pick_pos[2] *= 0.038
                    # pick up the block
                    self.tp.pick_up_block(pick_pos,pick_ori,approach_method)
                    time.sleep(1)
                    # mirror

                    print("put_pos", put_pos[num])
                    self.tp.put_block(put_pos[num],put_ori,approach_method)
                    num+=1
            i+=1

    def pyramid5(self):
        self.current_state = "pyramid3"
        self.next_state = "idle"
        # variables
        pick_pos = np.zeros(3)
        pick_ori = 0.0
        put_pos = np.zeros((15,3))
        put_ori = 0.0
        approach_method = "above"
        i = 0
        put_pos[0] = [-0.096, 0, 0.038]
        put_pos[1] = [-0.126, 0, 0.038]
        put_pos[2] = [-0.16, 0, 0.038]
        put_pos[3] = [-0.20, 0, 0.038]
        put_pos[4] = [-0.24, 0, 0.038]

        put_pos[5] = [-0.094, 0, 0.076]
        put_pos[6] = [-0.125, 0, 0.076]
        put_pos[7] = [-0.160, 0, 0.076]
        put_pos[8] = [-0.210, 0 , 0.076]

        put_pos[9] = [-0.098, 0, 0.114]
        put_pos[10] = [-0.135, 0, 0.114]
        put_pos[11] = [-0.173, 0, 0.114]
        put_pos[12] = [-0.115, 0, 0.152]
        put_pos[13] = [-0.149, 0, 0.152]
        put_pos[14] = [-0.133, 0, 0.190]
        num = 0
        while True:
            if i >= self.kinect.block_num-1:
                i = 0
            if num == 15:
                return
            # check if block is in the right side
            if self.kinect.block_pos[i][0]<-30 or self.kinect.block_pos[i][0]>30 \
                or self.kinect.block_pos[i][1]<-30 or self.kinect.block_pos[i][1]>30:
                if self.kinect.block_pos[i][0] >= 60:
                    pick_pos = self.kinect.block_pos[i][:]
                    pick_ori = self.kinect.block_ori[i][0]
                    pick_ori *= np.pi/180.0

                    pick_pos[:2] /= 1000.0
                    pick_pos[2] *= 0.038
                    # pick up the block
                    self.tp.pick_up_block(pick_pos,pick_ori,approach_method)
                    time.sleep(1)
                    # mirror

                    print("put_pos", put_pos[num])
                    self.tp.put_block(put_pos[num],put_ori,approach_method)
                    num+=1
            i+=1

    def record_block_position(self):
        self.current_state = "record_block_position"
        self.next_state = "idle"

        self.kinect.record_block_position()
        self.status_message = "Finished recording"