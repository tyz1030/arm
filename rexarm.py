#rexarm.py
import numpy as np
import kinematics
import time
import threading


""" 
TODO:

Implement the missing functions
add anything you see fit

"""

""" Radians to/from  Degrees conversions """
D2R = 3.141592/180.0
R2D = 180.0/3.141592

class Rexarm():
    def __init__(self, joints, gripper):
        self.initialized = False
        self.joints = joints
        self.gripper = gripper
        self.gripper_open_pos = np.deg2rad(-90.0)
        self.gripper_closed_pos = np.deg2rad(90.0)
        self.gripper_state = True
        self.estop = False
        self.kill = False
        """TODO: Find the physical angle limits of the Rexarm"""
        
        self.angle_limits = np.array([[-180, 179.99],
                            [-100, 100],
                            [-120, 120],
                            [-120, 120],
                            [-120, 120]], dtype=np.float)*D2R


        """ Commanded Values """
        self.num_joints = len(joints)

        self.position = [0.0] * self.num_joints     # degrees
        self.wrist_offset = 0.0/180*np.pi 

        self.gripper_position = 0.0
        self.speed = [0.05] * self.num_joints        # 0 to 1
        self.max_torque = [0.6] * self.num_joints   # 0 to 1

        """ Feedback Values """
        self.joint_angles_fb = [0.0] * self.num_joints # degrees

        self.speed_fb = [0.0] * self.num_joints        # 0 to 1   
        self.load_fb = [0.0] * self.num_joints         # -1 to 1  
        self.temp_fb = [0.0] * self.num_joints         # Celsius
        self.move_fb = [0] *  self.num_joints

        # Link links
        self.link_lengths = np.array([118, 100, 100, 50, 75]) / 1000.0

        # Set FK
        self.FK = 'dh'
        self.recording = False
        self.record = False

    def initialize(self):
        for joint, position in zip(self.joints, self.position):
            joint.enable_torque()
            joint.set_torque_limit(0.3)
            joint.set_speed(0.1)
            joint.set_position(position)

        if(self.gripper != 0):
            self.gripper.set_torque_limit(1.0)
            self.gripper.set_speed(0.8)
            self.open_gripper()
        
        # start background comms thread
        self.bg_thread = threading.Thread(target=self.communication_thread, args=[])
        self.bg_thread.start()

    def open_gripper(self):
        self.gripper_position = self.gripper_open_pos
        self.gripper_state = False

    def close_gripper(self):
        self.gripper_position = self.gripper_closed_pos
        self.gripper_state = True

    def set_gripper_rotate(self, initdeg):
        self.position[5] = initdeg

    def set_positions(self, joint_angles, update_now = True):
        joint_angles = self.clamp(joint_angles)
        for i,joint in enumerate(self.joints):
            self.position[i] = joint_angles[i]
            if(update_now):
                if i == 4:
                    joint.set_position(joint_angles[i]+self.wrist_offset)
                else:
                    joint.set_position(joint_angles[i])

    def set_gripper_positions(self, gripper_angle, update_now = True):
        if(update_now):
            self.gripper.set_position(gripper_angle)

    def set_speeds_normalized_global(self, speed, update_now = True):
        for i,joint in enumerate(self.joints):
            self.speed[i] = speed
            if(update_now):
                joint.set_speed(speed)

    def set_speeds_normalized(self, speeds, update_now = True):
        for i,joint in enumerate(self.joints):
            self.speed[i] = speeds[i]
            if(update_now):
                joint.set_speed(speeds[i])

    def set_speeds(self, speeds, update_now = True):
        for i,joint in enumerate(self.joints):
            self.speed[i] = speeds[i]
            speed_msg = abs(speeds[i]/joint.max_speed)
            if (speed_msg < 3.0/1023.0):
                speed_msg = 3.0/1023.0
            if(update_now):
                joint.set_speed(speed_msg)
    
    def set_torque_limits(self, torques, update_now = True):
        for i,joint in enumerate(self.joints):
            self.max_torque[i] = torques[i]
            if(update_now):
                joint.set_torque_limit(torques[i])

    def send_commands(self):
        self.set_positions(self.position)
        self.set_speeds_normalized(self.speed)
        self.set_torque_limits(self.max_torque)
        self.set_gripper_positions(self.gripper_position)
   
    def enable_torque(self):
        for joint in self.joints:
            joint.enable_torque()

    def disable_torque(self):
        for joint in self.joints:
            joint.disable_torque()

    def get_positions(self):
        tmp = [0.0]*5
        for i,joint in enumerate(self.joints):
            tmp[i] = joint.get_position()
        
        if not None in tmp:
            self.joint_angles_fb = tmp
            self.joint_angles_fb[4] -= self.wrist_offset

        return self.joint_angles_fb

    def get_speeds(self):
        tmp = [0]*5
        for i,joint in enumerate(self.joints):
            tmp[i]= joint.get_speed()

        if not None in tmp:
            self.speed_fb = tmp

        return self.speed_fb

    def get_loads(self):
        tmp = [0]*5
        for i,joint in enumerate(self.joints):
            tmp[i]= joint.get_load()

        if not None in tmp:
            self.load_fb = tmp

        return self.load_fb
  
    def get_gripper(self):
        load = self.gripper.get_load()
        pos = self.gripper.get_position()
        return load,pos

    def get_feedback(self):
        self.get_positions()
        self.get_speeds()
        #self.get_loads()
        #self.get_temps()
        #self.get_moving_status()
        print(self.get_gripper())

    def pause(self, secs):
        time_start = time.time()
        while((time.time()-time_start) < secs):
            self.get_feedback()
            time.sleep(0.05)
            if(self.estop == True):
                break

    def clamp(self, joint_angles):
        """TODO"""
        min_angles = self.angle_limits[:,0]
        max_angles = self.angle_limits[:,1]
        joint_angles = np.clip(joint_angles, min_angles, max_angles)
        return joint_angles

    def get_wrist_pose(self):
        if self.FK == 'pox':
            (x, y, z, phi) = kinematics.FK_pox(self)
        elif self.FK == 'dh':
            (x, y, z, phi) = kinematics.FK_dh(self)

        return [-x,y,z,phi]

    def communication_thread(self):
        ''' Background comms thread'''
        while not self.kill:
            try:
                self.send_commands()
                self.get_feedback()
            except:
                pass
            
            if self.recording:
                self.data.append(self.get_wrist_pose())
                
                if not self.record:
                    self.save(self.data)
                    self.recording=False
            else:
                if self.record:
                    self.recording=True
                    self.data = []
                    
            time.sleep(0.01)
    
    def save(self, data):
        np_data = np.asarray(data)
        np.save('end_eff_pose_slow.npy', np_data)

    def shutdown(self):
        self.kill = True
