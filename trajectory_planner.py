import numpy as np 
import time
import matplotlib.pyplot as plt
import copy
import kinematics

class TrajectoryPlanner():
    def __init__(self, rexarm):
        self.idle = True
        self.rexarm = rexarm
        self.num_joints = rexarm.num_joints
        self.initial_wp = [0.0]*self.num_joints
        self.final_wp = [0.0]*self.num_joints 
        self.waypoints = [self.initial_wp]

        self.dt = 0.05 # command rate
        self.motor_rads_per_sec = np.pi/ 3.0

        self.verbose = False
        # default waypoints from the tutorial
        self.default_waypoints = [[0.0, 0.0, 0.0, 0.0, 0.0],
                                [1.0, 0.8, 1.0, 1.0, 0.0],
                                [-1.0, -0.8, -1.0, -1.0, 0.0],
                                [-1.0, 0.8, 1.0, 1.0, 0.0],
                                [1.0, -0.8, -1.0, -1.0, 0.0],
                                [0.0, 0.0, 0.0, 0.0, 0.0]]

        self.mask_base = np.array([1, 0, 0, 0, 0])
        
        self.record = True
 
    def add_waypoint(self, waypoint):
        self.waypoints.append(waypoint)

    def clear_waypoints(self):
        self.waypoints = [self.initial_wp]

    def stop(self):
        pass

    def generate_cubic_spline(self, initial_wp, final_wp, T):

        initial_wp = np.asarray(initial_wp)
        final_wp = np.asarray(final_wp)

        b = np.array([initial_wp, [0]*5, final_wp, [0]*5], dtype='float')

        M = np.array([[1, 0, 0, 0], 
                        [0, 1, 0, 0], 
                        [1, T, T**2, T**3],
                        [0, 1, 2*T, 3 * T**2]],dtype='float')
        
        #a = np.matmul(np.linalg.inv(M), b)
        
        a = np.linalg.solve(M, b)

        times = np.arange(0, T+self.dt, self.dt)
        output_qs = []
        output_vs = []
        for t in times:
            qf = np.matmul(np.asarray([1, t, t**2, t**3]).T, a)
            vf = np.matmul(np.asarray([0, 1, 2*t, 3*t**2]).T, a)
            output_qs.append(qf)
            output_vs.append(vf)


        #output_vs[-1] = output_vs[-2]
        return output_qs, output_vs

    def round_up(self, x, a):
        return round(np.ceil(x / a) * a, 2)

    def plan_profile(self, waypoints, speed=None):
        if speed is None:
            speed = self.motor_rads_per_sec
        
        # total wps
        total_wp = []
        total_wpv = []
        
        # generate cubic spline
        for i in range(1, len(waypoints)):
               
            start = time.time()

            # How far do we need to move? 
            last_wp = np.asarray(waypoints[i-1])
            wp = np.asarray(waypoints[i])
    
            # what is our max distance
            dist = np.max(np.abs(wp - last_wp))
                    
            # How long do we want it to take us?
            T = dist / speed
            
            # round up to nearest 0.05
            T = self.round_up(T, self.dt)
           
            interim_wp, interim_v = self.generate_cubic_spline(last_wp, wp, T)
            
            total_wp.extend(interim_wp)
            total_wpv.extend(interim_v)

            # 0.1 second pause at end of waypoint
            #for j in range(0, 2):
            #    total_wp.append(interim_wp[-1])
            #    total_wpv.append(interim_v[-1])
            
        return total_wp, total_wpv

    def smooth_route(self, end, go_to_origin=True, base_first=True, speed=None):
        ''' Smooth route from start to end '''
        start = copy.copy(self.rexarm.joint_angles_fb)
        
        if np.sum(np.abs(start - end)) < 0.1:
            return

        if go_to_origin:
            # If we go to origin, dont bother moving the base
            zero = np.zeros(5)
            end_non_base = (1-self.mask_base) * zero + start * self.mask_base
            self.do_route(start, end_non_base, speed)
            start = end_non_base

        if base_first:
            # do base first
            end_base = self.mask_base * end + (1-self.mask_base) * start

            self.do_route(start, end_base, speed)
            start = end_base
        self.do_route(start, end, speed)


    def do_route(self, start, end, speed=None):
        if speed is None:
            speed = self.motor_rads_per_sec

        if (start==end).all():
            return

        total_wp, total_wpv = self.plan_profile([start, end], speed)
        look_ahead = 8
        # add look ahead
        for i in range(0, look_ahead):
            total_wp.append(total_wp[-1])

        # set waypoints
        for i in range(0, len(total_wpv)):
            start = time.time()
            self.rexarm.set_speeds(total_wpv[i], update_now=False)
            self.rexarm.set_positions(total_wp[i+look_ahead], update_now=False)

            end = time.time()
            if end - start > 0:
                time.sleep(self.dt - (end-start))
            start = time.time()

    
    def get_IK_solution(self, pos, omega_start, phi, ori, offset=True):
        omega = omega_start
        while True:
            try:
                theta = kinematics.IK(self.rexarm, pos, omega, phi=phi, ori=ori, offset=offset)
                return theta
            except:
                pass

            if np.abs(omega_start - omega) > np.pi/2.0 :
                raise ValueError('no IK solution')
            elif omega_start < -np.pi/4.0:
                omega += 1*np.pi/180
            else:
                omega -= 1*np.pi/180


    def move_to_pos(self, pos, ori, approach_from=None):
        ''' 
        
        Function to move to position
        moves most of the way first, and then carefully approaches slowly

        We find a few angles before calling IK, as the IK relies on some angles
        being pre-specified. These are namely:
            phi     - the angle of rotation of the wrist along end effector z
            omega   - the angle between the last link of the arm and the xy plane
            base    - we precalculate this to help find our interim target
        '''

        if self.verbose:
            print('MOVING TO POS...')
            print('location', pos)

        pos = pos.astype('float')
        [l1, l2, l3, l4, l5] = self.rexarm.link_lengths

        # How far away is pos
        d_from_origin = (pos[0]**2 + pos[1]**2 + (pos[2]-l1)**2)**0.5
        phi = 0.0

        # Find some initial angles
        base_angle = kinematics.find_base_angle(pos)
        omega = self.get_omega(pos)

        # Find interim target position
        pos_interim = pos.copy()
        if omega < -45*np.pi/180:
            d = 0.02 * np.array([np.sin(base_angle), np.cos(base_angle), 0])
            d[2] += 0.04
        else:
            d = 0.05 * np.array([-np.sin(base_angle), -np.cos(base_angle), 1.0])
            pos_interim += d

        if self.verbose: 
            print('POS INTERIM', pos_interim)

        # move to interim target
        theta = self.get_IK_solution(pos_interim, omega, phi=phi, ori=ori)
        self.smooth_route(theta)

        time.sleep(0.5)

        # move to final target
        theta = self.get_IK_solution(pos, omega, phi=phi, ori=ori)
        self.smooth_route(theta, False, False, np.pi/6.0)

    def get_left_unit_vector(self, pos, phi):
        if pos[0] > 0:
            if pos[1] > 0:
                dxy = np.array([-np.cos(phi), np.sin(phi), 0.0])
            else:
                dxy = np.array([np.cos(phi), np.sin(phi), 0.0])
        else:
            if pos[1] > 0:
                dxy = np.array([-np.cos(phi), -np.sin(phi), 0.0])
            else:
                dxy = np.array([np.cos(phi), -np.sin(phi), 0.0])
            
        return dxy

    def get_omega(self, pos):
        [l1, l2, l3, l4, l5] = self.rexarm.link_lengths
        d_from_origin = (pos[0]**2 + pos[1]**2 +(pos[2]-l1)**2)**0.5

        if d_from_origin < l2+l1:
            omega = -120 * np.pi / 180.0
        else:
            omega = -15 * np.pi/180
      
        return omega

    def back_away_from_pos(self, pos, leave_from=None):
        ''' 
            Carefully backs away from a position
        '''
        
        if self.verbose:
            print('BACKING AWAY...')

        # get starting angles
        start = copy.copy(self.rexarm.joint_angles_fb)
        [l1, l2, l3, l4, l5] = self.rexarm.link_lengths
        phi = start[4]
        base = start[0]
        ori = base-phi
        omega = self.get_omega(pos)

        # Get target pos
        if omega < -45 * np.pi/180:
            unit_vector_away = np.array([np.sin(base), np.cos(base), 1.0])
        else:
            unit_vector_away = np.array([-np.sin(base), -np.cos(base), 1.0])

        pos += 0.03*unit_vector_away
        base_new = kinematics.find_base_angle(pos)

        # Find IK solution and move away
        theta = self.get_IK_solution(pos, omega, phi=phi, offset=False, ori=ori)
        self.smooth_route(theta, False, False, np.pi/6.0)

        time.sleep(1.0)

    def readjust_grip(self):
        ''' 
        Raises arm, and bends arm backwards to allow for grip to be adjusted
        
        '''
        # get start theta
        start = copy.copy(self.rexarm.joint_angles_fb)

        # get end theta
        end = np.zeros(5)
        end[0] = start[0]
        end[3] = np.pi/6.0

        # Do readjust
        self.smooth_route(end, False, False)
        time.sleep(1.0)
        self.rexarm.open_gripper()
        time.sleep(1.0)
        self.rexarm.close_gripper()
        time.sleep(0.1)
        
    def execute_plan(self, plan, smooth=True, look_ahead=8):
      
        smooth = False
        # define teach and repeat as plan 0
        if not plan:

            if len(self.waypoints) == 1:
                    waypoints = self.default_waypoints
            else:
                waypoints = self.waypoints

            if self.record:
                self.rexarm.record = True

            if not smooth:
                for waypoint in waypoints:
                    self.rexarm.set_positions(waypoint, update_now=False)
                    time.sleep(2.0)
            else:
                total_wp, total_wpv = self.plan_profile(waypoints)

                # add look ahead
                for i in range(0, look_ahead):
                    total_wp.append(total_wp[-1])

                # set waypoints
                for i in range(0, len(total_wpv)):
                    start = time.time()
                    self.rexarm.set_speeds(total_wpv[i], update_now=False)
                    self.rexarm.set_positions(total_wp[i+look_ahead], update_now=False)

                    end = time.time()
                    if end - start > 0:
                        time.sleep(self.dt - (end-start))
                    start = time.time()

            self.rexarm.record = False

    def pick_up_block(self,pos,ori,approach_from=None):
        block_pos = copy.copy(pos)
        #block_pos[2] -= 0.01
        #open girpper
        self.rexarm.open_gripper()
        #move
        self.move_to_pos(block_pos,ori,approach_from)
        time.sleep(0.5)
        #close gripper
        self.rexarm.close_gripper()
        time.sleep(0.5)
        #readjust
        self.readjust_grip()
        time.sleep(0.5)
        

    def put_block(self,pos,ori,approach_from=None):

        #move
        self.move_to_pos(pos,ori,approach_from)
        time.sleep(0.5)
        #open_gripper
        self.rexarm.open_gripper()
        time.sleep(0.5)
        #back
        self.back_away_from_pos(pos,approach_from)
        time.sleep(0.5)
    

