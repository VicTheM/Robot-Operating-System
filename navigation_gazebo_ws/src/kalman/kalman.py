
import numpy as np
from numpy.random import randn

import copy

import scipy.stats as stats
from scipy.linalg import block_diag

from math import tan, sin, cos, sqrt
from math import cos, sin, atan2, pi

import matplotlib.pyplot as plt

from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import JulierSigmaPoints
from filterpy.kalman import unscented_transform, MerweScaledSigmaPoints
from filterpy.stats import plot_covariance_ellipse

class KalmanNav:
    def __init__(self, landmarks, dictSensors, initPos,
        sigma_gps = 10.0,

        sigma_imu_accel = 0.01,
        # sigma_imu_accel_bias = 0.001,

        sigma_imu_gyro = 0.1,
        # sigma_imu_gyro_bias = 0.1,

        sigma_imu_magnet = 0.3,

        sigma_range=1, 
        sigma_bearing=0.3,
        
        dt=0.1,
        gps_step=1,
        imu_step=1,
        mag_step=10):

        # TBD: currently, hardcoded and must be same as in .SDF (though data from sdf not used, but it is nice to 
        # use them at some point)
        self.world_magnet = np.array([0., 2.3e-05, 0.])

        # Sensor data:
        #
        # gps: x,y
        #
        # imu: orientation (roll, pitch, yaw), angular velocity (vx, vy, vz), linear acceleration (ax, ay, az)
        # imu currently used: orientation (yaw), angular velocity (vz), linear acceleration (ax, ay)
        #
        # landmarks: data in format [dist_1, bearing_1, dist_2, bearing_2,...]
        #
        # Here we list sensor name (like "gps") and number of data elements 
        # it returns (in case of GPS, it is 2: x and y)
        self.dictSensors = dictSensors
        self.landmarks = landmarks
        
        self.sigma_gps = sigma_gps
        self.sigma_imu_accel = sigma_imu_accel
        # self.sigma_imu_accel_bias = sigma_imu_accel_bias

        self.sigma_imu_gyro = sigma_imu_gyro
        # self.sigma_imu_gyro_bias = sigma_imu_gyro_bias

        self.sigma_imu_magnet = sigma_imu_magnet

        self.sigma_range = sigma_range 
        self.sigma_bearing = sigma_bearing

        self.dt = dt
        self.gps_step = gps_step
        self.imu_step = imu_step
        self.mag_step = mag_step

        # As robot moves, some landmarks become obscurred. The following is a list
        # of boolean values for visibility of each landmark
        self.arrShowLandmarks = []

        # Stored predictions from the prev. step. They are used if particular landmark
        # is not visible at this step, so we have to skip prediction for it, using
        # old value.
        self.arr_npPrediction = None

        # Stored values from the previous step
        # self.dictState = {}
        # self.dictState["gyro_drift_vz"] = 0
        # self.dictState["accel_drift_vx"] = 0
        # self.dictState["accel_drift_vy"] = 0

        # Robot's state: coordinates and orientation:
        # x = [x, y, theta, vx, vy, vTheta, ax, ay]T #, gyro_bias_z, accel_bias_vx, accel_bias_vy]T
        self.nNumOfStateParams = 8
        
        self.points = MerweScaledSigmaPoints(n=self.nNumOfStateParams, alpha=.00001, beta=2, kappa=0, subtract=self.residual_x)

        dim_z = 0
        for val in self.dictSensors.values():
            dim_z += val

        self.ukf = UKF(dim_x=self.nNumOfStateParams, dim_z=dim_z,
            fx=self.move, hx=self.Hx, dt=self.dt, points=self.points, x_mean_fn=self.state_mean,
            z_mean_fn=self.z_mean, residual_x=self.residual_x, residual_z=self.residual_h)
        self.ukf.x = np.array(initPos)

        # initial uncertainties (variances) in the robot's state vector
        self.ukf.P = np.diag([1., 1., 1., 1., 1., 1., 1., 1.])
        
        # measurement noise, dimensions (dim_z x dim_z)
        self.arrSigmas = []
        if("gps" in self.dictSensors):
            self.arrSigmas.extend([self.sigma_gps**2, self.sigma_gps**2])

        if("imu_accel" in self.dictSensors):
            self.arrSigmas.extend([self.sigma_imu_accel**2, self.sigma_imu_accel**2])

        if("imu_gyro" in self.dictSensors):
            # currently, just yaw, NOT quternion, as 2d world
            self.arrSigmas.extend([self.sigma_imu_gyro**2])

        if("imu_magnet" in self.dictSensors):
            # currently, just yaw, NOT quternion, as 2d world
            self.arrSigmas.extend([self.sigma_imu_magnet**2])            

        # Note: these are initial values. I will adjust them in a cycle below
        # as robot moves and distance to landmarks change, which means sigmas
        # for landmark distance change as well
        if("landmarks" in self.dictSensors):
            for lmark in self.landmarks:
                distance_adjusted_sigma_range = self.sigma_range * sqrt(lmark[0]**2 + lmark[1]**2)
                self.arrSigmas.extend([distance_adjusted_sigma_range**2, self.sigma_bearing**2])
        
        self.ukf.R = np.diag(self.arrSigmas)
        
        # process noise covariance matrix for the state vector
        # x, y, theta, vx, vy, vTheta, ax, ay, gyro_drift_vz
        diagonal_values = np.array([0.001, 0.001, 0.001, 0.00001, 0.00001, 0.001, 0.01, 0.01])
        #diagonal_values = np.array([0.0025, 0.0025, 0.01, 0.001, 0.001, 0.01, 0.2, 0.2])
        # Create a diagonal matrix with these values
        self.ukf.Q = np.diag(diagonal_values)

        self.sim_pos = self.ukf.x.copy()

        # Kalman filter cycle counter. "move()" can be called by us in Kalman 
        # cycle, but also by Kalman itself. In first case we need to adjust
        # values (speed, acceleration) we keep, in second, we don't. To distinguish,
        # we use cycle counters.
        self.nPrevKalmanStep = 0
        self.nCurrentKalmanStep = 0

    # From state and command, calculate next state
    def getAccelerations(self, u, x, dt):
        vel = u[0]
        angularVelZ = u[1]
                            
        heading = self.normalize_angle(x[2] + angularVelZ * dt)

        # --- calculate imu accelerations

        vx = vel * cos(heading)
        vy = vel * sin(heading)

        # ---

        ax = (vx - x[3])/dt
        ay = (vy - x[4])/dt

        #dist = vel * dt
        xPos = x[0] + vx*dt
        yPos = x[1] + vy*dt
        # xPos = x[0] + x[3]*dt + 0.5*ax*dt**2
        # yPos = x[1] + x[4]*dt + 0.5*ay*dt**2

        # gyro_bias_vz = self.dictState["gyro_drift_vz"]
        # accel_bias_vx = self.dictState["accel_drift_vx"]
        # accel_bias_vy = self.dictState["accel_drift_vy"]

        #print(f"{vel:.4f}, {angularVelZ:.4f}, {xPos:.4f}, {yPos:.4f}, {np.degrees(heading):.2f}, {ax:.5f}, {ay:.5f}", flush=True)

        return xPos, yPos, heading, vx, vy, angularVelZ, ax, ay #, gyro_bias_vz, accel_bias_vx, accel_bias_vy

    # --- Model of robot movement

    # control input u is the commanded velocity and steering angle
    # u = [v, α]T
    # return: x, y, vx, vy, heading
    def move(self, x, dt, u):

        xPos, yPos, heading, vx, vy, angularVelZ, ax, ay = self.getAccelerations(u, x, dt)

        if(self.nPrevKalmanStep < self.nCurrentKalmanStep):
            self.nPrevKalmanStep = self.nCurrentKalmanStep
            #print(f"{u[0]:.4f}, {angularVelZ:.4f}, {xPos:.4f}, {yPos:.4f}, {np.degrees(heading):.2f}, {ax:.5f}, {ay:.5f}", flush=True)

        return np.array([
            xPos, 
            yPos, 
            heading,
            vx, 
            vy,
            angularVelZ, 
            ax, 
            ay
        ])

    # ---

    def normalize_angle(self, x):
        x = x % (2 * np.pi)         # force in range [0, 2 pi)
        if x > np.pi:               # move to [-pi, pi)
            x -= 2 * np.pi
        return x

    # ---

    # The measurement model must convert the state x = [x, y, θ]T into a range and bearing to the landmark. If p is the position
    # of a landmark, the range r is r=sqrt((px-x)^2 + (py-y)^2)
    # We assume that the sensor provides bearing relative to the orientation of the robot, so we must subtract
    # the robot’s orientation from the bearing to get the sensor reading: angle = atan2(py - x[1], px - x[0])

    # The residual is y = z − h(x). Suppose z has a
    # bearing of 1 ◦ and h(x) is 359 ◦ . Subtracting them gives −358 ◦ . This will throw off the computation of the
    # Kalman gain because the correct angular difference is 2 ◦ . So we will have to write code to correctly compute
    # the bearing residual.

    ## as the robot maneuvers different landmarks
    ## will be visible, so we need to handle a variable number of measurements. The function for the residual in
    ## the measurement will be passed an array of several measurements, one per landmark.
    def residual_h(self, a, b):
        # TBD: does arr_npPrediction keep all prev data? Do we only need last one (few)?

        residuals = []
        
        i = 0

        if("gps" in self.dictSensors):
            residuals.append(a[i] - b[i])
            residuals.append(a[i + 1] - b[i + 1])
            i += self.dictSensors["gps"]

        if("imu_accel" in self.dictSensors):
            residuals.append(a[i] - b[i])
            residuals.append(a[i + 1] - b[i + 1])
            # residuals.append(a[i + 2] - b[i + 2])
            # residuals.append(a[i + 3] - b[i + 3])            
            i += self.dictSensors["imu_accel"] 

        if("imu_gyro" in self.dictSensors):
            residuals.append(self.normalize_angle(a[i] - b[i]))     # yaw speed  only
            i += self.dictSensors["imu_gyro"]

        if("imu_magnet" in self.dictSensors):
            residuals.append(self.normalize_angle(a[i] - b[i]))
            i += self.dictSensors["imu_magnet"]

        if("landmarks" in self.dictSensors):
            for j in range(0, self.dictSensors["landmarks"], 2):
                if(self.arrShowLandmarks[j//2] == True):
                    residuals.append(a[i + j] - b[i + j])   # distance
                    residuals.append(self.normalize_angle(a[i + j + 1] - b[i + j + 1]))
                else:
                    residuals.append(self.arr_npPrediction[i + j])
                    residuals.append(self.arr_npPrediction[i + j + 1])

            i += self.dictSensors["landmarks"]

        return residuals

    # ---

    def residual_x(self, a, b):
        y = a - b
        y[2] = self.normalize_angle(y[2])   # heading
        y[5] = self.normalize_angle(y[5])   # angularVelZ
        return y    

    # ---

    def transform_magnetic_field(self, world_mag, heading):
        """
        Transform magnetic field from world frame to robot's frame based on the heading.
        
        Args:
        - world_mag (np.array): Magnetic field vector in the world frame [mag_x, mag_y, mag_z]
        - heading (float): Robot's heading in radians
        
        Returns:
        - np.array: Transformed magnetic field vector in the robot's frame
        """
        # Define the rotation matrix around the z-axis
        R_z = np.array([
            [np.cos(heading), -np.sin(heading), 0],
            [np.sin(heading), np.cos(heading), 0],
            [0, 0, 1]
        ])

        # Transform the magnetic field vector
        robot_mag = R_z.dot(world_mag)

        return robot_mag

    # the measurement model
    # takes a state variable and returns the measurement
    # that would correspond to that state
    def Hx(self, x, u, dt):
        
        # if("imu_angular_vel" in self.dictSensors or "imu_accel" in self.dictSensors or "odom" in self.dictSensors):
        #     xPos, yPos, heading, vx, vy, angularVelZ, ax, ay = self.getAccelerations(u, x, dt)

        hx = []

        if("gps" in self.dictSensors):
            hx.extend(x[[0, 1]])

        if("imu_accel" in self.dictSensors):
            #hx.extend(x[[6, 7, 9, 10]])
            hx.extend(x[[6, 7]])
            
        if("imu_gyro" in self.dictSensors):
            hx.extend([x[2]])

        # <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
        if("imu_magnet" in self.dictSensors):
            # TBD: pass magnetic field of a world as argument
            # world_mag = np.array([6e-06, 2.3e-05, -4.2e-05])
            # robot_mag = world_mag #self.transform_magnetic_field(world_mag, x[2])
            hx.extend([x[2]])

        if("landmarks" in self.dictSensors):
            for lmark in self.landmarks:
                px, py = lmark
                dist = sqrt((px - x[0])**2 + (py - x[1])**2)
                angle = atan2(py - x[1], px - x[0])
                hx.extend([dist, self.normalize_angle(angle - x[2])])
        
            self.arr_npPrediction = np.array(hx)

        return np.array(hx)

    # ---

    def state_mean(self, sigmas, Wm):

        x = np.zeros(self.nNumOfStateParams)
        
        x[0] = np.sum(np.dot(sigmas[:, 0], Wm))     # x
        x[1] = np.sum(np.dot(sigmas[:, 1], Wm))     # y

        sum_sin = np.sum(np.dot(np.sin(sigmas[:, 2]), Wm))
        sum_cos = np.sum(np.dot(np.cos(sigmas[:, 2]), Wm))
        x[2] = atan2(sum_sin, sum_cos)              # theta

        x[3] = np.sum(np.dot(sigmas[:, 3], Wm))     # vx
        x[4] = np.sum(np.dot(sigmas[:, 4], Wm))     # vy    

        sum_sin = np.sum(np.dot(np.sin(sigmas[:, 5]), Wm))
        sum_cos = np.sum(np.dot(np.cos(sigmas[:, 5]), Wm))
        x[5] = atan2(sum_sin, sum_cos)              # angularVelZ

        x[6] = np.sum(np.dot(sigmas[:, 6], Wm))     # ax
        x[7] = np.sum(np.dot(sigmas[:, 7], Wm))     # ay

        # x[8] = np.sum(np.dot(sigmas[:, 8], Wm))     # gyro_bias_z

        # x[9] = np.sum(np.dot(sigmas[:, 9], Wm))     # accel_bias_vx
        # x[10] = np.sum(np.dot(sigmas[:, 10], Wm))   # accel_bias_vy

        return x    

    # ---

    def z_mean(self, sigmas, Wm):
        z_count = sigmas.shape[1]
        x = np.zeros(z_count)

        i = 0

        if("gps" in self.dictSensors):
            x[i] = np.sum(np.dot(sigmas[:, i], Wm))
            x[i+1] = np.sum(np.dot(sigmas[:,i+1], Wm))
            i += self.dictSensors["gps"]

        if("imu_accel" in self.dictSensors):
            x[i] = np.sum(np.dot(sigmas[:, i], Wm))
            x[i+1] = np.sum(np.dot(sigmas[:,i+1], Wm))
            # x[i+2] = np.sum(np.dot(sigmas[:, i+2], Wm))
            # x[i+3] = np.sum(np.dot(sigmas[:,i+3], Wm))            
            i += self.dictSensors["imu_accel"]

        if("imu_gyro" in self.dictSensors):
            sum_sin = np.sum(np.dot(np.sin(sigmas[:, i]), Wm))
            sum_cos = np.sum(np.dot(np.cos(sigmas[:, i]), Wm))
            x[i] = atan2(sum_sin, sum_cos)
            i += self.dictSensors["imu_gyro"]

        if("imu_magnet" in self.dictSensors):
            sum_sin = np.sum(np.dot(np.sin(sigmas[:, i]), Wm))
            sum_cos = np.sum(np.dot(np.cos(sigmas[:, i]), Wm))
            x[i] = atan2(sum_sin, sum_cos)

            i += self.dictSensors["imu_magnet"]

        if("landmarks" in self.dictSensors):
            for z in range(i, z_count, 2):
                x[z] = np.sum(np.dot(sigmas[:,z], Wm))

                sum_sin = np.sum(np.dot(np.sin(sigmas[:, z+1]), Wm))
                sum_cos = np.sum(np.dot(np.cos(sigmas[:, z+1]), Wm))
                x[z+1] = atan2(sum_sin, sum_cos)

            i += self.dictSensors["landmarks"]

        return x

    # ---

    def run_localization(self, cmds, ellipse_step):
        
        plt.figure()
        
        # plot landmarks
        if len(self.landmarks) > 0:
            plt.scatter(self.landmarks[:, 0], self.landmarks[:, 1], marker='s', s=60)

        track = []
        track_err = []

        for i, u in enumerate(cmds):
            self.nCurrentKalmanStep += 1

            # u.append(2)
            self.sim_pos = self.move(self.sim_pos, self.dt, u)

            track.append(self.sim_pos)
            track_err.append(sqrt((self.sim_pos[0] - self.ukf.x[0])**2 + (self.sim_pos[1] - self.ukf.x[1])**2))

            if("landmarks" in self.dictSensors):
                j = 0
                if("gps" in self.dictSensors):
                    j += self.dictSensors["gps"] 

                if("imu_accel" in self.dictSensors):
                    j += self.dictSensors["imu_accel"]

                if("imu_gyro" in self.dictSensors):
                    j += self.dictSensors["imu_gyro"] 

                if("imu_magnet" in self.dictSensors):
                    j += self.dictSensors["imu_magnet"]                                        

                for lmark in self.landmarks:
                    distance_adjusted_sigma_range = self.sigma_range * sqrt((self.sim_pos[0] - lmark[0])**2 + (self.sim_pos[1] - lmark[1])**2)
                    self.arrSigmas[j] = distance_adjusted_sigma_range**2
                    j += 2
                self.ukf.R = np.diag(self.arrSigmas)

            x_prev = self.ukf.x
            self.ukf.predict(u=u)

            x, y = self.sim_pos[0], self.sim_pos[1]
            z = []

            if("gps" in self.dictSensors):
                if(i % self.gps_step == 0):
                    gpsPos = self.sim_pos[:2] + np.random.normal(0, self.sigma_gps, size=(2))
                else:
                    gpsPos = self.ukf.x[:2] # TBD: should there be a + rand() here?
                z.extend(gpsPos[:2])

            if("imu_accel" in self.dictSensors):
                if(i % self.imu_step == 0):
                    imuAccel = self.sim_pos[6:8] + np.random.normal(0, self.sigma_imu_accel, size=(2))
                    
                    # self.dictState["accel_drift_vx"] = self.dictState["accel_drift_vx"] + imuAccel[0] * self.dt \
                    #     + np.random.normal(0, self.sigma_imu_accel_bias, size=(1))[0]
                    # self.dictState["accel_drift_vy"] = self.dictState["accel_drift_vy"] + imuAccel[1] * self.dt \
                    #     + np.random.normal(0, self.sigma_imu_accel_bias, size=(1))[0]
                else:
                    imuAccel = self.ukf.x[6:8]
                    # self.dictState["accel_drift_vx"] = self.ukf.x[9]
                    # self.dictState["accel_drift_vy"] = self.ukf.x[10]

                z.extend([imuAccel[0], imuAccel[1]]) #, self.dictState["accel_drift_vx"], self.dictState["accel_drift_vy"]])

            if("imu_gyro" in self.dictSensors):
                if(i % self.imu_step == 0):
                    gyro_yaw = self.sim_pos[2] + self.sim_pos[5] * self.dt + np.random.normal(0, self.sigma_imu_gyro, size=(1))

                    # if("imu_accel" in self.dictSensors):
                    #     self.normalize_angle(self.dictState["gyro_drift_vz"] + self.sim_pos[5] * self.dt
                    #     + np.random.normal(0, self.sigma_imu_gyro, size=(1)) 
                    #     + np.random.normal(0, self.sigma_imu_gyro_bias, size=(1)))[0]
                    # else:
                    #     self.dictState["gyro_drift_vz"] = 0
                    # imuGyroZ = np.array([self.dictState["gyro_drift_vz"]])
                else:
                    gyro_yaw = np.array(self.ukf.x[2])
                    # self.dictState["gyro_drift_vz"] = self.ukf.x[8]
                z.extend(gyro_yaw)

            if("imu_magnet" in self.dictSensors):

                if(i % self.mag_step == 0):

                    imuMag = self.transform_magnetic_field(self.world_magnet, (self.sim_pos[2] 
                        + np.random.normal(0, self.sigma_imu_magnet, size=(1)))[0])

                    mag_yaw = np.arctan2(imuMag[0], imuMag[1])
                    mag_yaw = np.array([-mag_yaw])

                    #print(mag_yaw, self.sim_pos[2])

                    # # Here we run Complimentary filter on accel and gyro
                    # if("imu_gyro" in self.dictSensors):

                    #     # Calculate yaw from magnetometer data
                    #     yaw = 0.98 * gyro_yaw + 0.02 * mag_yaw
                    # else:
                    #     yaw = mag_yaw
                    yaw = mag_yaw

                else:
                    yaw = [self.ukf.x[2]]

                z.extend(yaw)

                #print(yaw, self.sim_pos[2])

            # TBD
            if("landmarks" in self.dictSensors):
                # Set random elements in self.arrShowLandmarks to False (hide landmarks)
                nHideLandmarks = np.random.randint(0, 4)
                self.arrShowLandmarks = np.ones(len(self.landmarks), dtype=bool)
                arrFalse = np.random.choice(np.arange(len(self.landmarks)), size=nHideLandmarks, replace=False)
                self.arrShowLandmarks[arrFalse] = False

                for lmark in self.landmarks:
                    dx, dy = lmark[0] - x, lmark[1] - y
                    
                    # * sqrt(dx**2 + dy**2) adds error that depends on distance
                    d = sqrt(dx**2 + dy**2) + randn()*self.sigma_range * sqrt(dx**2 + dy**2)
                    bearing = atan2(lmark[1] - y, lmark[0] - x)
                    a = (self.normalize_angle(bearing - self.sim_pos[2] + randn()*self.sigma_bearing))
                    z.extend([d, a])
                
            self.ukf.update(z, u = u, dt=self.dt)

            if i % ellipse_step == 0:
                plot_covariance_ellipse(
                    (self.ukf.x[0], self.ukf.x[1]), self.ukf.P[0:2, 0:2], std=6,
                    facecolor='g', alpha=0.8)

        track = np.array(track)
        plt.plot(track[:, 0], track[:,1], color='k', lw=2)
        plt.axis('equal')
        plt.title("UKF: Robot localization")
        plt.show()

        # track_err = np.array(track_err)
        # plt.figure()
        # plt.plot(track_err, color='k', lw=2)
        # ymin = np.min(track_err)
        # ymax = np.max(track_err)
        # plt.ylim(ymin - 0.1 * (ymax - ymin), ymax + 0.1 * (ymax - ymin))  # Adjust buffer as needed
        # plt.title("Kalman Error")
        # plt.show()

        return self.ukf

    # ---

def turn(v, t0, t1, steps):
    return [[v, a] for a in np.linspace(np.radians(t0), np.radians(t1), steps)]

    # ---

def main(args=None):
    print("*** Kalman demo - main()", flush=True)

    #landmarks = np.array([])
    landmarks = np.array([[5, 10], [10, 5], [15, 15], [20, 5], [0, 30], [50, 30], [40, 10]])

    # Comment in/out any combination
    dictSensors = { 
        #"gps": 2, 
        #"imu_accel": 2, 
        #"imu_gyro": 1,
        #"imu_magnet": 1,
        "landmarks": 2 * len(landmarks) 
    }

    initPos = [2, 6, np.radians(0), 0, 0, np.radians(0), 0, 0]
    
    dt = 0.1
    nGpsStep = 1
    nImuStep = 1
    nMagStep = 10

    nav = KalmanNav(landmarks, dictSensors, initPos,
        sigma_gps = 10.0,
        sigma_imu_accel = 0.01, #2,
        # sigma_imu_accel_bias = 0.001,

        sigma_imu_gyro = 0.01, #0.05, #0.1, #
        # sigma_imu_gyro_bias = 0.0001,

        sigma_imu_magnet = 0.1,
        sigma_range=10, 
        sigma_bearing=1,
        dt=dt,
        gps_step=nGpsStep,
        imu_step=nImuStep,
        mag_step=nMagStep)

    # cmds = []
    
    # # accelerate from a stop
    # cmds = [[v, 0.] for v in np.linspace(0.001, 1.1, 10)]
    # v = cmds[-1][0]

    # # turn left
    # cmds.extend(turn(v, 0, 30, 50))
    # #cmds.extend([v.copy() for v in [cmds[-1]] * 100])

    # cmds[-1][1] = 0
    # cmds.extend([cmds[-1]]*100)

    # # Drive straight at 45 degrees (note: initial heading is 45 degrees)
    # cmds.extend([cmds[-1]] * 50)

    # # turn right
    # cmds.extend(turn(v, 0, -30, 50))
    # cmds[-1][1] = 0
    # cmds.extend([cmds[-1]]*100)

    # # more right
    # cmds.extend(turn(v, 0, -15, 50))
    # cmds[-1][1] = 0
    # cmds.extend([cmds[-1]]*100)

    # # turn left
    # cmds.extend(turn(v, 0, 30, 50))
    # cmds[-1][1] = 0
    # cmds.extend([cmds[-1]]*100)

    # # more left
    # cmds.extend(turn(v, 0, 15, 50))
    # cmds[-1][1] = 0
    # cmds.extend([cmds[-1]]*100)

    # # more left
    # cmds.extend(turn(v, 0, 50, 50))
    # cmds[-1][1] = 0
    # cmds.extend([cmds[-1]]*400)

    # cmds = [[v, vv] for v, vv in zip(np.linspace(0.001, 1.1, 50), np.linspace(0.001, 0.02, 50))]
    # cmds.extend([cmds[-1]]*10000)
    # cmds.append([0,0])    

    # cmds = [[v, vv] for v, vv in zip(np.linspace(0.001, 1.0, 50), np.linspace(0.001, 0.02, 50))]       # 50
    # cmds.extend([cmds[-1]]*3100)
    # cmds.append([0,0])    

    # cmds = []
    # for i in range(51):
    #     cmds.append([i / 25., 0.])
    # cmds.extend([cmds[-1]]*1000)
    # cmds.append([0,0]) 

    # cmds = []
    # for i in range(51):
    #     cmds.append([i / 25., 0.])
    # cmds.append([2., 0.2])
    # # Drive in circle
    # cmds.extend([cmds[-1]]*300)
    # cmds.append([0,0]) 

    # cmds = []
    # cmds.extend(turn(0, 0, 25., 9))
    # cmds.append([0,0])
    # for i in range(51):
    #     cmds.append([3*i / 50., 0.0])
    # cmds.extend([cmds[-1]]*10000)
    # cmds.append([0,0])     

    cmds = []
    cmds.append([10., 0.05])
    cmds.extend([cmds[-1]]*1200)
    cmds.append([0,0])     

    ukf = nav.run_localization(cmds, ellipse_step = 50)

    print('final covariance', ukf.P.diagonal())

# ---

if __name__ == '__main__':
    main()

# ---


