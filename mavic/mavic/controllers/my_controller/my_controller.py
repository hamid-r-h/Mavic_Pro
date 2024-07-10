from controller import Robot
import sys
import numpy as np
import matplotlib.pyplot as plt
import cv2
import numpy as np
import pandas as pd
import math


class Mavic(Robot):
    VERTICAL_CONSTANT = 68.5  
    VERTICAL_DISPLACEMENT_CONSTANT = 0.6
    K_VERTICAL_P = 3.0        
    ROOL_PROPORTIONAL = 50.0           
    PITCH_PROPORTIONAL = 30.0          
    MAX_YAW_deviation = 0.4
    MAX_pitch_deviation = -1
    target_tolerance = 0.1
    MAX_ERROR = 0.03 

    def __init__(self):
        super().__init__()  
        self.init_devices()
        self.motor_setup()
        self.pose_initial_setup()
        self.target_setup()
        self.boxes_setup()

    def motor_setup(self):
        motors = [self.front_left_motor, self.front_right_motor,
                  self.rear_left_motor, self.rear_right_motor]
        for motor in motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(1)

    def pose_initial_setup(self):
        self.current_pose = [0, 0, 0, 0, 0, 0]

    def target_setup(self):
        self.target_location = [0, 0, 0]
        self.target_index = 0
        self.target_height = 0

    def boxes_setup(self):
        self.boxes = [[-3, -2], [2, 5], [3, -3], [5, 0], [-5, 4]]

    def init_devices(self):
            self.time_step = int(self.getBasicTimeStep())
            self.camera = self.getDevice("camera")
            self.camera.enable(self.time_step)
            self.gyro = self.getDevice("gyro")
            self.gyro.enable(self.time_step)
            self.front_left_motor = self.getDevice("front left propeller")
            self.front_right_motor = self.getDevice("front right propeller")
            self.rear_left_motor = self.getDevice("rear left propeller")
            self.rear_right_motor = self.getDevice("rear right propeller")
            self.imu = self.getDevice("inertial unit")
            self.imu.enable(self.time_step)
            self.gps = self.getDevice("gps")
            self.gps.enable(self.time_step)
            self.camera_pitch_motor = self.getDevice("camera pitch")
            self.camera_pitch_motor.setPosition(1.62)
      
    def navigate_to_goal(self):

            if self.target_location[:2] == [0, 0]:
                self.target_location[:2] = self.boxes[0]
                print(f"target detected {self.target_location[0:2]}")

            current_target = self.target_location
            current_location = self.current_pose[:2]


            current_roll, current_pitch, current_yaw = self.imu.getRollPitchYaw()

            if abs(current_target[0] - current_location[0]) < self.target_tolerance and abs(current_target[1] - current_location[1]) < self.target_tolerance:
                
                self.target_index += 1
                self.target_index  %= len(self.boxes)
                
                self.target_location[0:2] = self.boxes[self.target_index]
                print("Target reached! New target: ", self.target_location[0:2])

                while abs(current_yaw) >= 0.1:
                    self.rotate()
                    current_roll, current_pitch, current_yaw = self.imu.getRollPitchYaw()
                
                self.show_img()

            roll_deviation, pitch_deviation, yaw_deviation = self.update_deviation(current_target, current_location)

            delta_y = self.target_location[1] - self.current_pose[1]
            delta_x = self.target_location[0] - self.current_pose[0]
            self.target_location[2] = np.arctan2(delta_y, delta_x)

            angle_left = self.target_location[2] - self.current_pose[5]

            angle_left = np.fmod((angle_left + 2 * np.pi), (2 * np.pi))
            angle_left = angle_left - 2 * np.pi if angle_left > np.pi else angle_left

            yaw_deviation = self.MAX_YAW_deviation * angle_left / (2 * np.pi)
            pitch_deviation = clamp(np.log10(abs(angle_left)), self.MAX_pitch_deviation, 0.1)

            return yaw_deviation, pitch_deviation, roll_deviation 

    def rotate(self):
            pitch_deviation = 0.0
            yaw_deviation = -1.3
            roll_deviation = 0.0

            delta_y = self.target_location[1] - self.current_pose[1]
            delta_x = self.target_location[0] - self.current_pose[0]
            self.target_location[2] = np.arctan2(delta_y, delta_x)

            angle_left = self.target_location[2] - self.current_pose[5]
            angle_left = np.fmod((angle_left + 2 * np.pi), (2 * np.pi))
            angle_left = angle_left - 2 * np.pi if angle_left > np.pi else angle_left

            roll, pitch, yaw = self.imu.getRollPitchYaw()

            x_location, y_location, height = self.gps.getValues()

            roll_acceleration, pitch_acceleration, _ = self.gyro.getValues()

            self.current_pose = [x_location, y_location, height, roll, pitch, yaw]

            t1 = self.getTime()

            roll_in = self.ROOL_PROPORTIONAL * clamp(roll, -1, 1) + roll_acceleration + roll_deviation
            pitch_in = self.PITCH_PROPORTIONAL * clamp(pitch, -1, 1) + pitch_acceleration + pitch_deviation
            yaw_input = yaw_deviation
            clamped_difference_height = clamp(self.target_height - height + self.VERTICAL_DISPLACEMENT_CONSTANT, -1, 1)
            vertical_input = self.K_VERTICAL_P * np.power(clamped_difference_height, 3.0)

            thrust = self.VERTICAL_CONSTANT + vertical_input

            front_left_motor_input = thrust - yaw_input + pitch_in - roll_in
            front_right_motor_input = thrust + yaw_input + pitch_in + roll_in
            rear_left_motor_input = thrust + yaw_input - pitch_in - roll_in
            rear_right_motor_input = thrust - yaw_input - pitch_in + roll_in
            self.front_left_motor.setVelocity(front_left_motor_input)
            self.front_right_motor.setVelocity(-front_right_motor_input)
            self.rear_left_motor.setVelocity(-rear_left_motor_input)
            self.rear_right_motor.setVelocity(rear_right_motor_input)
            self.step(3*self.time_step)

        
    def show_img(self):
            image = self.camera.getImage()
            if image:
                width, height = self.camera.getWidth(), self.camera.getHeight()
                image_array = np.frombuffer(image, dtype=np.uint8).reshape((height, width, 4))
                image_array = cv2.rotate(image_array, cv2.ROTATE_180)
                plt.imshow(image_array)
                cv2.imwrite("./besttemp.jpeg", image_array)
                plt.title("Captured Image")
                plt.show()
        
    def update_deviation(self, current_target, current_location):

        roll_disturbance = 0.0
        pitch_disturbance = 0.0
        yaw_disturbance = 0.0
    
        if abs(current_target[0] - current_location[0]) > self.MAX_ERROR and abs(current_target[1] - current_location[1]) > self.MAX_ERROR : 
            if current_location[0] < current_target[0]:
                roll_disturbance = -0.5
            if current_target[0] > current_location[0]:
                roll_disturbance = 0.5
        if abs(current_target[0] - current_location[0]) > self.MAX_ERROR:
    
            if current_location[0] < current_target[0]:
                yaw_disturbance = -1.3
            if current_target[0] > current_location[0]:
                yaw_disturbance = 1.3    
        if abs(current_target[1] - current_location[1]) > self.MAX_ERROR:
            if current_location[1] < current_target[1]:
                pitch_disturbance = -2.0
            if current_target[1] < current_location[1]:
                pitch_disturbance = 2.0
        return roll_disturbance, pitch_disturbance, yaw_disturbance 



                    

    def run(self):

            t1 = self.getTime()
            yaw_deviation = 0
            roll_deviation = 0
            pitch_deviation = 0
            self.target_height = 2.8
            self.boxes = [[-3, -2], [3, -3], [5, 0], [2, 5], [-5, 4]]

            while self.step(3*self.time_step) != -1:
            
                roll, pitch, yaw = self.imu.getRollPitchYaw()
            
                x_location, y_location, height = self.gps.getValues()
                roll_acceleration, pitch_acceleration, _ = self.gyro.getValues()
                self.current_pose = [x_location, y_location, height, roll, pitch, yaw]

                if height > self.target_height - 1:
                    if self.getTime() - t1 > 0.1:
                        yaw_deviation, pitch_deviation, roll_deviation = self.navigate_to_goal()
                        t1 = self.getTime()

                roll_in = self.ROOL_PROPORTIONAL * clamp(roll, -1, 1) + roll_acceleration + roll_deviation
                pitch_in = self.PITCH_PROPORTIONAL * clamp(pitch, -1, 1) + pitch_acceleration + pitch_deviation
                yaw_input = yaw_deviation
                clamped_difference_height = clamp(self.target_height - height + self.VERTICAL_DISPLACEMENT_CONSTANT, -1, 1)
                vertical_input = self.K_VERTICAL_P * np.power(clamped_difference_height, 3.0)

                thrust = self.VERTICAL_CONSTANT + vertical_input

                front_left_motor_input = thrust - yaw_input + pitch_in - roll_in
                front_right_motor_input = thrust + yaw_input + pitch_in + roll_in
                rear_left_motor_input = thrust + yaw_input - pitch_in - roll_in
                rear_right_motor_input = thrust - yaw_input - pitch_in + roll_in

                self.front_left_motor.setVelocity(front_left_motor_input)
                self.front_right_motor.setVelocity(-front_right_motor_input)
                self.rear_left_motor.setVelocity(-rear_left_motor_input)
                self.rear_right_motor.setVelocity(rear_right_motor_input)
        

def clamp(value, value_min, value_max):
            return max(value_min, min(value, value_max))




if __name__ == "__main__":
    robot = Mavic()
    robot.run()


