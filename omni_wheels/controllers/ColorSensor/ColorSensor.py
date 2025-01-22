"""line_following controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import numpy as np

EPUCK_MAX_VELOCITY = 6.28

# PID Factors
Kp = 0.01
Kd = 0.01
Ki = 0

# Last error to be used by the PID.
last_error = 0

#Integral (the accumulation of errors) to be used by the PID.
integral = 0

def range_conversion(s_start, s_end, d_start, d_end, value):
    """
    This function is responsible for mapping ranges
    examples:
    the mapping of the value 50 from range 0 -> 200 to range -50 -> 50 will be -25
    """
    ration = abs((value - s_start) / (s_end - s_start))
    if(d_start < d_end):
        return  d_start + abs(d_end - d_start) * ration 
    if(d_start > d_end):
        return  d_start - abs(d_end - d_start) * ration 

class RobotController(Robot):
    def __init__(self):
        Robot.__init__(self)
        self.timestep = int(self.getBasicTimeStep())

        self.left_motor = self.getDevice("left wheel motor")
        self.right_motor = self.getDevice("right wheel motor")

        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
        self.camera = self.getDevice("camera")
        self.camera.enable(self.timestep)

        self.sensors = list(map(lambda v: self.getDevice(f"lfs{v}"), range(8)))

        self.weights = [-1000, -1000, -1000, -1000, 1000, 1000, 1000, 1000]

        for sensor in self.sensors:
            print("enabled: ", sensor)
            sensor.enable(self.timestep)

        self.step(self.timestep)
    def get_camera_image(self):
        # Get the image from the camera
        image = self.camera.getImageArray()
        if image:
            # Example: Print the first pixel's RGB values
            pixel = image[0][0]  # Access the top-left pixel
            print("First pixel RGB:", pixel)
        else:
            print("No image data available")
    def process_ground_color(camera):
        # Get the camera image
        image = camera.getImageArray()
        if image:
            # Convert the 2D list to a NumPy array for easier manipulation
            np_image = np.array(image, dtype=np.uint8)

            # Get the center pixel (assuming width and height are equal)
            center_pixel = np_image[len(np_image)//2][len(np_image)//2]
            print(f"Center pixel RGB: {center_pixel}")

            # Example: Check for a specific color (e.g., red)
            if center_pixel[0] > 200 and center_pixel[1] < 50 and center_pixel[2] < 50:
                print("Red detected")
        else:
            print("No image data available")
    def get_sensors_value(self):
        value = 0

        for index, sensor in enumerate(self.sensors):
            if(sensor.getValue() > 200):
                value += self.weights[index]
        if value!=0:
            print("Value:",value)
        return value

    def PID_step(self, velocity = EPUCK_MAX_VELOCITY):
        global last_error, integral
        value = self.get_sensors_value()
        error = 0 - value
        # Get P term of the PID.
        P = Kp * error
        # Get D term of the PID.
        D = Kd * (last_error - error)
        # Update last_error to be used in the next iteration.
        last_error = error
        # Get I term of the PID.
        I = Ki * integral
        # Update intergral to be used in the next iteration.
        integral += error

        steering = P + D + I
        self.run_motors_stearing(steering,velocity)


    def line_follow_step(self, velocity = EPUCK_MAX_VELOCITY):
        value = self.get_sensors_value()
        self.run_motors_stearing(range_conversion(-4000, 4000, 30, -30, value), velocity)

    def run_motors_stearing(self, stearing, velocity = EPUCK_MAX_VELOCITY):

        right_velocity = velocity if stearing < 0 else range_conversion(0, 100, velocity, -velocity, stearing)
        left_velocity = velocity if stearing > 0 else range_conversion(0, -100, velocity, -velocity, stearing)
        self.left_motor.setVelocity(left_velocity)
        self.right_motor.setVelocity(right_velocity)

    def PID(self, velocity = EPUCK_MAX_VELOCITY):
        while(self.step(self.timestep) != -1):
            self.PID_step(velocity)
            self.get_camera_image()
    def print_sensor_value(self):
        while(self.step(self.timestep) != -1):
            print(self.get_sensors_value())
            


r = RobotController()
r.PID(2)
