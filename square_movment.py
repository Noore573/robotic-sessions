from controller import Robot
import math

EPUCK_WHEEL_RADUIS = 0.02 # the raduis of the wheel in E-Puck robot
EPUCK_RADUIS = 0.026 # half the distance between E-Puck wheels
EPUCK_MAX_VELCODITY = 6.28 # E-puck motor max velcoity

class RobotController(Robot):
    """
    The class that will represent the robot controller
    """

    def __init__(self):
        Robot.__init__(self)
        
        # define the timestep for the simulation
        self.timestep = int(self.getBasicTimeStep())

        # define the define motors
        self.left_motor = self.getDevice("left wheel motor")
        self.right_motor = self.getDevice("right wheel motor")

        # set motors positon to +infinity 
        # (
        #   the position represents the goal for the motor,
        #   so the motor will run for infinity,
        #   and we will control it using the velocity only
        # ) 
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))

        # define sensors
        self.left_motor_sensor = self.getDevice("left wheel sensor")
        self.right_motor_sensor = self.getDevice("right wheel sensor")

        # enabling sensors
        self.left_motor_sensor.enable(self.timestep)
        self.right_motor_sensor.enable(self.timestep)

        # doing one step to insure that the sensors have been enabled.
        self.step(self.timestep)

    def run_motors_for_rotations_by_right_motor(
        self,
        rotations,
        left_velocity,
        right_velocity
    ):
        """
            A funtion that will run two motors for specafic velocity
            for specafic rotations
            and will depend on right motor sensor to calculate rotations
        """

        # calculate angle = number_of_rotations / 2 * PI
        angle = rotations * 2 * math.pi
        
        # get first value of the sensor
        curr = self.right_motor_sensor.getValue()

        # set motors velocities
        self.left_motor.setVelocity(left_velocity)
        self.right_motor.setVelocity(right_velocity)

        # do stepping until the differance between initial sensor value
        # and current value is less than the required angle
        while(self.right_motor_sensor.getValue() - curr < angle):
            self.step(self.timestep)

        # reset motors velocities to zero
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
        

    def move_distance_by_right_motor(
        self,
        distance,
        left_velocity = EPUCK_MAX_VELCODITY,
        right_velocity = EPUCK_MAX_VELCODITY
    ):
        """
            A funtion that will move the robot by specafic distance,
            with specafic motors velocities,
            and will depend on right motor sensor to calculate the distance
        """

        rotations = distance / (2 * math.pi * EPUCK_WHEEL_RADUIS)
        self.run_motors_for_rotations_by_right_motor(
            rotations,
            left_velocity,
            right_velocity
        )

    
    def turn_angle(self, angle):
        """
            A funtion that will turn the robot by specafic angle (in degrees) counterclockwise
        """
        distance = (2 * math.pi * EPUCK_RADUIS) / (360 / angle)
        self.move_distance_by_right_motor(
            distance,
            -2,
            2
        )

    
    def loop_square(self):
        """
            A funtion that will move the motor in square forever
        """
        while self.step(self.timestep) != -1:
            self.move_distance_by_right_motor(0.25)
            self.turn_angle(98)

r = RobotController()
r.loop_square()
