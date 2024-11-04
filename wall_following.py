"""wall_following_2 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

EPUCK_MAX_VELOCITY = 6.28 # E-puck motor max velcoity


def range_convertor(s_start, s_end, d_start, d_end, value):
    """
    This function is responsible for mapping ranges
    examples:
    the mapping of the value 50 from range 0 -> 200 to range -50 -> 50 will be -25
    """
    ratio = abs((value - s_start) / (s_end - s_start))
    if d_start > d_end:
        return d_start - (abs(d_end - d_start) * ratio)
    else:
        return d_start + (abs(d_end - d_start) * ratio)

class RobotController(Robot):
    def __init__(self):
        Robot.__init__(self)

        self.timestep = int(self.getBasicTimeStep())

        #initializing motors
        self.left_motor = self.getDevice("left wheel motor")
        self.right_motor = self.getDevice("right wheel motor")

        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))

        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

        #initializing distance sensors
        #NOTE: all of these sensors are proximity sensor so they give bigger values as they go closer to the detected object (e.g. a wall).
        self.right_sensor = self.getDevice("ps2")
        self.right_sensor.enable(self.timestep)

        self.front_sensor = self.getDevice("ps0")
        self.front_sensor.enable(self.timestep)

        self.corner_sensor = self.getDevice("ps1")
        self.corner_sensor.enable(self.timestep)

        # running one step to ensure that the sensors been activated
        self.step(self.timestep)

    

    def steering(self, steering, velocity = EPUCK_MAX_VELOCITY):
        """
        A function that is responsible for the steering functionality for the motor
        Steering value:
            - from -100 to 0 will turn left
            - from 0 to 100 will turn right
            - if equals 100 will turn the robot around it self to the right
            - if equals -100 will turn the robot around it self to the left
            - if equals 50 will turn the robot around right wheel to the right
            - if equals -50 will turn the robot around left wheel to the left
        """
        right_velocity = velocity if  steering < 0 else range_convertor(100, 0, -velocity, velocity, steering)
        left_velocity = velocity if  steering > 0 else range_convertor(-100, 0, -velocity, velocity, steering)
        self.right_motor.setVelocity(right_velocity)
        self.left_motor.setVelocity(left_velocity)


    def wall_following_step(self, velocity = EPUCK_MAX_VELOCITY):
        """
        The function that is responsible for doing one step of the wall following (without looping) 
        """
        
        #getting sensors values
        right_value = self.right_sensor.getValue()
        front_value = self.front_sensor.getValue()
        corner_value = self.corner_sensor.getValue()

        self.left_motor.setVelocity(velocity)
        self.right_motor.setVelocity(velocity)

        # checking front wall condition
        if front_value > 80 or corner_value > 80:
            self.left_motor.setVelocity(-2)
            self.right_motor.setVelocity(2)
        else:
        # run steering according the right sensor value
            self.steering(
                range_convertor(
                    # the minimum value of the sensor
                    60,
                    # the maximum value of the sensor
                    140,
                    # the maximum value of the steering to make it smooth
                    20,
                    # the minimum value of the steering to make it smooth
                    -20,
                    # the value of the sensor which we want to map
                    right_value
                )
            )


    def wall_following(self):
        """
        The function that is responsible for the wall fallowing functionality (with looping) 
        """
        while(self.step(self.timestep) != -1):
            self.wall_following_step()


r = RobotController()
r.wall_following()