"""line_following controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

EPUCK_MAX_VELOCITY = 6.28


def range_conversion(s_start, s_end, d_start, d_end, value):
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

        # self.sensor = self.getDevice("line follow sensor")
        # self.sensor.enable(self.timestep)

        self.sensors = list(map(lambda v: self.getDevice(f"lfs{v}"), range(8)))

        self.weights = [-1000, -1000, -1000, -1000, 1000, 1000, 1000, 1000]

        for sensor in self.sensors:
            #print("enabled: ", sensor)
            sensor.enable(self.timestep)

        self.step(self.timestep)

    def get_sensors_value(self):
        value = 0

        for index, sensor in enumerate(self.sensors):
            if(sensor.getValue() > 200):
                value += self.weights[index]

        return value
                

    def line_follow(self, velocity = EPUCK_MAX_VELOCITY):
        value = self.get_sensors_value()
        print(value)
        self.run_motors_stearing(range_conversion(-4000, 4000, 30, -30, value), velocity)
        # if(value < 0):
        #     self.left_motor.setVelocity(velocity)
        #     self.right_motor.setVelocity(velocity / 2)
        # elif(value > 0):
        #     self.left_motor.setVelocity(velocity / 2)
        #     self.right_motor.setVelocity(velocity)
        # else:
        #     self.left_motor.setVelocity(velocity)
        #     self.right_motor.setVelocity(velocity)

    def run_motors_stearing(self, stearing, velocity = EPUCK_MAX_VELOCITY):
        right_velocity = velocity if stearing < 0 else range_conversion(0, 100, velocity, -velocity, stearing)
        left_velocity = velocity if stearing > 0 else range_conversion(0, -100, velocity, -velocity, stearing)
        self.left_motor.setVelocity(left_velocity)
        self.right_motor.setVelocity(right_velocity)

    def loop(self):
        while(self.step(self.timestep) != -1):
            self.line_follow()
            self.print_sensor_value()

    def print_sensor_value(self):
        while(self.step(self.timestep) != -1):
            print(self.get_sensors_value())


r = RobotController()
r.loop()

