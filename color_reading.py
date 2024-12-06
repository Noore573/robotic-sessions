"""my_controller controller."""

# You need first to add camera sensor to the Robot.
# And put the size of the camera sensor to 1x1 (width=1, height=1) from the sensor parameters.
# And it's prefered to set the fieldOfView to a very small value (e.g. 0,00001)
from controller import Robot

class RobotController(Robot):
    def __init__(self):
        Robot.__init__(Robot)
        self.timestep = int(self.getBasicTimeStep())

        self.camera = self.getDevice("cam")
        self.camera.enable(self.timestep)

        self.step()


    def loop(self):
        while(self.step(self.timestep) != -1):
            # print(dir(self.camera))
            cameraArray = self.camera.getImageArray()
            red = cameraArray[0][0][0]
            green = cameraArray[0][0][1]
            blue = cameraArray[0][0][2]

            if green == 0 and blue == 0: print("red")
            if red == 0 and blue == 0: print("green")
            if green == 0 and red == 0: print("blue")
            if green != 0 and red != 0 and blue == 0 : print("yellow")

            # print(red,"\t", green ,"\t",blue)
            # print("---------------")


r = RobotController()
r.loop()



