from controller import Robot
EPUCK_MAX_VELOCITY = 6.28 
# Initialize the robot
# robot = Robot()
class RobotController(Robot):
    def __init__(self):
        Robot.__init__(self)
        timestep = int(robot.getBasicTimeStep())
        self.left_motor = self.getDevice("left wheel motor")
        self.right_motor = self.getDevice("right wheel motor")

        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
        # Initialize the camera sensor

        self.camera = robot.getDevice("color_sensor")
        self.camera.enable(timestep)

    def get_average_color(self,image, width, height):
        """
        Calculates the average RGB color from a camera image.
        """
        total_red, total_green, total_blue = 0, 0, 0
        for x in range(width):
            for y in range(height):
                index = (y * width + x) * 4
                total_red += image[index]
                total_green += image[index + 1]
                total_blue += image[index + 2]

        num_pixels = width * height
        avg_red = total_red / num_pixels / 255.0  # Normalize to [0, 1]
        avg_green = total_green / num_pixels / 255.0
        avg_blue = total_blue / num_pixels / 255.0
        return (avg_red, avg_green, avg_blue)

    # Map RGB values to named colors
    def map_color_to_name(self,rgb):
        color_names = {
            (1, 0, 0): "Red",
            (0, 1, 0): "Green",
            (0, 0, 1): "Blue",
            (1, 1, 0): "Yellow"
        }
        closest_color = min(color_names.keys(), key=lambda c: sum(abs(c[i] - rgb[i]) for i in range(3)))
        return color_names[closest_color]

    # Main loop
    def loop(self):
        
     while robot.step(self.timestep) != -1:
         image = self.camera.getImageArray()
         self.left_motor.setVelocity(EPUCK_MAX_VELOCITY/2)
         self.right_motor.setVelocity(EPUCK_MAX_VELOCITY/2)
         if image:
             avg_color = self.get_average_color(image, self.camera.getWidth(), self.camera.getHeight())
             color_name = self.map_color_to_name(avg_color)
             print(f"Detected Color: {color_name}")
robot=RobotController()
robot.loop()