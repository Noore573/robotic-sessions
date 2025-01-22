from controller import Supervisor, Robot
import json
import numpy as np


class RobotController(Supervisor):  # Use Supervisor instead of Robot
    def __init__(self):
        super().__init__()
        self.timestep = int(self.getBasicTimeStep())

        # Load the color matrix from a JSON file
        try:
            with open("color_matrix.json", "r") as file:
                print("Reading color matrix...")
                data = json.load(file)
                self.color_matrix = data.get("color_matrix")
                if not self.color_matrix or not isinstance(self.color_matrix, list) or len(self.color_matrix[0]) != 4:
                    raise ValueError("Invalid or missing 'color_matrix' in JSON. Ensure it has one row with 4 colors.")
        except Exception as e:
            print(f"Error reading color matrix: {e}")
            exit(1)

        # Initialize wheels
        self.front_right_wheel = self.getDevice("wheel1")
        self.front_left_wheel = self.getDevice("wheel2")
        self.back_right_wheel = self.getDevice("wheel3")
        self.back_left_wheel = self.getDevice("wheel4")

        # Initialize arm and finger
        self.arm1 = self.getDevice("arm1")
        self.arm2 = self.getDevice("arm2")
        self.arm3 = self.getDevice("arm3")
        self.arm4 = self.getDevice("arm4")
        self.arm5 = self.getDevice("arm5")
        self.finger = self.getDevice("finger::left")
        self.camera = self.getDevice("camera")
        self.camera.enable(self.timestep)

        # Set motors to infinite position and initialize velocity
        for motor in [self.arm1, self.arm2, self.arm3, self.arm4, self.arm5, self.finger]:
            motor.setPosition(float("inf"))
            motor.setVelocity(0)

        for wheel in [self.front_right_wheel, self.front_left_wheel, self.back_right_wheel, self.back_left_wheel]:
            wheel.setPosition(float("inf"))
            wheel.setVelocity(0)

        # Setup the environment (Carpet colors)
        self.setup_environment()
    # color matrix
    def setup_environment(self):
        # Get the Carpet node by DEF name
        carpet_node = self.getFromDef("Carpet")
        if carpet_node is None:
            print("Error: 'Carpet' node not found.")
            exit(1)

        # Get the children of the Carpet node
        children_field = carpet_node.getField("children")
        if children_field is None:
            print("Error: 'Carpet' node has no 'children' field.")
            exit(1)

        # Loop through existing Transform nodes and modify their colors
        for i in range(children_field.getCount()):
            child = children_field.getMFNode(i)
            if child is None:
                print(f"Warning: Missing child at index {i}. Skipping.")
                continue

            # Get the Shape node within the Transform
            shape = child.getField("children").getMFNode(0)
            if shape is None:
                print(f"Warning: No Shape node found in child {i}. Skipping.")
                continue

            # Get the Appearance node
            appearance = shape.getField("appearance").getSFNode()
            if appearance is None:
                print(f"Warning: No Appearance node found in Shape of child {i}. Skipping.")
                continue

            # Get the Material node
            material = appearance.getField("material").getSFNode()
            if material is None:
                print(f"Warning: No Material node found in Appearance of child {i}. Skipping.")
                continue

            # Get the diffuseColor field and set its value
            diffuse_color_field = material.getField("diffuseColor")
            if diffuse_color_field is None:
                print(f"Warning: No 'diffuseColor' field found in Material of child {i}. Skipping.")
                continue

            try:
                new_color = self.color_matrix[0][i]
                diffuse_color_field.setSFColor(new_color)  # Correctly sets the color
                print(f"Updated child {i} color to {new_color}")
            except IndexError:
                print(f"Error: Insufficient colors in color_matrix for child {i}. Skipping.")
                continue
    def get_camera_image(self):
        # Get the image from the camera
        image = self.camera.getImageArray()
        if image:
            # Example: Print the first pixel's RGB values
            pixel = image[0][0]  # Access the top-left pixel
            if pixel!=[180, 180, 180]:
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
    def set_motors_velocity(self, wheel1_v, wheel2_v, wheel3_v, wheel4_v):
        self.front_right_wheel.setVelocity(wheel1_v)
        self.front_left_wheel.setVelocity(wheel2_v)
        self.back_right_wheel.setVelocity(wheel3_v)
        self.back_left_wheel.setVelocity(wheel4_v)

    def move_forward(self, velocity):
        self.set_motors_velocity(velocity, velocity, velocity, velocity)

    def move_backward(self, velocity):
        self.set_motors_velocity(-velocity, -velocity, -velocity, -velocity)

    def move_left(self, velocity):
        self.front_right_wheel.setVelocity(velocity)
        self.front_left_wheel.setVelocity(-velocity)
        self.back_left_wheel.setVelocity(velocity)
        self.back_right_wheel.setVelocity(-velocity)

    def move_right(self, velocity):
        self.front_right_wheel.setVelocity(-velocity)
        self.front_left_wheel.setVelocity(velocity)
        self.back_left_wheel.setVelocity(-velocity)
        self.back_right_wheel.setVelocity(velocity)

    def turn_cw(self, velocity):
        self.front_right_wheel.setVelocity(-velocity)
        self.front_left_wheel.setVelocity(velocity)
        self.back_left_wheel.setVelocity(velocity)
        self.back_right_wheel.setVelocity(-velocity)

    def turn_ccw(self, velocity):
        self.front_right_wheel.setVelocity(velocity)
        self.front_left_wheel.setVelocity(-velocity)
        self.back_left_wheel.setVelocity(-velocity)
        self.back_right_wheel.setVelocity(velocity)

    def loop(self):
        while self.step(self.timestep) != -1:
            self.move_forward(6)
            self.get_camera_image()


# Instantiate and run the controller
r = RobotController()
r.loop()
