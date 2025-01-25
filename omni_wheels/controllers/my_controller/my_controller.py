import math
import time
from controller import Supervisor, Robot
import json
import numpy as np
# center
#[1.1851988573892502, -2.5800691309387913, 0.10193773101325956]
color_matrixx = [] 
YOU_VELOCITY=14.0
reached_distance_Threshold=0.3

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
        self.armMotors = [self.getDevice(f"arm{i}") for i in range(1, 6)]
        for motor in self.armMotors:
            motor.setVelocity(1.5)
        # self.armMotors[1].setPosition(-1)
        # self.armMotors[2].setPosition(-1)
        # self.armMotors[3].setPosition(-1.4)

        self.armPositionSensors = [self.getDevice(f"arm{i}sensor") for i in range(1, 6)]
        for sensor in self.armPositionSensors:
            sensor.enable(self.timestep)
        self.finger1 = self.getDevice("finger::left")
        self.finger2 = self.getDevice("finger::right")
        self.finger1.setVelocity(1.5)
        self.finger2.setVelocity(1.5)
        self.fingerMinPosition = self.finger1.getMinPosition()
        self.fingerMaxPosition = self.finger1.getMaxPosition()
        
        self.camera_bottom = self.getDevice("camera")
        self.camera_bottom.enable(self.timestep)
        self.camera = self.getDevice("front-camera")
        self.camera.enable(self.timestep)
        
        self.gps = self.getDevice("gps")
        self.gps.enable(self.timestep)
        self.inertial_unit = self.getDevice("inertial unit")
        self.inertial_unit.enable(self.timestep)
        print(self.inertial_unit)
        self.reached_distance=False
        
        self.sector_coordinates = {
            "Red": [-2.073329050888169, -2.570625111558194],
            "Blue": [-0.5725307377437417, -1.57287519101565],  
            "Green": [-0.5725307377437417, -3.57287519101565], #-0.6875144492298266, -2.5710764570773796 
            "Yellow": [-0.5725307377437417, -2.57287519101565],  
            "Center":[1.0738674118287237, -2.5753436072422304], #to go back to the center
            "Wall":[0.9342484573122082, -0.167330050096285] #wall
        }
        self.emitter = self.getDevice('emitter')  # Get the emitter device
        self.emitter.setChannel(1)  
        

        # Set motors to infinite position and initialize velocity
        # for motor in [self.arm1, self.arm2, self.arm3, self.arm4, self.arm5, self.fingerL,self.fingerR]:
        #     motor.setPosition(float("inf"))
        #     motor.setVelocity(0)

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
        """
        Get the image from the camera, classify the top-left pixel color, 
        and update the global color matrix with unique colors.
        """
        global color_matrixx  # Use the global color matrix

        # Get the image from the camera
        image = self.camera_bottom.getImageArray()
        if image:
            # Access the top-left pixel
            pixel = image[0][0]  
            # Classify the color
            detected_color = self.classify_color(pixel)

            if detected_color and detected_color not in color_matrixx:
                # Add the detected color to the matrix if it's not already present
                color_matrixx.append(detected_color)
                print(f"Added {detected_color} to color_matrix: {color_matrixx}")
            
            # Exit if we have 4 unique colors
            if len(color_matrixx) == 4:
                print("All 4 colors detected. Stopping...")
                return "0"
        else:
            print("No image data available")

    def classify_color(self, rgb):
        """
        Classify the given RGB value into a specific color.
        """
        r, g, b = rgb
        if r > 150 and g < 100 and b < 100:  
            return "Red"
        elif b > 150 and r < 100 and g < 100:
            return "Blue"
        elif g > 150 and r < 100 and b < 100:
            return "Green"
        elif r > 150 and g > 150 and b < 100:
            return "Yellow"
        return None  # Not a classified color
    

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
            
    # gps stuff
    def get_position(self):
        return self.gps.getValues()
    def calculate_heading(self, target):
        current_pos = self.get_position()
        target_angle = np.arctan2(target[1] - current_pos[1], target[0] - current_pos[0])
        return target_angle

    def rotate_to_angle(self, target_angle):
        while self.step(self.timestep) != -1:
            current_orientation = self.inertial_unit.getRollPitchYaw()[2]
            angle_diff = target_angle - current_orientation

            if abs(angle_diff) < 0.04:  # Small threshold for alignment
                break

            speed = 0.1 if angle_diff > 0 else -0.1
            # self.left_motor.setVelocity(-speed)
            # self.right_motor.setVelocity(speed)
            self.set_motors_velocity(YOU_VELOCITY,-YOU_VELOCITY,YOU_VELOCITY,-YOU_VELOCITY)

        self.set_motors_velocity(0,0,0,0)
    def rotate_to_sector(self, sector):
        sector_quaternions = {
            # original_orientation = (0,0,1, -3.14)
            "Center": (0, 0, 0, -(math.pi/2)),
            "Yellow": (0, 0, 0, -(math.pi/2)),
            "Red": (0, 0, 0, -(math.pi/2)),
            "Blue": (0, 0, 0, 3.14),
            "Green": (0, 0, 0, 3.14),
        }

        # Get the target quaternion for the given sector
        if sector not in sector_quaternions:
            print(f"Unknown sector: {sector}")
            return

        target_quaternion = sector_quaternions[sector]
        print(f"Rotating to sector: {sector}...")

        # Function to extract the yaw (rotation around z-axis) from the quaternion
        def quaternion_to_yaw(q):
            _, _, z, w = q
            yaw = math.atan2(2.0 * (w * z), 1.0 - 2.0 * (z * z))
            return yaw

        # Get the target yaw from the quaternion
        target_yaw = quaternion_to_yaw(target_quaternion)

        while self.step(self.timestep) != -1:
            # Get the current yaw
            current_yaw = self.inertial_unit.getRollPitchYaw()[2]

            # Calculate the difference in angles
            angle_diff = target_yaw - current_yaw

            # Normalize the angle difference to the range [-π, π]
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

            # Check if the robot is close enough to the target yaw
            if abs(angle_diff) < 0.05:  # Small threshold for alignment
                break

            # Set the rotation speed based on the direction of the angle difference
            speed = 10.0 if angle_diff > 0 else -10.0
            self.set_motors_velocity(speed, -speed, speed, -speed)

        # Stop the motors once aligned
        print(f"Rotated to sector: {sector}")
        self.set_motors_velocity(0, 0, 0, 0)
    def rotate_to_world_angle(self, target_position):
         """
         Rotates the robot to face a specific position in world coordinates.

         Args:
             target_position (tuple): The (x, y) coordinates of the target position in the world.
         """
         print(f"Rotating to world coordinates: {target_position}...")

         while self.step(self.timestep) != -1:
             # Get the robot's current position in world coordinates
             current_position = self.gps.getValues()  # Assuming GPS gives (x, y, z)
             current_x, current_y = current_position[0], current_position[1]

             # Calculate the target angle (yaw) using world coordinates
             delta_x = target_position[0] - current_x
             delta_y = target_position[1] - current_y
             target_yaw = math.atan2(delta_y, delta_x)  # The angle in radians the robot needs to face

             # Get the robot's current yaw
             current_yaw = self.inertial_unit.getRollPitchYaw()[2]

             # Calculate the difference in angles
             angle_diff = target_yaw - current_yaw

             # Normalize the angle difference to the range [-π, π]
             angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

             # Check if the robot is close enough to the target angle
             if abs(angle_diff) < 0.05:  # Small threshold for alignment
                 break

             # Set the rotation speed based on the direction of the angle difference
             speed = 10.0 if angle_diff > 0 else -10.0
             self.set_motors_velocity(speed, -speed, speed, -speed)

         # Stop the motors once aligned
         print("Aligned to target world position.")
         self.set_motors_velocity(0, 0, 0, 0)


    def move_to_target(self, target):
        step_count = 0
        while self.step(self.timestep) != -1:
            step_count += 1
            print(self.reached_distance)
            # if self.reached_distance:
            #     break
            current_pos = self.get_position()
            distance = np.linalg.norm([target[0] - current_pos[0], target[1] - current_pos[1]])
            print("Distance :",distance , "therhold :   ",reached_distance_Threshold)
            if distance <= reached_distance_Threshold:
              print("Target reached. Stopping robot.")
              self.reached_distance = True
              self.set_motors_velocity(0, 0, 0, 0)
              break

            # else:
            self.set_motors_velocity(YOU_VELOCITY,YOU_VELOCITY,YOU_VELOCITY,YOU_VELOCITY)
            
        print("notttt")
        self.set_motors_velocity(0,0,0,0)


    def move_to_target_with_line_following(self, target):
     while self.step(self.timestep) != -1:
         current_pos = self.get_position()
        #  print("POS:",current_pos[0],' - ',current_pos[1])
        #  print("")
         distance = np.linalg.norm([target[0] - current_pos[0], target[1] - current_pos[1]])
         print("Distance: ",distance)

         if distance < reached_distance_Threshold:
             break

         # Follow the line while heading towards the target(i hate line following)
         value = self.get_sensors_value()
         error = range_conversion(-4000, 4000, 30, -30, value)
         self.run_motors_stearing(error, velocity=YOU_VELOCITY)

     self.left_motor.setVelocity(0)
     self.right_motor.setVelocity(0)
    def navigate_to_sector(self, color):  # this is without line following
        if color in self.sector_coordinates:
            target = self.sector_coordinates[color]
            print(f"Navigating to {color} sector at {target}...")

            target_angle = self.calculate_heading(target)
            self.rotate_to_angle(target_angle)
            print("move to targes")
            self.move_to_target(target)
            print("ending move to target")
        else:
            print(f"Color {color} not found in sectors.")
    def navigate_to_sector2(self, color): # this is a version of the same previous function  with line following
     if color in self.sector_coordinates:
         target = self.sector_coordinates[color]
         print(f"Navigating to {color} sector at {target}...")
         target_angle = self.calculate_heading(target)
         self.rotate_to_angle(target_angle)
         self.move_to_target_with_line_following(target)
     else:
         print(f"Color {color} not found in sectors.")
    def StandStill(self):
        # get box 
        print("Standing still")
        time.sleep(1)
    
    

    def detect_box_camera(self):
        """
        Automatically detect the box location using the existing camera by performing a 360-degree rotation.
        Returns:
            (x, y, angle_to_box): The relative coordinates of the box and the angle in the robot's reference frame.
        """
        print("Starting 360-degree rotation to find the box...")

        # Rotate the robot in place
        rotation_speed = 14  # Adjust rotation speed if needed
        total_rotation = 0  # Track how far the robot has rotated
        timestep_seconds = self.timestep / 1000.0  # Convert timestep to seconds
        angular_velocity = 0.1  # Approximate angular velocity of the robot in radians per second

        while total_rotation < 2 * math.pi:  # Rotate 360 degrees
            self.set_motors_velocity(rotation_speed, -rotation_speed, rotation_speed, -rotation_speed)

            # Capture image from the camera
            camera_image = self.camera.getImage()
            width, height = self.camera.getWidth(), self.camera.getHeight()

            # Loop through the image to find the target colors
            for y in range(height):
                for x in range(width):
                    r = self.camera.imageGetRed(camera_image, width, x, y)
                    g = self.camera.imageGetGreen(camera_image, width, x, y)
                    b = self.camera.imageGetBlue(camera_image, width, x, y)

                    # Detect red color
                    if r > 200 and g < 50 and b < 50:
                        detected_color = "red"
                    # Detect yellow color
                    elif r > 200 and g > 200 and b < 50:
                        detected_color = "yellow"
                    # Detect green color
                    elif r < 50 and g > 200 and b < 50:
                        detected_color = "green"
                    # Detect blue color
                    elif r < 50 and g < 50 and b > 200:
                        detected_color = "blue"
                    else:
                        detected_color = None

                    if detected_color:
                        print("Detected color: ",detected_color)
                        # Stop the robot
                        self.set_motors_velocity(0, 0, 0, 0)

                        # Calculate angle to the box
                        angle_to_box = (x - width / 2) * self.camera.getFov() / width
                        distance_to_box = 1.0  # Placeholder distance value; estimate if needed

                        print(f"{detected_color.capitalize()} box detected at angle: {angle_to_box:.2f} radians")
                        return detected_color, angle_to_box, distance_to_box

            # Update the total rotation based on angular velocity
            total_rotation += angular_velocity * timestep_seconds
            self.step(self.timestep)

        # Stop the robot if no box is found
        self.set_motors_velocity(0, 0, 0, 0)
        print("Box not found after 360-degree rotation.")
        return None

    def pick_box(self):
        print("Picking up box...")
        self.armMotors[1].setPosition(-1.13)
        self.armMotors[2].setPosition(-0.95)
        self.armMotors[3].setPosition(-1.125)
        self.finger1.setPosition(self.fingerMaxPosition)
        self.finger2.setPosition(self.fingerMaxPosition)
        self.step(50 * self.timestep)
        self.finger1.setPosition(0.0)
        self.finger2.setPosition(0.0)
        self.step(50 * self.timestep)
        self.armMotors[1].setPosition(-0.5)
        self.step(100 * self.timestep)
    def drop_box_in_inv(self):
        """
        Drop the box onto the inventory surface area.
        """
        print("Dropping the box onto the inventory...")
     
        # Move the arm to the drop-off position
        # Adjust these positions to match the inventory's location
        print("moving 1")
        self.armMotors[1].setPosition(+0.6)
        self.step(30 * self.timestep)
        print("moving 2")
        self.armMotors[2].setPosition(+0.6)
        self.step(30 * self.timestep)
        print("moving 3")
        self.armMotors[3].setPosition(+1.3)
        self.step(100 * self.timestep)
     
        # Open the gripper to release the box
        self.finger1.setPosition(self.fingerMaxPosition)
        self.finger2.setPosition(self.fingerMaxPosition)
        self.step(50 * self.timestep)
     
        # Reset the arm to its default (home) position
        print("Resetting the arm to the default position...")
        self.armMotors[1].setPosition(0.0)
        self.armMotors[2].setPosition(0.0)
        self.armMotors[3].setPosition(0.0)
        self.step(100 * self.timestep)
     
        print("Box successfully dropped!")
    # get from inv
    def pick_box_from_inv(self):
        """
        Pick the box from the inventory surface area.
        """
        print("Picking up the box from the inventory...")

        # Move the arm to the inventory position
        print("Moving arm to inventory position...")
        self.armMotors[1].setPosition(+0.75)  # Adjust height
        self.step(30 * self.timestep)
        self.armMotors[2].setPosition(+0.75)  # Adjust forward reach
        self.step(30 * self.timestep)
        self.armMotors[3].setPosition(+1.45)  # Lower arm to grab the box
        self.step(100 * self.timestep)

        # Close the gripper to grab the box
        print("Gripping the box...")
        self.finger1.setPosition(0.0)
        self.finger2.setPosition(0.0)
        self.step(50 * self.timestep)

        # Lift the box slightly
        print("Lifting the box...")
        self.armMotors[3].setPosition(0.5)
        self.step(50 * self.timestep)

        print("Box picked up from the inventory!")
        
    def drop_box_at_front(self):
        """
        Drop the box at the front of the robot.
        """
        print("Dropping the box at the front...")

        # Move the arm to the front drop-off position
        print("Moving arm to front position...")
        self.armMotors[1].setPosition(-0.4)  # Adjust height (lower arm)
        self.step(30 * self.timestep)
        self.armMotors[2].setPosition(0.2)  # Adjust forward reach
        self.step(30 * self.timestep)
        self.armMotors[3].setPosition(-1.0)  # Lower arm over the front surface
        self.step(100 * self.timestep)

        # Open the gripper to release the box
        print("Releasing the box...")
        self.finger1.setPosition(self.fingerMaxPosition)
        self.finger2.setPosition(self.fingerMaxPosition)
        self.step(50 * self.timestep)

        # Reset the arm to its default (home) position
        print("Resetting the arm to the default position...")
        self.armMotors[1].setPosition(0.0)
        self.armMotors[2].setPosition(0.0)
        self.armMotors[3].setPosition(0.0)
        self.step(100 * self.timestep)

        print("Box successfully dropped at the front!")

    def release_box(self):
        print("Releasing the box...")
        self.step(10 * self.timestep)
        self.armMotors[1].setPosition(-1.13)
        self.step(20 * self.timestep)
        self.armMotors[2].setPosition(-0.95)
        self.step(20 * self.timestep)
        self.armMotors[3].setPosition(-1.125)
        self.step(20 * self.timestep)
        self.finger1.setPosition(self.fingerMaxPosition)
        self.finger2.setPosition(self.fingerMaxPosition)
        self.step(50 * self.timestep)
        self.finger1.setPosition(0.0)
        self.finger2.setPosition(0.0)
        self.step(10 * self.timestep)
        self.armMotors[3].setPosition(0)
        self.armMotors[2].setPosition(0)
        self.armMotors[1].setPosition(0)
        self.step(20 * self.timestep)
        # self.step(100 * self.timestep)
        print("Released the box!")
    def CallEmitter(self):
        # Send a signal to the second robot
        message = "Come baby come"
        self.emitter.send(message.encode('utf-8'))  
        print("Signal sent to the second robot.")
         
    def detect_and_pick_box(self):
        # Detect the box
        result = self.detect_box_camera()
        if result:
            angle_to_box, distance_to_box = result
            self.pick_box()
            print("Box picked up!")
            # Move to the drop zone and drop the box
            # self.drop_box_in_inv()
            # print("Box dropped in the inventory!")
    def move_box_from_inv_to_front(self):
        """
        Complete process to pick the box from the inventory and drop it at the front.
        """
        print("Starting process to move box from inventory to front...")

        # Pick the box from the inventory
        self.pick_box_from_inv()

        # Drop the box at the front
        self.drop_box_at_front()

        print("Box successfully moved from inventory to the front!")







    def get_color_matrix(self):
        while self.step(self.timestep) != -1:
            self.move_forward(YOU_VELOCITY)
            value=self.get_camera_image()
            if value=="0": break
            
    def set_motors_velocity(self, wheel1_v, wheel2_v, wheel3_v, wheel4_v):
        # print(wheel1_v)
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
            # ---------------------------
            # for one box
            # self.navigate_to_sector("Center")
            # self.StandStill()
            # self.navigate_to_sector("Yellow")
            # self.StandStill()
            # self.detect_and_pick_box()
            # self.navigate_to_sector("Center")
            # self.StandStill()
            # self.navigate_to_sector("Wall")
            # self.StandStill()
            # self.release_box()
            # self.StandStill()
            # self.CallEmitter()
            
            # self.StandStill()
            # self.navigate_to_sector("Center")
            
            # ---------------------------
            
            # for box test
            # ---------------------------------------
            # self.detect_and_pick_box()
            # ---------------------------------------
        

            
            # self.navigate_to_sector("Center")

            # self.StandStill()
            # self.navigate_to_sector("Yellow")
            # self.StandStill()
            # self.detect_and_pick_box()
            # self.navigate_to_sector("Center")
            # self.StandStill()
            # self.navigate_to_sector("Wall")
            # self.StandStill()
            # self.move_box_from_inv_to_front()
            # self.StandStill()
            
            
            # self.move_forward(YOU_VELOCITY)
            self.get_color_matrix()
            self.navigate_to_sector("Center")
            self.StandStill()
            self.navigate_to_sector(color_matrixx[0])
            self.StandStill()
            self.detect_and_pick_box()
            self.navigate_to_sector("Wall")
            self.StandStill()
            self.release_box()
            self.StandStill()
            self.navigate_to_sector("Center")
            self.StandStill()
            self.navigate_to_sector(color_matrixx[1])
            self.StandStill()
            self.navigate_to_sector("Wall")
            self.StandStill()
            self.navigate_to_sector("Center")
            self.StandStill()
            
            self.navigate_to_sector(color_matrixx[2])
            self.StandStill()
            self.navigate_to_sector("Wall")
            self.StandStill()
            self.navigate_to_sector("Center")
            self.StandStill()
            self.navigate_to_sector(color_matrixx[3])
            self.StandStill()
            self.navigate_to_sector("Wall")
            self.StandStill()
            self.navigate_to_sector("Center")
            
            # self.move_forward(YOU_VELOCITY)
            
            # self.get_camera_image()
            # current_pos = self.get_position()
            # print(current_pos)


# Instantiate and run the controller
r = RobotController()
r.loop()
