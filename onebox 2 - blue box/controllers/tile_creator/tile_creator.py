from controller import Supervisor
import json

# Initialize Supervisor
supervisor = Supervisor()

# Load the color matrix from a JSON file
try:
    with open("color_matrix.json", "r") as file:
        data = json.load(file)
        color_matrix = data.get("color_matrix")
        if not color_matrix or not isinstance(color_matrix, list) or len(color_matrix[0]) != 4:
            raise ValueError("Invalid or missing 'color_matrix' in JSON. Ensure it has one row with 4 colors.")
except Exception as e:
    print(f"Error reading color matrix: {e}")
    exit(1)

# Get the Carpet node by DEF name
carpet_node = supervisor.getFromDef("Carpet")
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
        new_color = color_matrix[0][i]
        diffuse_color_field.setSFColor(new_color)  # Correctly sets the color
        print(f"Updated child {i} color to {new_color}")
    except IndexError:
        print(f"Error: Insufficient colors in color_matrix for child {i}. Skipping.")
        continue

# Run the simulation
while supervisor.step(int(supervisor.getBasicTimeStep())) != -1:
    pass
