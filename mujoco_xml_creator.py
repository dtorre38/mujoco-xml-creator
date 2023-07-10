import xml.etree.ElementTree as ET
import xml.dom.minidom as minidom
import random
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
import random


def random_2d_coordinates(num_grids, height, length):
    # num_grids: number of random grid coordinates to return
    # height: spacing between adjacent grid points
    # length: total length of square grid

    # Total grid points in 1d, add one to avoid Fencepost Error
    pts_qty = int(length / height) + 1

    # Produce an odd number of points, allows center point to always be (0,0)
    if pts_qty % 2 == 0:
        pts_qty -= 1

    # Coordinates of points in 1d
    pts_1d = list(range(pts_qty))

    # Center points at zero
    pts_1d = [val - (pts_qty - 1) / 2 for val in pts_1d]

    # Scale 1d points by space between grid points
    pts_1d = [height * val for val in pts_1d]

    # Generate meshgrid arrays
    xx, yy = [], []
    for x in pts_1d:
        for y in pts_1d:
            xx.append(x)
            yy.append(y)

    # Update pts_qty to reflect total number of 2d points
    pts_qty = len(xx)

    # Store x-y coordinate pairs in [1xP] list for convenience
    crd_all = [(xx[i], yy[i]) for i in range(pts_qty)]

    # Select random x-y coordinate pairs at random
    crd_rnd = []
    for _ in range(num_grids):
        # Size of list
        len_list = len(crd_all) - 1

        # Generate random index
        ran_id = random.randint(0, len_list)

        # Store randomly selected coordinate
        crd_rnd.append(crd_all[ran_id])

        # Remove selected coordinate from list of all coordinates
        crd_all.pop(ran_id)

    # Return list of random coordinates
    return crd_rnd


# If integrating with other XML then only some attributes are needed, the
# world body & bodies.
# To integrate this xml with main xml:
# 1) Add the xml that this script creates to the same folder as the main
# xml.
# 2) In main xml, underneath line <mujoco model="...">, add:
#     <include file="xml_created_by_make_objects.m" />

# solo xml file (1)
# integrate xml into other project (0)
solo_flag = 0

# object to be removed if at undesired position
# e.g. starting position of robot shouldn't have any obstacles
remove_undes = 0
undes_coord = [0, 0, 0]

# Create grid for objects to be placed
# always +1 to account for object that represents the GOAL
num_objects = 6  # num_objects_desired + 1

# robot diagonal largest (diagonal) length to clear all objects
robot_diagL = 1  # meters
spacing = 3 * robot_diagL  # from center to center

plane_width = 0.5 * num_objects * spacing

# assume robot_diagL for radius
obj_radius = 1 * robot_diagL

obj_center = random_2d_coordinates(num_objects, spacing, plane_width)

# Find undesired object in list
undes_coord_id = None
for i in range(len(obj_center)):
    if obj_center[i][0] == undes_coord[0] and obj_center[i][1] == undes_coord[1]:
        remove_undes = 1
        undes_coord_id = i

# Remove undesired object from list
if remove_undes:
    obj_center.pop(undes_coord_id)
    num_objects = num_objects - 1

# plot
# Calculate bounds
bounds = [0.5 * plane_width * val for val in (-1, 1)]

# Plot
plt.xlim(bounds)
plt.ylim(bounds)
plt.axis('equal')

# visualize grid for objects
plt.gca().add_patch(Rectangle((bounds[0], bounds[0]), plane_width, plane_width, linewidth=4, edgecolor='r'))

# mujoco, x is forward, y is sideways
plt.plot([obj[1] for obj in obj_center], [obj[0] for obj in obj_center], linestyle='None', marker='.', markersize=20,
         color='k')

for obj in obj_center:
    # mujoco, +x is forward, +y is to the left
    circle = Circle((obj[1], obj[0]), obj_radius, facecolor='none', edgecolor='k')
    plt.gca().add_patch(circle)

plt.show()

# XML creation

# Reference mujoco xml documentation to create xml
# https://mujoco.readthedocs.io/en/latest/XMLreference.html

# create root element
toc = ET.Element('mujoco')
toc.set('model', 'objects')

if solo_flag:
    # set compiler settings
    compiler = ET.SubElement(toc, 'compiler')
    compiler.set('angle', 'radian')
    compiler.set('eulerseq', 'xyz')

    # set option settings
    option = ET.SubElement(toc, 'option')
    option.set('gravity', '0 0 -9.81')

    # set assets - material and textures on floor
    asset = ET.SubElement(toc, 'asset')
    material = ET.SubElement(asset, 'material')
    material.set('name', 'plane')
    material.set('reflectance', '0')
    material.set('texrepeat', '1 1')
    material.set('texture', 'plane')
    material.set('texuniform', 'true')
    texture = ET.SubElement(asset, 'texture')
    texture.set('builtin', 'checker')
    texture.set('height', '512')
    texture.set('name', 'plane')
    texture.set('rgb1', '0.6 0.8 1')
    texture.set('rgb2', '0.4 0.4 0.4')
    texture.set('type', '2d')
    texture.set('width', '512')

    # create worldbody
    worldbody = ET.SubElement(toc, 'worldbody')

    # set light settings
    light = ET.SubElement(worldbody, 'light')
    light.set('directional', 'true')
    light.set('diffuse', '.8 .8 .8')
    light.set('pos', '0 0 10')
    light.set('dir', '0 0 -10')

    # create floor
    floor = ET.SubElement(worldbody, 'geom')
    floor.set('name', 'floor')
    floor.set('type', 'plane')
    floor.set('conaffinity', '1')
    floor.set('condim', '3')
    floor.set('contype', '1')
    floor.set('rgba', '0.5 0.9 0.9 0.1')
    floor.set('material', 'plane')
    floor.set('pos', '0 0 0')
    floor.set('size', '0 0 1')
else:
    # create worldbody
    worldbody = ET.SubElement(toc, 'worldbody')

# SET ACCORDING TO THE NUMBER OF NEEDED OBJECTS
for i in range(num_objects):
    # object names
    name = 'object' + str(i + 1) if i != num_objects - 1 else 'goal'

    # object position
    pos_rand_z = round(random.uniform(0, 1), 1) + 0.1
    pos = str(obj_center[i][0]) + ' ' + str(obj_center[i][1]) + ' ' + str(
        pos_rand_z) if i != num_objects - 1 else '10 10 2.5'

    # object rotational position
    euler = '0 0 0'

    # object mass
    mass_rand = 10 * round(random.uniform(0, 1), 1) + 1
    mass = str(mass_rand)

    # object size
    size = str(robot_diagL) + ' ' + str(robot_diagL) + ' ' + str(pos_rand_z) if i != num_objects - 1 else '0.5 0.5 0.5'

    # object color
    color_rand = [round(random.uniform(0, 1), 1) for _ in range(3)] + [1]
    rgba = ' '.join(map(str, color_rand)) if i != num_objects - 1 else '0.1 0.9 0.1 0.1'

    # set the object attributes
    body = ET.SubElement(worldbody, 'body')
    body.set('name', name)
    body.set('pos', pos)
    body.set('euler', euler)

    # set inertial attributes
    inertial = ET.SubElement(body, 'inertial')
    inertial.set('pos', '0 0 0')
    inertial.set('mass', mass)
    inertial.set('diaginertia', '0.01 0.01 0.01')

    # set joint attributes
    joint = ET.SubElement(body, 'joint')
    joint.set('type', 'free')

    # set geom attributes
    geom = ET.SubElement(body, 'geom')
    geom.set('type', 'box')
    geom.set('size', size)
    geom.set('rgba', rgba)

    # set site attributes
    site = ET.SubElement(body, 'site')
    site.set('name', name + '_site')
    site.set('pos', '0 0 0')
    site.set('quat', '1 0 0 0')
    site.set('size', '0.01')

# create sensor
sensor = ET.SubElement(toc, 'sensor')

# set sensors for objects
for i in range(num_objects):
    # choose sensors from https://mujoco.readthedocs.io/en/latest/XMLreference.html#sensor
    framepos = ET.SubElement(sensor, 'framepos')

    # get object info
    name = 'object' + str(i + 1) + '_pos' if i != num_objects - 1 else 'goal_pos'
    objname = 'object' + str(i + 1) if i != num_objects - 1 else 'goal'

    # set framepos sensor attributes
    framepos.set('name', name)
    framepos.set('objtype', 'site')
    framepos.set('objname', objname + '_site')

# create and write to XML file
tree = ET.ElementTree(toc)

# Save XML to a string
xml_string = ET.tostring(tree.getroot(), encoding="utf-8")

# Format XML string
dom = minidom.parseString(xml_string)
formatted_xml = dom.toprettyxml(indent="  ")

# Save formatted XML to a file
with open('objects.xml', 'w') as file:
    file.write(formatted_xml)

# Print the formatted XML file contents
print(formatted_xml)
