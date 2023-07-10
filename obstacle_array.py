"""

Creates Obstacle Array for Dynamic Window Approach

By: Daniel Torres

"""

import math
import numpy as np
import time
import xml.etree.ElementTree as et
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# xml filename to extract obstacles
xml_filename = 'objects_new.xml'


def xml_parser(filename):
    """"
        obtain position and size of obstacles from xml
    """
    # Parse the XML file
    tree = et.parse(filename)
    root = tree.getroot()

    obj_pos = []
    obj_size = []

    # Iterate over the XML structure
    for child in root.iter('worldbody'):
        for child_sub1 in child:
            if child_sub1.attrib.get('name') != 'goal':
                pos = child_sub1.attrib.get('pos')  # Get the 'pos' attribute value

                # Split the position values and convert them to floats
                pos_arr = np.array([float(pos_str) for pos_str in pos.split()]).reshape((1, 3))

                obj_pos.append(pos_arr)

                for child_sub2 in child_sub1:
                    if child_sub2.tag == 'geom':
                        if child_sub2.attrib.get('type') == 'box':
                            size = child_sub2.attrib.get('size')  # Get the 'size' attribute value

                            # Split the size values and convert them to floats
                            size_arr = np.array([float(size_str) for size_str in size.split()]).reshape((1, 3))

                            obj_size.append(size_arr)
                        elif child_sub2.attrib.get('type') == 'cylinder':
                            size = child_sub2.attrib.get('size')  # Get the 'size' attribute value

                            # Split the size values and convert them to floats
                            size_arr = np.array([float(size_str) for size_str in size.split()]).reshape((1, 2))

                            obj_size.append(size_arr)

    # Combine the position arrays into a single array
    obj_pos = np.concatenate(obj_pos, axis=0)

    # Combine the size arrays into a single array
    obj_size = np.concatenate(obj_size, axis=0)

    return obj_pos, obj_size


def object_placement(filename):
    """
        obtains obstacles from xml
    """

    # get object COM position and size from xml
    obj_pos_xml, obj_size_xml = xml_parser(filename)

    obj_arr = []

    for i in range(len(obj_pos_xml[:, 0])):
        # extract x and y position and size
        obj_pos = obj_pos_xml[i, :2]
        obj_size = obj_size_xml[i, :2]

        num_markers = 5j  # number of object markers to place
        padding = 0  # add some padding to ensure that robot doesn't hit obstacle

        # Create mesh grid for obstacles
        x, y = np.mgrid[obj_pos[0] - obj_size[0] - padding: obj_pos[0] + obj_size[0] + padding:num_markers,
               obj_pos[1] - obj_size[1] - padding: obj_pos[1] + obj_size[1] + padding:num_markers]

        # Define the mask condition to identify interior points
        mask = (x <= obj_pos[0] - obj_size[0]) | (x >= obj_pos[0] + obj_size[0]) | \
               (y <= obj_pos[1] - obj_size[1]) | (y >= obj_pos[1] + obj_size[1])

        # Filter out interior points using boolean indexing
        x = x[mask]
        y = y[mask]

        obj_temp = np.round(np.vstack([x.ravel(), y.ravel()]).T, 2)

        obj_arr.append(obj_temp)

    obj_arr = np.vstack(obj_arr)

    return obj_arr


def plot_robot(x, y, length, width):
    """
        Plots robot at starting position
    """
    outline = np.array([[-length / 2, length / 2,
                         (length / 2), -length / 2,
                         -length / 2],
                        [width / 2, width / 2,
                         - width / 2, -width / 2,
                         width / 2]])
    outline[0, :] += x
    outline[1, :] += y
    plt.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(), "-k")


def main():
    obj_pos_ = object_placement(xml_filename)

    # prints obstacle array for DWA code
    print('self.ob = np.array([[' + str(obj_pos_[0, 0]) + ', ' + str(obj_pos_[0, 1]) + '],')
    for i in range(1, len(obj_pos_[:, 0]) - 1):
        print('                    [' + str(obj_pos_[i, 0]) + ', ' + str(obj_pos_[i, 1]) + '],')
    print('                    [' + str(obj_pos_[-1, 0]) + ', ' + str(obj_pos_[-1, 1]) + ']])')

    # get actual obstacles for plotting
    objpos_shape, objsize_shape = xml_parser(xml_filename)
    objpos_shape = objpos_shape[:, :2]
    objsize_shape = objsize_shape[:, :2]

    # start position [x(m), y(m)]
    start = np.array([0.0, 0.0])

    # goal position [x(m), y(m)]
    goal = np.array([10.0, 10.0])

    # robot
    robot_len = 0.7
    robot_wid = 0.4

    # plot environment
    plt.plot(obj_pos_[:, 0], obj_pos_[:, 1], 'ko')
    plt.plot(start[0], start[1], "xr")
    plt.plot(goal[0], goal[1], "xb")
    plot_robot(start[0], start[1], robot_len, robot_wid)
    for i in range(len(objpos_shape)):
        rect = mpatches.Rectangle((objpos_shape[i, 0]-objsize_shape[i, 0], objpos_shape[i, 1]-objsize_shape[i, 1]), 2*objsize_shape[i, 0], 2*objsize_shape[i, 1],
                                  fill=True,
                                  color="purple",
                                  linewidth=2,
                                  label='Obstacle')
        # circle = mpatches.Circle((objpos_shape[i, 1], objpos_shape[i, 0]),
        #                          radius=objsize_shape[i, 0],
        #                          fill=True,
        #                          color="purple",
        #                          linewidth=2)
        plt.gca().add_patch(rect)
        # plt.gca().add_patch(circle)
    circle = mpatches.Circle((goal[0], goal[1]), radius=0.85,
                             fill=True,
                             color="green",
                             linewidth=2,
                             label='Goal')
    plt.gca().add_patch(circle)
    plt.legend(handles=[circle, rect])
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.axis("equal")
    plt.grid(True)
    plt.show()


if __name__ == '__main__':
    main()
