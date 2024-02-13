#! /usr/bin/env python3

import rospy
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge
import os
import torch
import math
from sklearn.cluster import KMeans
import warnings
warnings.filterwarnings("ignore")

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Point, Vector3
from ur5.srv import VisionService, VisionServiceResponse

# Record for blocks data
blocks_data = []

start = False

# Constants
table_height = 0.88
tolerance_error = 0.001
block_error = 0.001
value_error = 0.001
distance_error = 0.005

# "block_short_side", "block_long_side", "block_height", values for each block class of the dataset
blocks_values = {
    'X1-Y1-Z2': [0.031, 0.031, 0.057],
    'X1-Y2-Z1': [0.031, 0.062, 0.038], 
    'X1-Y2-Z2-CHAMFER': [0.031, 0.062, 0.057],
    'X1-Y2-Z2-TWINFILLET': [0.031, 0.062, 0.057],
    'X1-Y2-Z2': [0.031, 0.062, 0.057],
    'X1-Y3-Z2-FILLET': [0.031, 0.095, 0.057],
    'X1-Y3-Z2': [0.031, 0.095, 0.057],
    'X1-Y4-Z1': [0.031, 0.127, 0.038],
    'X1-Y4-Z2': [0.031, 0.127, 0.057],
    'X2-Y2-Z2-FILLET': [0.063, 0.063, 0.057],
    'X2-Y2-Z2': [0.062, 0.063, 0.057],
}

# Color ranges used for masking the image
color_ranges = {
            'red': [(0, 50, 50), (10, 255, 255)],
            'green': [(36, 50, 50), (70, 255, 255)],
            'blue': [(90, 50, 50), (130, 255, 255)],
            'yellow': [(20, 50, 50), (35, 255, 255)],
            'fuchsia': [(145, 50, 50), (175, 255, 255)],
            'orange': [(11, 50, 50), (25, 255, 255)]
        }

# Function that returns given block values (block_short_side, block_long_side, block_height)
def get_block_values(name):
    value = np.array(blocks_values[name])
    return value[0], value[1], value[2]

# Function that applies color filtering on the image to return 2D points within the given block's bounding box
def get_block_points(image, block):
    # Create a mask to filter out colors in the image
    mask = np.zeros(image.shape[:2],dtype=np.uint8)
    image_hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    
    # Apply the mask to the image
    for color_range in color_ranges.values():
        lower_color = np.array(color_range[0])
        upper_color = np.array(color_range[1])
        color_mask = cv.inRange(image_hsv, lower_color, upper_color)
        mask = cv.bitwise_or(mask, color_mask)

    # Apply the mask to the original image
    result = cv.bitwise_and(image, image, mask=mask)
    gray_image = cv.cvtColor(result, cv.COLOR_BGR2GRAY)
    mask = cv.inRange(gray_image, 1, 255)

    # Find non-black points in the mask
    non_black_points = cv.findNonZero(mask)

    block_points_2D = []

    # If non-black points are found
    if non_black_points is not None:
        # Iterate through each non-black point
        for point in non_black_points:
            x, y = point[0]
            # Adjust the coordinates to be relative to the block's position
            block_points_2D.append([x + int(block[2]), y + int(block[3])])
        
    return block_points_2D

def convert_2D_to_3D(point_cloud2_msg, zed_points):
    # Transforming 2D points in 3D points (of the boundary box)
    points_3d = point_cloud2.read_points(point_cloud2_msg, field_names=['x','y','z'], skip_nans=False, uvs=zed_points)

    # selection of informations from point cloud
    points_from_zed = []
    for point in points_3d:
        points_from_zed.append(point[:3])

    return points_from_zed
    
# Function that gets the coordinates of 3 vertices of the block's lowest surface
def get_vertices(wf_points):
    points=np.array(wf_points)

    # Get only the block's lower surface points -> block's points close to the table's surface
    z_value = table_height + tolerance_error
    l_surface_points = points[abs(points[:,2] - z_value) <= block_error]

    # Get the indexes of the points with the desired values (min y, max y, min x)
    miny_index = np.argmin(l_surface_points[:,1])
    maxy_index = np.argmax(l_surface_points[:,1])
    minx_index = np.argmin(l_surface_points[:,0])

    # Get the values of min y, max y and min x
    miny_value = l_surface_points[miny_index, 1]
    maxy_value = l_surface_points[maxy_index, 1]
    minx_value = l_surface_points[minx_index, 0]

    # Get only the points of the lower surface whose values are close to the one found above
    miny_points = l_surface_points[abs(l_surface_points[:,1] - miny_value) <= value_error]
    maxy_points = l_surface_points[abs(l_surface_points[:,1] - maxy_value) <= value_error]
    minx_points = l_surface_points[abs(l_surface_points[:,0] - minx_value) <= value_error]

    # Get the indexes of the points
    miny_index_f = np.argmin(miny_points[:,0])
    maxy_index_f = np.argmin(maxy_points[:,0])
    minx_index_f = np.argmin(minx_points[:,1])

    # Get the coordinates of the three points
    miny_minx = miny_points[miny_index_f]
    maxy_minx = maxy_points[maxy_index_f]
    minx = minx_points[minx_index_f]

    return miny_minx, maxy_minx, minx

# Function that calculates the angular value phi
def get_block_pose(miny_minx, maxy_minx, minx):
    # Coordinates of three vertices of the lower surface of the block
    miny_minx = np.array(miny_minx)
    maxy_minx = np.array(maxy_minx)
    minx = np.array(minx)

    miny_minx_dist = math.dist(miny_minx, minx)

    if miny_minx_dist <= distance_error:
        configuration = 1
    else:
        if maxy_minx[0] <= miny_minx[0]:
            configuration = 2
        else:
            configuration = 3
    
    if configuration == 2:
        phi = abs(math.atan2(minx[1]-miny_minx[1],miny_minx[0]-minx[0]))
        return phi
    elif configuration == 3:
        phi = abs(math.atan2(minx[1]-miny_minx[1],miny_minx[0]-minx[0]))
        return phi - np.pi/2
    elif configuration == 1:
        return 0

# Function that gets the coordinates of the central point of the block
def get_block_center_point(min_x, phi, block_short_side, block_long_side, block_height):
    x = min_x[0] + (block_long_side * math.cos(phi) + block_short_side * math.sin(abs(phi)))/2
    y = min_x[1] - np.sign(phi) * (block_short_side * math.cos(phi) - block_long_side*math.sin(abs(phi)))/2
    z = table_height + block_height/2

    return x, y, z

# Function that stores block's data
def store_block_data(name, x, y, z, phi):
    blocks_data.append([name, x, y, z, phi])

def object_detection(image_msg: Image, point_cloud2_msg: PointCloud2, model) -> None:
    # convert received image (bgr8 format) to a cv2 image
    img = CvBridge().imgmsg_to_cv2(image_msg, "bgr8")

    # Apply model to the image
    results = model(img)

    # Get bounding box information of each block in the image
    blocks = []
    bboxes = results.pandas().xyxy[0].to_dict(orient="records")
    for bbox in bboxes:
        name = bbox['name']
        block_short_side, block_long_side, block_height = get_block_values(name)

        conf = bbox['confidence']
        x1 = int(bbox['xmin'])
        y1 = int(bbox['ymin'])
        x2 = int(bbox['xmax'])
        y2 = int(bbox['ymax'])

        blocks.append((name, conf, x1, y1, x2, y2, block_short_side, block_long_side, block_height))

    # Localize each detected block and get its pose with respect to the world frame
    for block in blocks:
        # Crop image -> take only the points inside block's bounding box
        image = img[block[3]:block[5], block[2]:block[4]]

        # Get 2D points of the block by filtering the image through a mask
        block_points_2D = get_block_points(image, block)

        # Convert from a list of tuples to a list of lists
        zed_points = []
        for point in list(block_points_2D):
            zed_points.append([int(coordinate) for coordinate in point])

        # Convert 2D points to 3D points using point cloud data
        block_points_3D = convert_2D_to_3D(point_cloud2_msg, zed_points)

        # Rotational matrix to go from the zed frame to the world frame
        rot_matrix = np.array([[0., -0.49948, 0.86632],
                                [-1., 0., 0.],
                                [0., -0.86632, -0.49948]])
                    
        # Zed camera's position from the world frame
        zed_pose = np.array([-0.4, 0.59, 1.4])

        # Convert each block's point from camera frame to world frame
        wf_block_points = []
        for point in block_points_3D:
            point = rot_matrix.dot(point) + zed_pose
            point = np.array(point)
            wf_block_points.append(point)

        # Get block's data and store it
        vertices_result = get_vertices(wf_block_points)
        miny_minx, maxy_minx, minx = vertices_result
        phi = get_block_pose(miny_minx, maxy_minx, minx)
        x, y, z = get_block_center_point(minx, phi, block[6], block[7], block[8]) # block[6] = block_short_side, block[7] = block_long_side, block[8] = block_height
        store_block_data(block[0], x, y, z, phi) # block[0] = block's class name
        

# Function that prepares blocks's information as a response to the request sent by the task planner node
def send_blocks_info(req):
    global start, blocks_data
    start = req.start

    i = 0

    res = VisionServiceResponse()
    for block in blocks_data:
        i = i+1
        vector = Vector3()
        vector.x = 0
        vector.y = 0
        vector.z = block[4]
        res.block_orientation.append(vector)

        position = Point()
        position.x = block[1]
        position.y = block[2]
        position.z = block[3]
        res.block_position.append(position)

        res.class_of_block.append(block[0])
    
    res.n_blocks = i

    return res

if __name__ ==  '__main__':
    # Vision node initialization
    rospy.init_node('vision_node')

    # Path creation
    home_path = os.path.expanduser('~')
    path_yolo = os.path.join(home_path, "ros_ws/src/ur5/vision/yolov5")
    path_vision = os.path.join(home_path, "ros_ws/src/ur5/vision")
    path_weigths = os.path.join(path_vision, 'best.pt')

    # Model loading
    model = torch.hub.load(path_yolo, "custom", path_weigths, source='local')

    # Vision node gets the raw image from the zed node
    image = rospy.wait_for_message("/ur5/zed_node/left_raw/image_raw_color", Image)
    # Vision node gets the pointcloud from the zed node
    point_cloud2_msg = rospy.wait_for_message("/ur5/zed_node/point_cloud/cloud_registered", PointCloud2)

    # Detect and estimate the pose of the objects in the image sent by the zed node
    object_detection(image, point_cloud2_msg, model)

    # Send data to task planner node
    s = rospy.Service('Vision', VisionService, send_blocks_info)

    print("VISION NODE SHUTDOWN")
    
    while not start and not rospy.is_shutdown():
        pass
