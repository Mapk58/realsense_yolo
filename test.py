from driver import RealSenseCamera
import pyrealsense2 as rs
import cv2 as cv
import numpy as np

def random_samples(data, k=0.5):
    x, y = data
    combined = np.column_stack((x, y))
    num_to_keep = int(len(combined) * k)
    indices = np.random.choice(len(combined), size=num_to_keep, replace=False)
    reduced_coordinates = combined[indices]
    x_reduced, y_reduced = reduced_coordinates[:, 0], reduced_coordinates[:, 1]
    final_coordinates = (x_reduced, y_reduced)
    return final_coordinates

def rgb_to_3d(x, y, depth):
    _intrinsics = rs.intrinsics()
    # _intrinsics.width = 640
    # _intrinsics.height = 480
    # _intrinsics.fx = 608.673828125
    # _intrinsics.fy = 607.3576049804688
    # _intrinsics.ppx = 321.5899963378906
    # _intrinsics.ppy = 252.52552795410156
    _intrinsics.width = 640
    _intrinsics.height = 480
    _intrinsics.fx = 604.6063232421875
    _intrinsics.fy = 604.6441650390625
    _intrinsics.ppx = 315.34197998046875
    _intrinsics.ppy = 233.6892852783203
    result = rs.rs2_deproject_pixel_to_point(_intrinsics, [x, y], depth[y][x])
    return result[0], -result[1], result[2]

def find_nonzero_center(depth):
    nonzero_indices = np.nonzero(depth)
    center_x = int(np.mean(nonzero_indices[1]))
    center_y = int(np.mean(nonzero_indices[0]))
    return center_x, center_y

def find_max_table_height(depth):
    unique, counts = np.unique((depth//20).flatten(), return_counts=True)
    return unique[1]*20 + 30

width = 640
height = 480

folders = [
    '1__213-168__9554',
    '1__213-293__9633',
    '1__213-418__9725',
    '1__338-243__8875',
    '1__388-168__9472',
    '1__388-393__9394',
    '1__513-243__9233',
    '1__513-393__9318',
    '2__163-343__157',
    '2__163-443__219',
    '2__363-343__86',
    '2__363-443__9992',
    '2__513-243__9858',
    '2__513-443__9926'
]

error_x = []
error_y = []

for fld in folders:
    # camera = RealSenseCamera()
    data = RealSenseCamera.get_frame_from_file(f'data/{fld}')
    depth = data['depth_density']
    # print(camera.get_frame_data()['intrinsics'])

    depth[depth>find_max_table_height(depth)] = 0

    projection = np.zeros(depth.shape, dtype=np.uint8)
    nonzero_indices = np.nonzero(depth)
    nonzero_indices_reduced = random_samples(nonzero_indices, 1)
    points_3d = [[],[],[]]
    for y, x in zip(nonzero_indices_reduced[0], nonzero_indices_reduced[1]):
        point = rgb_to_3d(x, y, depth)
        points_3d[0].append(point[0])
        points_3d[1].append(point[1])
        points_3d[2].append(point[2])
        projection[height//2 - int(point[1]), width//2 + int(point[0])] = -point[2]
    points_3d = np.array(points_3d)
    center_x = int(np.mean(points_3d[0]))
    center_y = int(np.mean(points_3d[1]))
    calc_x = center_x + 281
    calc_y = center_y - 292

    x, y = fld.split('__')[1].split('-')
    x, y = int(x), int(y)

    print((x-calc_x)/10, (y+calc_y)/10)
    error_x.append(abs(x-calc_x)/10)
    error_y.append(abs(y+calc_y)/10)
    # print(f"{x, y} -> {center_x, center_y}")


    # print(find_nonzero_center(projection))

    # if True:
    #     cv.imshow('image', np.uint8(cv.normalize(depth, None, 0, 255, cv.NORM_MINMAX)))
    #     cv.imshow('projection', np.uint8(cv.normalize(projection, None, 0, 255, cv.NORM_MINMAX)))
    #     cv.waitKey()


print(np.mean(np.array(error_x)), np.mean(np.array(error_y)))