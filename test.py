from driver import RealSenseCamera
import pyrealsense2 as rs
import cv2 as cv
import numpy as np

camera = RealSenseCamera()
# data = camera.get_frame_from_file('data/1-1/')
data = camera.get_frame_data()

depth = data['depth_accuracy']
# depth = data['depth_accuracy']//20
# unique, counts = np.unique(depth.flatten(), return_counts=True)
# print(unique*20, counts)

# depth = data['depth_accuracy']
# depth[depth>550] = 0


# nonzero_indices = np.nonzero(depth)

# center_x = int(np.mean(nonzero_indices[1]))
# center_y = int(np.mean(nonzero_indices[0]))

# print("Центр координат ненулевых элементов:", (center_x, center_y))

# depth[center_y][center_x] = 0

cv.imshow('image', np.uint8(cv.normalize(depth, None, 0, 255, cv.NORM_MINMAX)))
cv.waitKey()





# depth_intrinsics = camera.get_intrinsics()

# pixel = [320, 240]  # Пример координат пикселя (центр изображения)
# depth = depth_frame.get_distance(pixel[0], pixel[1])  # Получаем глубину для этого пикселя

# # Преобразуем 2D координаты в 3D
# point_3d = rs.rs2_deproject_pixel_to_point(depth_intrinsics, pixel, depth)