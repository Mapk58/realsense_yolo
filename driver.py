import pyrealsense2 as rs
import numpy as np
import cv2
import os
import warnings
warnings.filterwarnings("ignore", category=RuntimeWarning)


class RealSenseCamera:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        
        self.pipeline.start(self.config)
        self.depth_sensor = self.pipeline.get_active_profile().get_device().first_depth_sensor()
        self.depth_sensor.set_option(rs.option.visual_preset, 3) # 0 - default, 1 - hand, 2 - accuracy, 3 - density, 4 - medium density
        self.depth_sensor.set_option(rs.option.emitter_enabled, 1) # 0 - off, 1 - on, 2 - auto
        align_to = rs.stream.color
        self.align = rs.align(align_to)

    # def get_intrinsics(self):
    #     frames = self.pipeline.wait_for_frames()
    #     depth_frame = frames.get_depth_frame()
    #     depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
    #     return depth_intrinsics

    def get_frame_from_file(self, path=""):
        data = {}
        data['color_image'] = np.load(path + '/color_image.npy')
        data['depth_accuracy'] = np.load(path + '/depth_accuracy.npy')
        data['depth_density'] = np.load(path + '/depth_density.npy')
        return data
    
    def get_frame_data(self):
    
        def make_mask(img):
            return np.where(img == 0, 0, 1.0)

        self.depth_sensor.set_option(rs.option.visual_preset, 3)        
        full_depth = np.zeros((480, 640))
        full_mask = np.zeros((480, 640))

        for j in range(10): # тут можно сделать меньше
            for i in range(5): # тут возможно можно сделать меньше
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                aligned_depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()

            full_depth += np.array(aligned_depth_frame.get_data())
            full_mask += make_mask(np.array(aligned_depth_frame.get_data()))

        full_mask[full_mask == 0] = 1
        dpth = np.divide(full_depth, full_mask)
        depth_image = dpth.astype(np.uint16)
        color_temp = np.asarray(color_frame.get_data())
        color_image = color_temp
        
        data = {}
        data['color_image'] = np.array(color_image)
        data['depth_density'] = np.array(depth_image)

        self.depth_sensor.set_option(rs.option.visual_preset, 2)        
        full_depth = np.zeros((480, 640))
        full_mask = np.zeros((480, 640))

        for j in range(10): # тут можно сделать меньше
            for i in range(5): # тут возможно можно сделать меньше
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                aligned_depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()

            full_depth += np.array(aligned_depth_frame.get_data())
            full_mask += make_mask(np.array(aligned_depth_frame.get_data()))

        full_mask[full_mask == 0] = 1
        dpth = np.divide(full_depth, full_mask)
        depth_image = dpth.astype(np.uint16)
        color_temp = np.asarray(color_frame.get_data())
        color_image = color_temp

        data['depth_accuracy'] = np.array(depth_image)
        
        return data

    def save_frame_data(self, frame_name, data):
        os.makedirs(frame_name, exist_ok=True)

        if data['color_image'] is not None:
            cv2.imwrite(os.path.join(frame_name, 'color_image.png'), data['color_image'])
            np.save(os.path.join(frame_name, 'color_image.npy'), data['color_image'])

        normalized_depth_accuracy = cv2.normalize(data['depth_accuracy'], None, 0, 255, cv2.NORM_MINMAX)
        normalized_depth_accuracy = np.uint8(normalized_depth_accuracy)
        cv2.imwrite(os.path.join(frame_name, f'depth_accuracy.png'), normalized_depth_accuracy)
        np.save(os.path.join(frame_name, f'depth_accuracy.npy'), data['depth_accuracy'])

        normalized_depth_density = cv2.normalize(data['depth_density'], None, 0, 255, cv2.NORM_MINMAX)
        normalized_depth_density = np.uint8(normalized_depth_density)
        cv2.imwrite(os.path.join(frame_name, f'depth_density.png'), normalized_depth_density)
        np.save(os.path.join(frame_name, f'depth_density.npy'), data['depth_density'])

    def stop(self):
        self.pipeline.stop()


# from driver import RealSenseCamera
# Пример использования класса
if __name__ == "__main__":
    camera = RealSenseCamera()
    data = camera.get_frame_data()
    camera.save_frame_data('frame_data', data)
    camera.stop()
