import numpy as np
import os

class CameraCalibration:
    def __init__(self, parameter_folder):
        # Paths to the .dat files in the `camera_parameters` folder
        camera0_intrinsic_file = os.path.join(parameter_folder, 'camera0_intrinsics.dat')
        camera0_rot_trans_file = os.path.join(parameter_folder, 'camera0_rot_trans.dat')
        camera1_intrinsic_file = os.path.join(parameter_folder, 'camera1_intrinsics.dat')
        camera1_rot_trans_file = os.path.join(parameter_folder, 'camera1_rot_trans.dat')
        
        # Camera 0 parameters
        self.camera0_intrinsic_matrix = None
        self.camera0_distortion_coeffs = None
        self.camera0_rotation_matrix = None
        self.camera0_translation_vector = None
        
        # Camera 1 parameters
        self.camera1_intrinsic_matrix = None
        self.camera1_distortion_coeffs = None
        self.camera1_rotation_matrix = None
        self.camera1_translation_vector = None
        
        # Load calibration data for both cameras
        self.load_intrinsic(camera0_intrinsic_file, camera_num=0)
        self.load_rot_trans(camera0_rot_trans_file, camera_num=0)
        self.load_intrinsic(camera1_intrinsic_file, camera_num=1)
        self.load_rot_trans(camera1_rot_trans_file, camera_num=1)

    def load_intrinsic(self, filename, camera_num):
        with open(filename, 'r') as f:
            lines = f.readlines()
            intrinsic_data = lines[1:4]
            distortion_data = lines[5]
            
            # Intrinsic Matrix (3x3)
            intrinsic_matrix = np.array([
                [float(val) for val in intrinsic_data[0].split()],
                [float(val) for val in intrinsic_data[1].split()],
                [float(val) for val in intrinsic_data[2].split()]
            ])
            
            # Distortion coefficients
            distortion_coeffs = np.array([float(val) for val in distortion_data.split()])
            
            if camera_num == 0:
                self.camera0_intrinsic_matrix = intrinsic_matrix
                self.camera0_distortion_coeffs = distortion_coeffs
            elif camera_num == 1:
                self.camera1_intrinsic_matrix = intrinsic_matrix
                self.camera1_distortion_coeffs = distortion_coeffs

    def load_rot_trans(self, filename, camera_num):
        with open(filename, 'r') as f:
            lines = f.readlines()
            rotation_data = lines[1:4]
            translation_data = lines[5:8]
            
            # Rotation matrix (3x3)
            rotation_matrix = np.array([
                [float(val) for val in rotation_data[0].split()],
                [float(val) for val in rotation_data[1].split()],
                [float(val) for val in rotation_data[2].split()]
            ])
            
            # Translation vector (3x1)
            translation_vector = np.array([float(val) for val in translation_data])
            
            if camera_num == 0:
                self.camera0_rotation_matrix = rotation_matrix
                self.camera0_translation_vector = translation_vector
            elif camera_num == 1:
                self.camera1_rotation_matrix = rotation_matrix
                self.camera1_translation_vector = translation_vector

    def get_camera0_intrinsic(self):
        return self.camera0_intrinsic_matrix

    def get_camera0_distortion(self):
        return self.camera0_distortion_coeffs

    def get_camera0_rotation(self):
        return self.camera0_rotation_matrix

    def get_camera0_translation(self):
        return self.camera0_translation_vector

    # Methods to retrieve parameters for Camera 1
    def get_camera1_intrinsic(self):
        return self.camera1_intrinsic_matrix

    def get_camera1_distortion(self):
        return self.camera1_distortion_coeffs

    def get_camera1_rotation(self):
        return self.camera1_rotation_matrix

    def get_camera1_translation(self):
        return self.camera1_translation_vector

# Example usage of the class
calibration = CameraCalibration('camera_parameters')

# Camera 0 parameters
print("Camera 0 - Intrinsic Matrix:\n", calibration.get_camera0_intrinsic())
print("Camera 0 - Distortion Coefficients:\n", calibration.get_camera0_distortion())
print("Camera 0 - Rotation Matrix:\n", calibration.get_camera0_rotation())
print("Camera 0 - Translation Vector:\n", calibration.get_camera0_translation())

# Camera 1 parameters
print("Camera 1 - Intrinsic Matrix:\n", calibration.get_camera1_intrinsic())
print("Camera 1 - Distortion Coefficients:\n", calibration.get_camera1_distortion())
print("Camera 1 - Rotation Matrix:\n", calibration.get_camera1_rotation())
print("Camera 1 - Translation Vector:\n", calibration.get_camera1_translation())
