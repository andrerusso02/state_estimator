import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
from ellipsoid_fit import fit_ellipsoid, apply_calibration

class CalibrationController(Node):
    def __init__(self, topic_name='/imu/mag'):
        super().__init__('mag_calibrator_node')
        self.subscription = self.create_subscription(
            MagneticField, topic_name, self.listener_callback, 10
        )
        self.is_acquiring = False
        self.raw_data = []
        self.hard_iron = None
        self.soft_iron = None

    def listener_callback(self, msg):
        if self.is_acquiring:
            self.raw_data.append([msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z])

    def start_acquisition(self):
        self.is_acquiring = True

    def stop_acquisition(self):
        self.is_acquiring = False
        return len(self.raw_data)

    def clear_data(self):
        self.raw_data.clear()
        self.hard_iron = None
        self.soft_iron = None

    def perform_calibration(self):
        if len(self.raw_data) < 20:
            raise ValueError("Insufficient data points for calibration. Please collect more data.")
        
        bias, matrix = fit_ellipsoid(np.array(self.raw_data))
        self.hard_iron = bias
        self.soft_iron = matrix
        return bias, matrix

    def get_plot_data(self, apply_cal=False, downsample=10):
        if not self.raw_data:
            return None
        
        # Downsample for UI performance
        data = np.array(self.raw_data)[::downsample]
        
        if apply_cal and self.hard_iron is not None:
            data = apply_calibration(data, self.hard_iron, self.soft_iron)
        
        return data