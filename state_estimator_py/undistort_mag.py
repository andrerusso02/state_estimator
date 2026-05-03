import numpy as np

hard_iron_bias = np.array([-0.39529558,  0.3067545 , -0.12118412])

soft_iron_matrix = np.array([[2.32441508, 0.12046236, 0.07802267],
 [0.12046236, 2.35003198, 0.03071853],
 [0.07802267, 0.03071853, 2.38180137]])


def undistort_mag(mag_data):
    """
    Applies hard and soft iron calibration to raw magnetometer data.
    mag_data: [x, y, z] raw magnetometer reading
    Returns calibrated magnetometer reading.
    """

    # Calculate the 'gain' of your current matrix
    det = np.linalg.det(soft_iron_matrix)
    avg_scale = np.power(det, 1/3)
    # Create a neutral matrix that corrects shape but preserves scale
    # We do that because the calibration scales the sphere to 1.
    soft_iron_neutral = soft_iron_matrix / avg_scale

    mag_vector = np.array(mag_data)
    # Apply hard iron correction
    mag_corrected = mag_vector - hard_iron_bias
    # Apply soft iron correction
    mag_calibrated = np.dot(soft_iron_neutral, mag_corrected)
    return mag_calibrated
    