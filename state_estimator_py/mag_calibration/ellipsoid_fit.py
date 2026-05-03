import numpy as np
import scipy.linalg

def fit_ellipsoid(mag_data):
    """
    Fits an ellipsoid to a 3D point cloud and calculates calibration matrices.
    """
    # 1. PRE-CENTER THE DATA
    # This guarantees the origin is inside the point cloud, preventing 
    # negative-definite matrices and imaginary numbers.
    center = np.mean(mag_data, axis=0)
    centered_data = mag_data - center

    x = centered_data[:, 0]
    y = centered_data[:, 1]
    z = centered_data[:, 2]

    # 2. FIT ELLIPSOID TO CENTERED DATA
    # Design matrix
    D = np.array([x*x, y*y, z*z, 2*x*y, 2*x*z, 2*y*z, 2*x, 2*y, 2*z]).T
    O = np.ones(len(x))

    # Solve least squares problem: D * v = O
    v, _, _, _ = np.linalg.lstsq(D, O, rcond=None)

    # Form the algebraic form matrix A
    A = np.array([[v[0], v[3], v[4]],
                  [v[3], v[1], v[5]],
                  [v[4], v[5], v[2]]])

    # Calculate offset of the centered data
    bias_centered = np.linalg.solve(-A, v[6:9])

    # 3. CALCULATE FINAL MATRICES
    # Total hard iron bias is the initial rough center + the fine-tuned fit offset
    hard_iron_bias = center + bias_centered

    # Calculate the scaling factor to normalize the sphere
    # Equation: (x - bias)^T * A * (x - bias) = 1 + bias^T * A * bias
    R_squared = 1.0 + np.dot(bias_centered.T, np.dot(A, bias_centered))

    # Scale the matrix so the calibrated output vectors have a normalized magnitude of 1.0
    A_normalized = A / abs(R_squared)

    # Calculate soft iron transformation matrix
    soft_iron_matrix = np.real(scipy.linalg.sqrtm(A_normalized))

    return hard_iron_bias, soft_iron_matrix

def apply_calibration(mag_data, hard_iron_bias, soft_iron_matrix):
    """
    Applies calibration to N x 3 data.
    """
    return (mag_data - hard_iron_bias).dot(soft_iron_matrix)