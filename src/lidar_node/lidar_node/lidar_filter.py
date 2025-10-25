import numpy as np

def filter_lidar_points(lidar_data, confidence_threshold=50):
    mask = np.array(lidar_data.Confidence_i) >= confidence_threshold

    return {
        "Distance": np.array(lidar_data.Distance_i)[mask],
        "Degree_angle": np.array(lidar_data.Degree_angle)[mask],
        "Angle": np.array(lidar_data.Angle_i)[mask],
        "Confidence": np.array(lidar_data.Confidence_i)[mask],
    }


def binarize_by_degree(degrees, distances, confidences, n_bins=360):
    dist_bins = np.full(n_bins, np.nan)
    conf_bins = np.full(n_bins, np.nan)

    degrees = np.array(degrees)
    distances = np.array(distances)
    confidences = np.array(confidences)

    angle_increment = 360 / n_bins

    for i in range(n_bins):
        mask = (degrees >= i) & (degrees < i + angle_increment)
        if np.any(mask):
            dist_bins[i] = np.nanmedian(distances[mask])
            conf_bins[i] = np.nanmedian(confidences[mask])

    return dist_bins, conf_bins
