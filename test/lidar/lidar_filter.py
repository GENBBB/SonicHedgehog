import numpy as np

def filter_lidar_points(lidar_data, confidence_threshold=50):
    """
    Отфильтровать точки по уровню Confidence.
    """
    mask = np.array(lidar_data.Confidence_i) >= confidence_threshold

    return {
        "Distance": np.array(lidar_data.Distance_i)[mask],
        "Degree_angle": np.array(lidar_data.Degree_angle)[mask],
        "Angle": np.array(lidar_data.Angle_i)[mask],
        "Confidence": np.array(lidar_data.Confidence_i)[mask],
    }


def binarize_by_degree(degrees, distances, confidences, n_bins=180):
    """
    Бинаризация данных по углу (1 градус = 1 бин).
    Усредняет расстояние и confidence в каждом угловом бине.
    """
    dist_bins = np.full(n_bins, np.nan)
    conf_bins = np.full(n_bins, np.nan)

    degrees = np.array(degrees)
    distances = np.array(distances)
    confidences = np.array(confidences)

    for i in range(n_bins):
        mask = (degrees >= i) & (degrees < i + 2)
        if np.any(mask):
            dist_bins[i] = np.nanmedian(distances[mask])
            conf_bins[i] = np.nanmedian(confidences[mask])

    return dist_bins, conf_bins
