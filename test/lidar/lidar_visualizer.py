import os
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

class LidarVisualizer:
    def __init__(self, output_folder="lidar_images"):
        self.output_folder = output_folder
        os.makedirs(output_folder, exist_ok=True)
        self.fig = plt.figure(figsize=(8, 8))
        self.ax = self.fig.add_subplot(111, projection='polar')
        self.counter = 0

    def plot_frame(self, dist_bins):
        """
        Отрисовать и сохранить кадр.
        """
        self.ax.cla()
        self.ax.set_theta_offset(math.pi / 2)
        self.ax.set_title("Lidar view (EMA stabilized)", fontsize=16)

        # Убираем NaN
        mask = np.isfinite(dist_bins)
        if not np.any(mask):
            return

        angles_rad = np.deg2rad(np.arange(0, 360, 2)[mask])
        dists = dist_bins[mask]

        self.ax.scatter(angles_rad, dists, s=6, c='b', alpha=0.7)
        filename = os.path.join(self.output_folder, f"lidar_{self.counter:04d}.png")
        self.fig.savefig(filename, bbox_inches='tight')
        print(f"[INFO] Saved {filename} ({mask.sum()} pts)")
        self.counter += 1
