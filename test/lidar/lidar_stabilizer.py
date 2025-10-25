import numpy as np
from collections import deque

class MedianStabilizer:
    """
    Медианный стабилизатор для угловых бинов лидара (1°).
    Хранит фиксированное число последних измерений и выдает медиану по каждому бину.
    """
    def __init__(self, n_bins=180, window_size=5, spatial_kernel=3):
        self.n_bins = n_bins
        self.window_size = window_size
        self.spatial_kernel = spatial_kernel if spatial_kernel % 2 == 1 else spatial_kernel + 1
        # Очереди последних значений для каждого бина
        self.history = [deque(maxlen=window_size) for _ in range(n_bins)]

    def _spatial_smooth(self, arr):
        """Пространственное медианное сглаживание по соседним углам."""
        k = self.spatial_kernel
        pad = k // 2
        padded = np.pad(arr, pad, mode='wrap')
        result = np.empty_like(arr)
        for i in range(len(arr)):
            window = padded[i:i + k]
            if np.all(np.isnan(window)):
                result[i] = np.nan
            else:
                result[i] = np.nanmedian(window)
        return result

    def update(self, distances):
        """
        Обновляет буфер измерений и возвращает стабилизированные данные (медиану по времени и пространству).
        """
        arr = np.asarray(distances, dtype=float)
        arr = self._spatial_smooth(arr)

        # Добавляем новое измерение в историю
        for i in range(self.n_bins):
            if np.isfinite(arr[i]):
                self.history[i].append(arr[i])

        # Вычисляем медиану по временной оси
        stabilized = np.array([
            np.nanmedian(self.history[i]) if len(self.history[i]) > 0 else np.nan
            for i in range(self.n_bins)
        ])

        # Дополнительное пространственное сглаживание медианных значений
        stabilized = self._spatial_smooth(stabilized)

        return stabilized.copy()

