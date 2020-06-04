# -*- coding: utf-8 -*-
import numpy as np

class SphericalTrigonometry:
    # 赤道半径
    equatorial_radius = 6377397.155
    # 第一離心率^2
    squared_first_eccentricity = 0.00667436061028297

    def calculate(self, lat1, lon1, lat2, lon2):
        # 緯度経度をラジアンに変換

        radLat1 = np.radians(lat1) # 緯度１
        radLon1 = np.radians(lon1) # 経度１
        radLat2 = np.radians(lat2) # 緯度２
        radLon2 = np.radians(lon2) # 経度２

        # 緯度差
        radLatDiff = radLat1 - radLat2

        # 経度差算
        radLonDiff = radLon1 - radLon2

        # 平均緯度
        radLatAve = (radLat1 + radLat2) / 2.0

        # 赤道上の子午線曲率半径
        a1e2 = self.equatorial_radius * (1 - self.squared_first_eccentricity)
        # a1e2 = 6334832.10663254 

        sinLat = np.sin(radLatAve)
        W2 = 1.0 - self.squared_first_eccentricity * sinLat ** 2
        M = a1e2 / (np.sqrt(W2)*W2) # 子午線曲率半径M
        N = self.equatorial_radius / np.sqrt(W2) # 卯酉線曲率半径

        t1 = M * radLatDiff
        t2 = N * np.cos(radLatAve) * radLonDiff
        dist = np.sqrt(t1 ** 2 + t2 ** 2)

        return dist