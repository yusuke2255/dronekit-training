# -*- coding: utf-8 -*-
from math import *

# 楕円体
ELLIPSOID_GRS80 = 1 # GRS80
ELLIPSOID_WGS84 = 2 # WGS84

# 楕円体別の長軸半径と扁平率
GEODETIC_DATUM = {
    ELLIPSOID_GRS80: [
        6378137.0,         # [GRS80]長軸半径
        1 / 298.257222101, # [GRS80]扁平率
    ],
    ELLIPSOID_WGS84: [
        6378137.0,         # [WGS84]長軸半径
        1 / 298.257223563, # [WGS84]扁平率
    ],
}

# 反復計算の上限回数
ITERATION_LIMIT = 1000

'''
Vincenty法(順解法)
始点の座標(緯度経度)と方位角と距離から、終点の座標と方位角を求める
:param lat: 緯度
:param lon: 経度
:param azimuth: 方位角
:param distance: 距離
:param ellipsoid: 楕円体
:return: 終点の座標、方位角
'''
def direct(lat, lon, azimuth, distance, ellipsoid=None):

    # 計算時に必要な長軸半径(a)と扁平率(f)を定数から取得し、短軸半径(b)を算出する
    # 楕円体が未指定の場合はGRS80の値を用いる
    a, f = GEODETIC_DATUM.get(ellipsoid, GEODETIC_DATUM.get(ELLIPSOID_GRS80))
    b = (1 - f) * a

    # ラジアンに変換する(距離以外)
    phi1 = radians(lat)
    lam1 = radians(lon)
    alpha1 = radians(azimuth)
    s = distance

    sinalpha1 = sin(alpha1)
    cosalpha1 = cos(alpha1)

    # 更成緯度(補助球上の緯度)
    U1 = atan((1 - f) * tan(phi1))

    sinU1 = sin(U1)
    cosU1 = cos(U1)
    tanU1 = tan(U1)

    sigma1 = atan2(tanU1, cosalpha1)
    sinalpha = cosU1 * sinalpha1
    cos2alpha = 1 - sinalpha ** 2
    u2 = cos2alpha * (a ** 2 - b ** 2) / (b ** 2)
    A = 1 + u2 / 16384 * (4096 + u2 * (-768 + u2 * (320 - 175 * u2)))
    B = u2 / 1024 * (256 + u2 * (-128 + u2 * (74 - 47 * u2)))

    # sigmaをs/(b*A)で初期化
    sigma = s / (b * A)

    # 以下の計算をsigmaが収束するまで反復する
    # 地点によっては収束しないことがあり得るため、反復回数に上限を設ける
    for i in range(ITERATION_LIMIT):
        cos2sigmam = cos(2 * sigma1 + sigma)
        sinsigma = sin(sigma)
        cossigma = cos(sigma)
        deltasigma = B * sinsigma * (cos2sigmam + B / 4 * (cossigma * (-1 + 2 * cos2sigmam ** 2) - B / 6 * cos2sigmam * (-3 + 4 * sinsigma ** 2) * (-3 + 4 * cos2sigmam ** 2)))
        sigmadash = sigma
        sigma = s / (b * A) + deltasigma

        # 偏差が.000000000001以下ならbreak
        if abs(sigma - sigmadash) <= 1e-12:
            break
    else:
        # 計算が収束しなかった場合はNoneを返す
        return None

    # sigmaが所望の精度まで収束したら以下の計算を行う
    x = sinU1 * sinsigma - cosU1 * cossigma * cosalpha1
    phi2 = atan2(sinU1 * cossigma + cosU1 * sinsigma * cosalpha1, (1 - f) * sqrt(sinalpha ** 2 + x ** 2))
    lam = atan2(sinsigma * sinalpha1, cosU1 * cossigma - sinU1 * sinsigma * cosalpha1)
    C = f / 16 * cos2alpha * (4 + f * (4 - 3 * cos2alpha))
    L = lam - (1 - C) * f * sinalpha * (sigma + C * sinsigma * (cos2sigmam + C * cossigma * (-1 + 2 * cos2sigmam ** 2)))
    lam2 = L + lam1

    alpha2 = atan2(sinalpha, -x) + pi

    return {
        'lat': degrees(phi2),     # 緯度
        'lon': degrees(lam2),     # 経度
        'azimuth': degrees(alpha2), # 方位角
    }