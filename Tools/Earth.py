# -*- coding:utf-8 -*-
# @Time : 2021/2/25 上午11:17
# @Author :  rebeater
# @File : WGS84.py
# @Project:  Mt
# @Function: TODO

import numpy as np

class WGS84:
    """
    和WGS84坐标系相关的量
    """

    def __init__(self):
        # 长轴
        self.r = 6378137.0
        # 地球自转速度
        self.omega_e = 7.2921151467e-5
        # WGS84椭球坐标系 第一偏心率
        self.e2 = 0.00669437999013
        # 赤道重力
        self.g0 = 9.7803267715

        self.omega_ie_e = np.array([
            0, 0, self.omega_e
        ], dtype=np.double)

    def RM(self, phi):
        return self.r * (1 - self.e2) / np.power(1 - self.e2 * np.sin(phi) * np.sin(phi), 1.5)

    def RN(self, phi):
        return self.r / np.sqrt(1 - self.e2 * np.sin(phi) * np.sin(phi))

    def dN(self, lat1, lat2):
        dn = (lat1 - lat2) * self.RM(lat1)
        return dn

    def dE(self, lat1, lon1, lon2):
        de = (lon1 - lon2) * self.RN(lat1) * np.cos(lat1)
        return de

    def ecef_to_lla(self, xyz):
        x, y, z = xyz
        p = np.sqrt(x ** 2 + y ** 2)
        lon = 2.0 * np.arctan2(y, x + p)
        lat = np.arctan(z / (p * (1 - self.e2)))
        h = 0
        circle = 0
        while True:
            RN = self.RN(lat)
            h_pre = h
            h = p / np.cos(lat) - RN
            lat = np.arctan2(z, p * (1 - self.e2 * RN / (RN + h)))
            if np.fabs(h - h_pre) < 1e-8:
                break
            circle += 1
            if circle > 10:
                print("ecef-frame can't be transmit to n-frame")
                break
        return np.array([lat, lon, h], dtype=np.double)

    def lla_to_ecef(self, lla):
        """
        纬度经度高度转换为e系下xyz
        :reference::reference:Shin, E.-H. (2005).  E2.20
        :param lla: deg
        :return:
        """
        lat, lon, h = lla[0], lla[1], lla[2]
        re = np.zeros(3, dtype=np.double)
        rn = self.RN(lat)
        re[0] = (rn + h) * np.cos(lat) * np.cos(lon)
        re[1] = (rn + h) * np.cos(lat) * np.sin(lon)
        re[2] = (rn * (1 - self.e2) + h) * np.sin(lat)
        return re

    # @staticmethod
    def lla_to_cne(self, lat, lon):
        """
        Cne: DCM N-frame to E-frame
        :reference::reference:Shin, E.-H. (2005).  E2.26
        :param lat:
        :param lon:
        :return:
        """
        sin_lat = np.sin(lat)
        cos_lat = np.cos(lat)
        sin_lon = np.sin(lon)
        cos_lon = np.cos(lon)
        cne = np.array([
            [-sin_lat * cos_lon, -sin_lon, -cos_lat * cos_lon],
            [-sin_lat * sin_lon, cos_lon, -cos_lat * sin_lon],
            [cos_lat, 0, -sin_lat]
        ], np.double)
        return cne

    def omega_en_n(self, vn, ve, h, lat):
        rm = self.RM(lat)
        rn = self.RN(lat)
        return np.array([
            ve / (rn + h), -vn / (rm + h), -ve * np.tan(lat) / (rn + h)
        ], dtype=np.double)

    def omega_ie_n(self, phi):
        return np.array([self.omega_e * np.cos(phi), 0.0, -self.omega_e * np.sin(phi)], dtype=np.double)

    def g(self, lat, h):
        sin2 = np.sin(lat) ** 2
        g = self.g0 * (
                1 + 0.0052790414 * sin2 + 0.0000232718 * sin2 ** 2) + h * (
                    0.0000000043977311 * sin2 - 0.0000030876910891
            ) + 0.0000000000007211 * h ** 2
        return g
