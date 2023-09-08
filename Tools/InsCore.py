# -*- coding: utf-8 -*-
# @Time : 2020/9/10 下午9:50
# @Author : rebeater
# @File : ins_core.py
# @Project: LooselyCouple_2020
# @Function: Inertial Navigation System Core Functions


import numpy as np

# import yaml


_deg = np.pi / 180.0
_hour = 3600
_sqrt_hour = 60
_mGal = 1e-5
_ppm = 1e-6


class ImuPara:

    def __init__(self, yml_path):
        import yaml
        with open(yml_path) as f:
            args = yaml.safe_load(f)
        self.gb_std = np.array(args["gb-std"]) * _deg / _hour
        self.ab_std = np.array(args["ab-std"]) * _mGal
        self.gs_std = np.array(args["gs-std"]) * _ppm
        self.as_std = np.array(args["as-std"]) * _ppm
        self.gb_ini = np.array(args["gb-ini"]) * _deg / _hour
        self.ab_ini = np.array(args["ab-ini"]) * _mGal
        self.gs_ini = np.array(args["gs-ini"]) * _ppm
        self.as_ini = np.array(args["as-ini"]) * _ppm
        self.arw = args["arw"] * _deg / _sqrt_hour
        self.vrw = args["vrw"] / _sqrt_hour
        self.at_corr = args["at-corr"] * _hour
        self.gt_corr = args["gt-corr"] * _hour


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


# Quaternion

class Quaternion:
    def __init__(self, p0, pv1, pv2, pv3):
        self.p0 = p0
        self.pv = np.array([pv1, pv2, pv3], dtype=np.double)

    def __str__(self):
        string = "Q: %.9f,[%.9f,%.9f,%.9f]" % (self.p0, self.pv[0], self.pv[1], self.pv[2])
        return string

    def __eq__(self, other):
        return (self.p0 == other.p0) & (self.pv == other.pv)[0] & (self.pv == other.pv)[1] & (self.pv == other.pv)[2]

    def __add__(self, other):
        p0 = self.p0 + other.p0
        pv = self.pv + other.pv
        return Quaternion(p0, pv[0], pv[1], pv[2])

    def __sub__(self, other):
        p0 = self.p0 - other.p0
        pv = self.pv - other.pv
        return Quaternion(p0, pv[0], pv[1], pv[2])

    def __mul__(self, other):
        p0 = self.p0
        p1 = self.pv[0]
        p2 = self.pv[1]
        p3 = self.pv[2]
        q0 = other.p0
        q1 = other.pv[0]
        q2 = other.pv[1]
        q3 = other.pv[2]
        r0 = p0 * q0 - p1 * q1 - p2 * q2 - p3 * q3
        v0 = p0 * q1 + p1 * q0 + p2 * q3 - p3 * q2
        v1 = p0 * q2 + p2 * q0 + p3 * q1 - p1 * q3
        v2 = p0 * q3 + p3 * q0 + p1 * q2 - p2 * q1
        return Quaternion(r0, v0, v1, v2)

    def __abs__(self):
        return np.sqrt(self.p0 * self.p0 + np.matmul(self.pv, self.pv))

    # 除以整数
    def __truediv__(self, other):
        return Quaternion(self.p0 / other, self.pv[0] / other, self.pv[1] / other, self.pv[2] / other)

    # 归一化
    def normalize(self):
        return self / abs(self)

    def conj(self):
        return Quaternion(self.p0, -self.pv[0], -self.pv[1], -self.pv[2])


def gyro_to_rv(delta_theta_k, delta_theta_k1):
    """
    角速度增量转换为旋转矢量
    :reference:Shin, E.-H. (2005).  E2.60
    :param delta_theta_k:
    :param delta_theta_k1:
    :return:
    """
    return delta_theta_k + np.cross(delta_theta_k1, delta_theta_k) / 12.0


def rv_to_quaternion(rv):
    """
    旋转向量转换为四元数
    :reference:Shin, E.-H. (2005).  E2.5
    :return:
    """
    mag2 = rv[0] ** 2 + rv[1] ** 2 + rv[2] ** 2
    if mag2 < np.pi ** 2:
        mag2 = 0.25 * mag2
        cos = 1.0 - mag2 / 2.0 * (1.0 - mag2 / 12.0 * (1 - mag2 / 30.0))
        sin = 1.0 - mag2 / 6.0 * (1.0 - mag2 / 20.0 * (1 - mag2 / 42.0))
        q = Quaternion(cos, 0.5 * sin * rv[0], 0.5 * sin * rv[1], 0.5 * sin * rv[2])
    else:
        mag = np.sqrt(mag2)
        s_mag = np.sin(mag / 2)
        q = Quaternion(
            np.cos(mag / 2),
            rv[0] * s_mag / mag,
            rv[1] * s_mag / mag,
            rv[2] * s_mag / mag,
        )
        if q.p0 < 0:
            q = Quaternion(
                -np.cos(mag / 2),
                - rv[0] * s_mag / mag,
                - rv[1] * s_mag / mag,
                - rv[2] * s_mag / mag,
            )

    return q


def rv_to_dcm(rv):
    norm = np.sqrt(rv[0] * rv[0] + rv[1] * rv[1] + rv[2] * rv[2])
    eye3 = np.eye(3, dtype=np.double)
    skew = skew_sym(rv)
    return eye3 + np.sin(norm) / norm * skew + (1 - np.cos(norm)) / (norm * norm) * np.matmul(skew, skew)


def quaternion_to_dcm(q: Quaternion):
    """
    DCM in terms of Quaternion
    :reference:Shin, E.-H. (2005).  E2.9
    :param q:
    :return:
    """
    q1 = q.p0
    q2 = q.pv[0]
    q3 = q.pv[1]
    q4 = q.pv[2]
    dcm = np.zeros([3, 3], dtype=np.double)
    dcm[0, 0] = q1 * q1 + q2 * q2 - q3 * q3 - q4 * q4
    dcm[0, 1] = 2 * (q2 * q3 - q1 * q4)
    dcm[0, 2] = 2 * (q2 * q4 + q1 * q3)
    dcm[1, 0] = 2 * (q2 * q3 + q1 * q4)
    dcm[1, 1] = q1 * q1 - q2 * q2 + q3 * q3 - q4 * q4
    dcm[1, 2] = 2 * (q3 * q4 - q1 * q2)
    dcm[2, 0] = 2 * (q2 * q4 - q1 * q3)
    dcm[2, 1] = 2 * (q3 * q4 + q1 * q2)
    dcm[2, 2] = q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4
    return dcm


def dcm_to_euler(dcm):
    """
    Euler in terms of DCM
    :reference:Shin, E.-H. (2005).  E2.16
    :param dcm:
    :return:
    """
    theta = np.zeros(3, dtype=np.double)
    pitch = np.arctan(-dcm[2, 0] / np.sqrt(dcm[2, 1] * dcm[2, 1] + dcm[2, 2] * dcm[2, 2]))
    if dcm[2, 0] <= -0.999:
        roll = np.NaN
        heading = np.arctan2(dcm[1, 2] - dcm[0, 1], dcm[0, 2] + dcm[1, 1])
    elif dcm[2, 0] >= 0.999:
        roll = np.NaN
        heading = np.arctan2(dcm[1, 2] + dcm[0, 1], dcm[0, 2] + dcm[1, 1]) + np.pi
    else:
        roll = np.arctan2(dcm[2, 1], dcm[2, 2])
        heading = np.arctan2(dcm[1, 0], dcm[0, 0])
    theta[1] = pitch
    theta[0] = roll
    theta[2] = heading
    return theta


# def dcm_to_quaternion(dcm):
#     """
#     quaternion in terms of dcm
#     :param dcm:
#     :return:
#     """
#     pass


def euler_to_quaternion(euler):
    roll, pitch, heading = euler[0], euler[1], euler[2]
    q0 = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(heading / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(
        heading / 2)
    q1 = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(heading / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(
        heading / 2)
    q2 = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(heading / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(
        heading / 2)
    q3 = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(heading / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(
        heading / 2)
    return Quaternion(q0, q1, q2, q3)


def euler_to_dcm(roll, pitch, heading):
    """
    DCM in terms of euler
    :reference:Shin, E.-H. (2005).  E2.15
    :param roll:
    :param pitch:
    :param heading:
    :return:
    """
    phi, theta, psi = roll, pitch, heading
    c = np.zeros([3, 3], dtype=np.double)
    c[0, 0] = np.cos(theta) * np.cos(psi)
    c[0, 1] = -np.cos(phi) * np.sin(psi) + np.sin(phi) * np.sin(theta) * np.cos(psi)
    c[0, 2] = np.sin(phi) * np.sin(psi) + np.cos(phi) * np.sin(theta) * np.cos(psi)
    c[1, 0] = np.cos(theta) * np.sin(psi)
    c[1, 1] = np.cos(phi) * np.cos(psi) + np.sin(phi) * np.sin(theta) * np.sin(psi)
    c[1, 2] = -np.sin(phi) * np.cos(psi) + np.cos(phi) * np.sin(theta) * np.sin(psi)
    c[2, 0] = - np.sin(theta)
    c[2, 1] = np.sin(phi) * np.cos(theta)
    c[2, 2] = np.cos(phi) * np.cos(theta)
    return c


# cross-product(skew-symmetric) equation 2.11
def skew_sym(phi):
    m = np.zeros([3, 3], dtype=np.double)
    m[0, 1] = -phi[2]
    m[0, 2] = phi[1]
    m[1, 0] = phi[2]
    m[1, 2] = -phi[0]
    m[2, 0] = -phi[1]
    m[2, 1] = phi[0]
    return m


def lla_to_cne(lat, lon):
    dcm = np.zeros([3, 3], dtype=np.double)
    dcm[0, 0] = -np.sin(lat) * np.cos(lon)
    dcm[0, 1] = -np.sin(lon)
    dcm[0, 2] = -np.cos(lat) * np.cos(lon)
    dcm[1, 0] = -np.sin(lat) * np.sin(lon)
    dcm[1, 1] = np.cos(lon)
    dcm[1, 2] = -np.cos(lat) * np.sin(lon)
    dcm[2, 0] = np.cos(lat)
    dcm[2, 1] = 0
    dcm[2, 2] = -np.sin(lat)
    return dcm


def lla_to_qne(latitude, longitude):
    phi = -0.25 * np.pi - 0.5 * latitude
    lamda = 0.5 * longitude
    q0 = np.cos(phi) * np.cos(lamda)
    q1 = -np.sin(phi) * np.sin(lamda)
    q2 = np.sin(phi) * np.cos(lamda)
    q3 = np.cos(phi) * np.sin(lamda)
    return Quaternion(q0, q1, q2, q3)


def qne_to_lla(q: Quaternion):
    """
    Quaternion Q_ne -> lat,lon
    :param q: Q_ne
    :return:
    """
    a = np.arctan(q.pv[1] / q.p0)
    b = np.arctan2(q.pv[2], q.p0)
    latitude = (-0.5 * np.pi - 2 * a)
    longitude = 2 * b
    return latitude, longitude


class NavData:
    def __init__(self, nav):
        self.week = nav[0]
        self.time_s = nav[1]
        self.latitude = nav[2] * np.pi / 180.0
        self.longitude = nav[3] * np.pi / 180.0
        self.height = nav[4]
        self.roll = nav[8] * np.pi / 180.0
        self.pitch = nav[9] * np.pi / 180.0
        self.heading = nav[10] * np.pi / 180.0
        self.v_f_k_b = np.zeros(3, dtype=np.double)
        self.dvn = np.zeros(3, dtype=np.double)
        self.Qbn = euler_to_quaternion(np.deg2rad(nav[8:11]))
        self.Cbn = euler_to_dcm(self.roll, self.pitch, self.heading)
        self.Qne = lla_to_qne(self.latitude, self.longitude)
        self.Vn = np.array([
            nav[5], nav[6], nav[7]
        ], dtype=np.double)
        self.atti = np.array([
            self.roll, self.pitch, self.heading
        ], dtype=np.double)
        self.gyro_bias = np.zeros(3, dtype=np.double)
        self.acce_bias = np.zeros(3, dtype=np.double)
        self.gyro_scale = np.zeros(3, dtype=np.double)
        self.acce_scale = np.zeros(3, dtype=np.double)

    def __str__(self):
        return "%d,%.4f,%.8f,%.8f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%0.4f" % (
            self.week, self.time_s,
            self.latitude * 180.0 / np.pi, self.longitude * 180.0 / np.pi, self.height,
            self.Vn[0], self.Vn[1], self.Vn[2],
            self.roll * 180.0 / np.pi, self.pitch * 180.0 / np.pi, self.heading * 180.0 / np.pi)

    def to_array(self):
        return np.array([
            self.time_s, self.latitude, self.longitude, self.height,
            self.Vn[0], self.Vn[1], self.Vn[2],
            self.roll, self.pitch, self.heading,
        ], dtype=np.double)


def compensate_imu(imu_data, nav: NavData, delta_t):
    scale = np.eye(3) - np.diag(nav.gyro_scale)
    d_theta = scale @ (imu_data[1:4] - nav.gyro_bias * delta_t)  # 陀螺仪输出
    scale = np.eye(3) - np.diag(nav.acce_scale)
    dv_fb = scale @ (imu_data[4:7] - nav.acce_bias * delta_t)
    return np.array([
        imu_data[0], d_theta[0], d_theta[1], d_theta[2], dv_fb[0], dv_fb[1], dv_fb[2]
    ], np.double)


def progress(percent, width=100):
    """
    进度打印
    :param percent:进度
    :param width:
    :return: None
    """
    if percent >= 100:
        percent = 100
    show_str = ('[%%-%ds]' % width) % (int(width * percent / 100) * ">")  # 字符串拼接的嵌套使用 219
    print('\r%s %.1f%%' % (show_str, percent), end='')


def write2txt(f, nav: NavData):
    f.write("%5d" % nav.week)
    f.write("%13.5f " % nav.time_s)
    f.write("%20.15f " % (nav.latitude * 180.0 / np.pi))
    f.write("%20.15f " % (nav.longitude * 180.0 / np.pi))
    f.write("%16.10f " % nav.height)
    f.write("%20.10f " % (nav.Vn[0]))
    f.write("%20.10f " % (nav.Vn[1]))
    f.write("%20.10f " % (nav.Vn[2]))
    f.write("%12.6f " % (nav.roll * 180.0 / np.pi))
    f.write("%12.6f " % (nav.pitch * 180.0 / np.pi))
    f.write("%12.6f " % (nav.heading * 180.0 / np.pi))
    f.write(" %10.0f " % (nav.gyro_bias[0] / _deg * _hour))
    f.write(" %10.0f " % (nav.gyro_bias[1] / _deg * _hour))
    f.write(" %10.0f " % (nav.gyro_bias[2] / _deg * _hour))
    f.write(" %10.0f " % (nav.acce_bias[0] / _mGal))
    f.write(" %10.0f " % (nav.acce_bias[1] / _mGal))
    f.write(" %10.0f " % (nav.acce_bias[2] / _mGal))
    f.write("\n")


wgs84 = WGS84()


def get_distance(nav1: NavData, nav2: NavData):
    dn = wgs84.dN(nav1.latitude, nav2.latitude)
    de = wgs84.dE(nav1.latitude, nav1.longitude, nav2.longitude)
    dd = nav1.height - nav2.height
    d = np.sqrt(dn ** 2 + de ** 2 + dd ** 2)  #
    return dn, de, dd, d


def get_distance_d(lat, lon, lat2, lon2):
    dn = wgs84.dN(lat, lat2)
    de = wgs84.dE(lat, lon, lon2)
    d = np.sqrt(dn ** 2 + de ** 2)
    return dn, de, d


def mechanization_n(cur_imu, last_imu, last_epoch: NavData):
    """
     INS mechanization in terms of N-frame
    :reference:Shin, E.-H. (2005).  C2.3
    :param cur_imu: [0]:GPS-t  [1]:gx [2]:gy, [3]:gz [4]:ax [5]:ay [6]:az
    :param last_imu: [0]:GPS-t  [1]:gx [2]:gy, [3]:gz [4]:ax [5]:ay [6]:az
    :param last_epoch: last Navigation Data
    :return:
    """
    last_q_b_n = last_epoch.Qbn  #
    last_dcm_b_n = last_epoch.Cbn  #
    last_q_n_e = last_epoch.Qne  #

    # if cur_imu[0] < last_epoch.time_s:
    #     raise ValueError("current imu time(%f) is NOT after last epoch(%f)" % (cur_imu[0], last_epoch.time_s))
    delta_t = cur_imu[0] - last_epoch.time_s
    if np.fabs(delta_t) < 1e-4:
        print("imu 时间戳重复出现(%f)" % last_epoch.time_s)
        return last_epoch
    # if delta_t > 1.0 / 100:
    #     raise Warning("current imu time(%f) is pretty FAR from last epoch(%f)" % (cur_imu[0], last_epoch.time_s))
    # assert -1000 < last_epoch.height < 10000  # 单纯通知你程序跑飞啦!

    current_epoch = last_epoch  # 复制上时刻信息，其实主要是零偏，单独复制零偏也可
    current_epoch.time_s = cur_imu[0]

    '''补偿'''
    cur_imudata = compensate_imu(cur_imu, last_epoch, delta_t)
    last_imu = compensate_imu(last_imu, last_epoch, delta_t)
    theta_delta = cur_imudata[1:4]  # 陀螺仪输出
    theta_delta_k1 = last_imu[1:4]
    last_v_n = last_epoch.Vn
    dv_fb = cur_imudata[4:7]  # 当前加表输出
    dv_fb_last = last_imu[4:7]  # 上时刻加表输出

    omega_en_n = wgs84.omega_en_n(last_v_n[0], last_v_n[1], last_epoch.height, last_epoch.latitude)
    omega_ie_n = wgs84.omega_ie_n(last_epoch.latitude)
    '''速度更新'''
    # 外插一个历元 E2.50-2.51,计算中间时刻的位置速度
    h_mid = last_epoch.height - last_epoch.Vn[2] * delta_t / 2  # E2.50
    zeta_mid = (omega_en_n + omega_ie_n) * delta_t / 2
    epsilon_mid = wgs84.omega_ie_e * delta_t / 2  #
    q_nn_mid = rv_to_quaternion(zeta_mid)  # E2.51C
    q_ee_mid = rv_to_quaternion(epsilon_mid).conj()  # E2.51d
    q_ne_mid = q_ee_mid * last_q_n_e * q_nn_mid  # E2.51b中间时刻的位置
    lat_mid, lon_mid = qne_to_lla(q_ne_mid)  #
    vn_mid = last_v_n + 0.5 * last_epoch.dvn
    # 重新计算omega_in_n
    omega_ie_n = wgs84.omega_ie_n(lat_mid)
    omega_en_n = wgs84.omega_en_n(vn_mid[0], vn_mid[1], h_mid, lat_mid)
    # 正式开始更新速度
    gamma = np.array([0, 0, wgs84.g(last_epoch.latitude, last_epoch.height)],
                     dtype=np.double)
    v_g_cor = (gamma - np.cross(2 * omega_ie_n + omega_en_n, last_epoch.Vn)) * delta_t
    # 等效旋转矢量 n系
    i3 = np.eye(3, dtype=np.double)
    zeta_k1_k = (omega_en_n + omega_ie_n) * delta_t
    # 比力变化量 b系
    # sculling motion
    last_v_f_k_b = dv_fb + 0.5 * np.cross(theta_delta, dv_fb) + (
            np.cross(theta_delta_k1, dv_fb) + np.cross(dv_fb_last, theta_delta)) / 12.0
    # 计算当前速度
    current_v_f_k_b = (i3 - 0.5 * skew_sym(zeta_k1_k)) @ last_dcm_b_n @ last_v_f_k_b
    current_v_n = last_v_n + current_v_f_k_b + v_g_cor
    current_epoch.dvn = current_v_f_k_b + v_g_cor
    current_epoch.v_f_k_b = current_v_f_k_b / delta_t
    current_epoch.Vn = current_v_n

    ''' 位置更新 更新Qne'''
    epsilon_k = wgs84.omega_ie_e * delta_t
    q_e_e_delta = rv_to_quaternion(epsilon_k).conj()
    zeta_k = (omega_ie_n + omega_en_n) * delta_t
    q_n_n_delta = rv_to_quaternion(zeta_k)
    current_q_n_e = q_e_e_delta * last_q_n_e * q_n_n_delta
    current_epoch.Qne = current_q_n_e.normalize()
    # 位置更新完毕，重新计算omega_en_n,omega_ie_e
    lat, lon = qne_to_lla(current_epoch.Qne)
    vn_mid = (last_v_n + current_v_n) / 2
    omega_ie_n = wgs84.omega_ie_n(lat)
    omega_en_n = wgs84.omega_en_n(vn_mid[0], vn_mid[1], h_mid, lat)
    # zeta_k = (omega_en_n + omega_ie_n) / 2

    ''' 姿态更新更新Qbn'''
    # rotation vector
    rv_b_b_delta = gyro_to_rv(theta_delta, theta_delta_k1)
    #  b-frame quaternion
    q_b_b_delta = rv_to_quaternion(rv_b_b_delta)
    # n-frame quaternion
    zeta_k = (omega_ie_n + omega_en_n) * delta_t
    q_n_n_delta_skew = rv_to_quaternion(zeta_k).conj()
    # update quaternion
    q_b_n_current = q_n_n_delta_skew * last_q_b_n * q_b_b_delta
    current_epoch.Qbn = q_b_n_current.normalize()  # 归一化

    # output
    current_epoch.latitude, current_epoch.longitude = lat, lon
    current_epoch.height = last_epoch.height - vn_mid[2] * delta_t
    current_epoch.Cbn = quaternion_to_dcm(q_b_n_current)  # 先变成dcm再变成姿态角
    euler_angle = dcm_to_euler(current_epoch.Cbn)
    current_epoch.roll = euler_angle[0]  #
    current_epoch.pitch = euler_angle[1]
    current_epoch.heading = euler_angle[2]
    current_epoch.atti = euler_angle
    current_epoch.Qbn = q_b_n_current

    return current_epoch

if __name__ == '__main__':
    w = WGS84()
    print( w.g(lat=39.980544920000 *_deg,h=20))