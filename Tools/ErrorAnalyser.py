# -*- coding:utf-8 -*-
# @Time : 2021/2/25 下午5:17
# @Author :  rebeater
# @File : ErrorAnalyser.py
# @Project:  NavigationTool_qt
# @Function: 误差分析

import numpy as np
import pandas as pd
import scipy.interpolate as interpolate
import InsCore as ins
from Earth import WGS84

wgs84 = WGS84()


def _rms(error):
    return np.sqrt(np.sum(error ** 2) / error.shape[0])


## 杆臂补偿
def compensate(nav, atti_deg, offset):
    atti = np.deg2rad(atti_deg)
    nav[0] *= np.pi/180
    nav[1] *= np.pi/180
    cbn = ins.euler_to_dcm(atti[0], atti[1], atti[2])
    cne = ins.lla_to_cne(nav[0], nav[1])
    re = wgs84.lla_to_ecef(nav) + cne @ cbn @ offset
    nav = wgs84.ecef_to_lla(re)
    nav[0] /= np.pi/180
    nav[1] /= np.pi/180
    return nav


class Error:
    def __init__(self, times=None, error=None, bins=None):
        self.times = times
        self.error = error
        if error is None:
            self.pdf, self.cdf = None, None
            self.cdf = None
            self.first_sigma, self.second_sigma = None, None
            self.rms = None
            self.exception = None
        else:
            self.pdf, self.edges = np.histogram(np.abs(self.error), bins=bins)
            self.cdf = np.cumsum(self.pdf)
            self.first_sigma, self.second_sigma = self.__set_sigma(self.edges, self.cdf)
            self.rms = _rms(self.error)
            self.exception = None

    def __set_sigma(self, edges, cdf):
        first_sigma = 0.6826
        second_sigma = 0.9544
        try:
            cdf_inter = interpolate.interp1d(cdf, edges[1:])
            first_sigma_err = cdf_inter(first_sigma * cdf[-1])
            second_sigma_err = cdf_inter(second_sigma * cdf[-1])
            return first_sigma_err, second_sigma_err
        except Exception as e:
            self.exception = e
            return 0, 0


class ErrorAnalyser:
    def __init__(self, target_file, ref_file, column_index,
                 delimiter="\s+",
                 skip_rows=0,
                 error_hz=5,  ## 误差频率
                 bins=None,
                 write2file=True,
                 offset=None
                 ):
        self.exception = None
        self.target_file = target_file
        self.delimiter = delimiter
        self.skip_rows = skip_rows
        self.error_hz = error_hz
        self.bins = bins
        self.write2file = write2file
        self.ref_file = ref_file
        self.column_index = column_index
        self.wide = 0
        self.start = self.end = 0
        self.times = None
        self.sensors_by_time = None
        self.error_3d = Error()
        self.error_2d = Error()
        self.errs = [Error() for i in column_index]
        print("[Error Analyser.py offset= %s]"%(str(offset)))
        if offset is None:
            self.offset = None
        else:
            self.offset = np.array(offset)

    def analyse(self, _3d=False, _2d=False):
        """
        :param _3d: 统计3D平面误差
        :param _2d: 统计2D平面误差
        :return: None
        """
        try:
            target_data_frame = pd.read_csv(self.target_file,
                                            delimiter=self.delimiter,
                                            engine='python',
                                            header=None,
                                            dtype="double",
                                            skiprows=self.skip_rows,
                                            )
            target_data = np.array(target_data_frame)
            ref_data_frame = pd.read_csv(self.ref_file,
                                         delimiter=self.delimiter,
                                         engine='python',
                                         header=None,
                                         dtype="double",
                                         skiprows=self.skip_rows
                                         )
            ref_data = np.array(ref_data_frame)
        except Exception as e:
            self.exception = e
            return
        if self.bins is None:
            self.bins = max(int(target_data.shape[0] / 500), 100)
        if not np.all(np.isfinite(target_data)):
            exception = Exception("target data is not finite")
            self.exception = exception
            return
        self.wide = target_data.shape[1]
        column_index = self.column_index
        # self.start = np.ceil(t_start)  # 向上取整数
        # self.end = np.floor(t_end)  # 向下取整数
        # self.length = int((np.floor(t_end) - np.ceil(t_start)) * error_hz) + 1  # 每秒error_hz个数据
        # assert ref_data[-1, column_index[1]] > target_data[:, column_index[1]]
        # assert ref_data[0, column_index[1]] < target_data[:, column_index[1]]
        index = np.bitwise_and(ref_data[-1, column_index[1]] > target_data[:, column_index[1]],
                               ref_data[0, column_index[1]] < target_data[:, column_index[1]])
        times = target_data[index, column_index[1]]
        if len(times) < 1:
            self.exception = Exception("no common time")
            return
        self.start = times[0]
        self.end = times[-1]
        self.length = times.shape[0]
        self.times = times
        if self.length == 0:
            exception = Exception("no common time")
            self.exception = exception
            return
        target_new = np.zeros([self.length, self.wide], dtype=np.double)
        ref_new = np.zeros([self.length, self.wide], dtype=np.double)
        # 航向消除循环特性
        if 0 <= column_index[10] < self.wide:
            for t in range(1, ref_data.shape[0]):
                # TODO 这是一种效率很低的写法
                while ref_data[t, column_index[10]] - ref_data[t - 1, column_index[10]] > 180:
                    ref_data[t, column_index[10]] -= 360
                while ref_data[t, column_index[10]] - ref_data[t - 1, column_index[10]] < -180:
                    ref_data[t, column_index[10]] += 360
        # 线性插值拟合
        for id in column_index[2:11]:
            if 0 <= id < self.wide:
                linear = interpolate.interp1d(target_data[:, column_index[1]], target_data[:, id], kind='linear')
                target_new[:, id] = linear(times)
                ref_linear = interpolate.interp1d(ref_data[:, column_index[1]], ref_data[:, id], kind='linear')
                ref_new[:, id] = ref_linear(times)
        target_new = target_data[index]
        target_new[target_new[:, column_index[10]] < 0, column_index[10]] += 360
        target_new[target_new[:, column_index[10]] > 360, column_index[10]] -= 360

        ref_new[ref_new[:, column_index[10]] < 0, column_index[10]] += 360
        ref_new[ref_new[:, column_index[10]] > 360, column_index[10]] -= 360

        if column_index[11] > 0:
            self.sensors_by_time = target_new[:,column_index[11]]
        if self.offset is not None:
            for i, nav in enumerate(ref_new):
                ref_new[i, 2:5] = compensate(nav[2:5], nav[8:11], self.offset)

        if 0 <= column_index[2] < self.wide and 0 <= column_index[3] < self.wide and 0 <= column_index[4] < self.wide:
            self.errs[column_index[2]] = Error(self.times,
                                               wgs84.dE(ref_new[:, column_index[2]] * np.pi / 180.0,
                                                        target_new[:, column_index[3]] * np.pi / 180.0,
                                                        ref_new[:, column_index[3]] * np.pi / 180.0),
                                               self.bins)
            self.errs[column_index[3]] = Error(self.times,
                                               wgs84.dN(target_new[:,
                                                        column_index[2]] * np.pi / 180.0,
                                                        ref_new[:, column_index[2]] * np.pi / 180.0),
                                               self.bins)

            self.errs[column_index[4]] = Error(self.times, target_new[:, column_index[4]] - ref_new[:, column_index[4]],
                                               self.bins)

            if _2d:  # 统计平面定位误差
                _2d_error = [np.sqrt(d[0] ** 2 + d[1] ** 2) for d in
                             zip(self.errs[column_index[2]].error, self.errs[column_index[3]].error)]
                self.error_2d = Error(self.times, np.array(_2d_error), self.bins)

            if _3d:  # 统计3D平面误差
                _3d_error = [np.sqrt(d[0] ** 2 + d[1] ** 2 + d[2] ** 2) for d in
                             zip(self.errs[column_index[2]].error, self.errs[column_index[3]].error,
                                 self.errs[column_index[4]].error)]
                self.error_3d = Error(self.times, np.array(_3d_error), self.bins)

        self.exception = self.errs[0].exception
        for k in [5, 6, 7, 8, 9]:
            if column_index[k] < 0 or column_index[k] > self.wide: continue
            self.errs[column_index[k]] = Error(self.times, target_new[:, column_index[k]] - ref_new[:, column_index[k]],
                                               self.bins)

        # 航向拿出来单独算
        index = column_index[10]
        if 0 <= index <= self.wide:
            yaw_error = target_new[:, index] - ref_new[:, index]
            while np.any(yaw_error > 180):
                yaw_error[yaw_error > 180] -= 360
            while np.any(yaw_error < -180):
                yaw_error[yaw_error < -180] += 360
            self.errs[index] = Error(self.times, yaw_error, self.bins)

        if self.write2file:
            print("analyse finished,write to file....")
            error = np.zeros([self.times.shape[0], 13])
            error[:, 1] = self.times
            for i in range(2, 11):
                if column_index[i] > 0:
                    error[:, i] = self.errs[column_index[i]].error
                else:
                    error[:, i] = 0
            error[:, 11] = self.error_2d.error
            error[:, 12] = self.error_3d.error
            df = pd.DataFrame(error,
                              columns=['week', 'gpst', 'lat', 'lon', 'h', 'vx', 'vy', 'vz', 'roll', 'pitch', 'yaw',
                                       '2d', '3d'])
            df.to_csv(self.target_file + ".err.csv", header=True, index=False)
