/*
 * @Description: GNSS信息
 * @Author: Xiangmy
 * @Date: 2021-08-31
 */

#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_GNSS_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_GNSS_DATA_HPP_

#include <deque>

#include "Geocentric/LocalCartesian.hpp"

namespace lidar_localization
{
    class GNSSData
    {
    public:
        double time = 0.0;
        double longitude = 0.0; // 经度
        double latitude = 0.0; // 纬度
        double altitude = 0.0; // 海拔高
        double local_E = 0.0; // 米制坐标X
        double local_N = 0.0; // 米制坐标Y
        double local_U = 0.0; // 米制坐标Z
        int status = 0;
        int service = 0;

    private:
        static GeographicLib::LocalCartesian geo_converter;
        static bool origin_position_inited;

    public:
        void InitOriginPosition();
        void UpdateXYZ();
        static bool SyncData(std::deque<GNSSData>& UnsyncedData, std::deque<GNSSData>& SyncedData, double sync_time);
    };
}

#endif