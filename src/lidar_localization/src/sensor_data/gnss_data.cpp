/*
 * @Description: GNSS信息
 * @Author: Xiangmy
 * @Date: 2021-08-27
 */

#include "lidar_localization/sensor_data/gnss_data.hpp"

#include "glog/logging.h"

// 静态成员变量必须在类外初始化
bool lidar_localization::GNSSData::origin_position_inited = false;
GeographicLib::LocalCartesian lidar_localization::GNSSData::geo_converter;

namespace lidar_localization
{
    // 初始化
    void GNSSData::InitOriginPosition()
    {
        geo_converter.Reset(latitude, longitude, altitude);
        origin_position_inited = true;
    }

    // 将经纬高转换成米制坐标[E,N,U]
    void GNSSData::UpdateXYZ()
    {
        if (!origin_position_inited)
        {
            LOG(WARNING) << "GeoConverter has not set origin position";
        }
        
        geo_converter.Forward(latitude, longitude, altitude, local_E, local_N, local_U); // 使用地理计算库GeographicLib将经纬高转换成米制坐标[E,N,U]
    }
}