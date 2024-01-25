#ifndef SLAM_IN_AUTO_DRIVING_IO_UTILS_H
#define SLAM_IN_AUTO_DRIVING_IO_UTILS_H
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <fstream>
#include <functional>
#include <utility>
#include "gnss.h"
#include "imu.h"
#include "math_utils.h"
#include "odom.h"

#include "dataset_type.h"
#include "global_flags.h"
#include "imu_gnss/utm_convert.h"
#include "lidar_utils.h"
#include "livox_ros_driver/CustomMsg.h"
#include "message_def.h"
#include "tools/pointcloud_convert/velodyne_convertor.h"

namespace lh {
// 读取数据文件
class TxtIO {
   public:
    TxtIO(const std::string& file_path) : fin(file_path) {}

    using IMUProcessFuncType = std::function<void(const IMU&)>;
    using GNSSProcessFuncType = std::function<void(const GNSS&)>;
    using OdomProcessFUncType = std::function<void(const Odom&)>;

    TxtIO& SetIMUProcessFunc(IMUProcessFuncType imu_proc) {
        imu_proc_ = std::move(imu_proc);
        return *this;
    }

    TxtIO& SetGNSSProcessFunc(GNSSProcessFuncType gnss_proc) {
        gnss_proc_ = std::move(gnss_proc);
        return *this;
    }

    TxtIO& SetOdomProcessFUncType(OdomProcessFUncType odom_proc) {
        odom_proc_ = std::move(odom_proc);
        return *this;
    }
    // 遍历文件内容
    void Go();

   private:
    std::ifstream fin;
    IMUProcessFuncType imu_proc_;
    GNSSProcessFuncType gnss_proc_;
    OdomProcessFUncType odom_proc_;
};

class RosbagIO {
   public:
    using MessageProcessFunction = std::function<bool(const rosbag::MessageInstance& m)>;
    using Scan2DHandler = std::function<bool(Scan2d::Ptr)>;
    using MultiScan2DHandler = std::function<bool(MutilScan2d::Ptr)>;
    using PointCloud2Handler = std::function<bool(sensor_msgs::PointCloud2::Ptr)>;
    using FullPointCloudHandler = std::function<bool(FullCloudPtr)>;
    using ImuHandler = std::function<bool(IMUPtr)>;
    using GNSSHandler = std::function<bool(GNSSPtr)>;
    using OdomHandler = std::function<bool(const Odom&)>;
    using LivoxHandler = std::function<bool(const livox_ros_driver::CustomMsg::ConstPtr& msg)>;

   public:
    explicit RosbagIO(std::string bag_file, DatasetType dataset_type = DatasetType::NCLT)
        : bag_file_(std::move(bag_file)), dataset_type_(dataset_type) {
        assert(dataset_type_ != DatasetType::UNKNOWN);
    }
    /**
     * @brief 遍历文件，调用回调函数
     */
    void Go();
    /**
     * @brief 添加消息处理函数
     */
    RosbagIO& AddHandler(const std::string& topic_name, MessageProcessFunction func) {
        process_func_.emplace(topic_name, func);
        return *this;
    }
    /**
     * @brief 2D激光处理函数
     */
    RosbagIO& AddScan2DHandler(const std::string& topic_name, Scan2DHandler func) {
        return AddHandler(topic_name, [func](const rosbag::MessageInstance& m) -> bool {
            auto msg = m.instantiate<sensor_msgs::LaserScan>();
            if (msg == nullptr) {
                return false;
            }
            // 调用2d激光处理函数
            return func(msg);
        });
    }
    /**
     * @brief 多回波2d激光
     */
    RosbagIO& AddMultiScan2DHandler(const std::string& topic_name, MultiScan2DHandler func) {
        return AddHandler(topic_name, [func](const rosbag::MessageInstance& m) -> bool {
            auto msg = m.instantiate<MutilScan2d>();
            if (msg == nullptr) {
                return false;
            }
            return func(msg);
        });
    }

    /**
     * @brief 根据数据集自动确定topic 名称
     */
    RosbagIO& AddAutoPointCloudHandler(PointCloud2Handler func) {
        if (dataset_type_ == DatasetType::WXB_3D) {
            return AddHandler(wxb_lidar_topic, [func, this](const rosbag::MessageInstance& m) -> bool {
                auto msg = m.instantiate<PacketsMsg>();
                if (msg == nullptr) {
                    return false;
                }
                FullCloudPtr cloud(new FullPointCloudType);
                // FullCloudPtr cloud_out(new FullPointCloudType);
                vlp_parser_.ProcessScan(msg, cloud);
                sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
                pcl::toROSMsg(*cloud, *cloud_msg);
                return func(cloud_msg);
            });
        } else if (dataset_type_ == DatasetType::AVIA) {
            return *this;
        } else {
            // [func](const rosbag::MessageInstance& m) 这个函数再GO中调用
            return AddHandler(GetLidarTopicName(), [func](const rosbag::MessageInstance& m) -> bool {
                auto msg = m.instantiate<sensor_msgs::PointCloud2>();

                if (msg == nullptr) {
                    return false;
                }
                return func(msg);
            });
        }
    }

    /**
     * @brief rtk处理函数
     */
    RosbagIO& AddAutoRTKHandler(GNSSHandler func) {
        if (dataset_type_ == DatasetType::NCLT) {
            return AddHandler(nclt_rtk_topic, [func, this](const rosbag::MessageInstance& m) -> bool {
                auto msg = m.instantiate<sensor_msgs::NavSatFix>();
                if (msg == nullptr) {
                    return false;
                }
                GNSSPtr gnss(new GNSS(msg));
                ConvertGps2UTMOnlyTrans(*gnss);
                if (std::isnan(gnss->lat_lon_alt_[2])) {
                    return false;
                }
            });
        } else {
            // 其他RTK数据
        }
    }

    /**
     * @brief point cloud
     */
    RosbagIO& AddPointCloud2Handle(const std::string& topic_name, PointCloud2Handler func) {
        return AddHandler(topic_name, [func](const rosbag::MessageInstance& m) -> bool {
            auto msg = m.instantiate<sensor_msgs::PointCloud2>();
            if (msg == nullptr) {
                return false;
            }
            return func(msg);
        });
    }

    /**
     * @brief livox msg
     */

    RosbagIO& AddLivoxHandle(LivoxHandler func) {
        return AddHandler(GetLidarTopicName(), [func, this](const rosbag::MessageInstance& m) -> bool {
            auto msg = m.instantiate<livox_ros_driver::CustomMsg>();
            if (msg == nullptr) {
                LOG(INFO) << "cannot inst: " << m.getTopic();
                return false;
            }
            return func(msg);
        });
    }

    /**
     * @brief velodyne packets
     */
    RosbagIO& AddVelodyneHandle(const std::string& topic_name, FullPointCloudHandler f) {
        return AddHandler(topic_name, [f, this](const rosbag::MessageInstance& m) -> bool {
            auto msg = m.instantiate<PacketsMsg>();
            if (msg == nullptr) {
                return false;
            }

            FullCloudPtr cloud(new FullPointCloudType), cloud_out(new FullPointCloudType);
            vlp_parser_.ProcessScan(msg, cloud);

            return f(cloud);
        });
    }

    // imu
    RosbagIO& AddImuHandler(ImuHandler func);

    void CleanProcessFunc() { process_func_.clear(); }

   private:
    std::string GetLidarTopicName() const;

    std::string GetIMUTopicName() const;

   private:
    std::map<std::string, MessageProcessFunction> process_func_;
    std::string bag_file_;
    DatasetType dataset_type_;

    // packets
    lh::tools::VelodyneConvertor vlp_parser_;
};
}  // namespace lh

#endif