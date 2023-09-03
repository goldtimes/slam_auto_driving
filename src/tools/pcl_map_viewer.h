#ifndef SLAM_IN_AUTO_DRIVING_PCL_MAP_VIEWER_H
#define SLAM_IN_AUTO_DRIVING_PCL_MAP_VIEWER_H

#include <glog/logging.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "common/point_cloud_utils.h"
#include "common/point_types.h"

namespace lh {
class PCLMapViewer {
   public:
    // 构造函数，需要初始化点云
    PCLMapViewer(const float& leaf_size, bool use_pcl_vis = true)
        : leaf_size_(leaf_size), tmp_cloud_(new PointCloudType), local_map_(new PointCloudType) {
        if (use_pcl_vis) {
            viewer_.reset(new pcl::visualization::PCLVisualizer());
            viewer_->addCoordinateSystem(10, "world");
        } else {
            viewer_ = nullptr;
        }
        voxel_filter_.setLeafSize(leaf_size, leaf_size, leaf_size);
    }
    /**
     * @brief 增加pose和它的点云(世界坐标系)
     */
    void SetPoseAndCloud(const SE3& pose, CloudPtr cloud_world) {
        voxel_filter_.setInputCloud(cloud_world);
        voxel_filter_.filter(*tmp_cloud_);
        // 将关键帧点云拼接成地图
        *local_map_ += *tmp_cloud_;
        // 过滤
        voxel_filter_.setInputCloud(local_map_);
        voxel_filter_.filter(*local_map_);

        if (viewer_ != nullptr) {
            viewer_->removePointCloud("local_map");
            viewer_->removeCoordinateSystem("vehicle");

            pcl::visualization::PointCloudColorHandlerGenericField<PointType> fieldColor(local_map_, "z");
            viewer_->addPointCloud<PointType>(local_map_, fieldColor, "local_map");

            Eigen::Affine3f T;
            T.matrix() = pose.matrix().cast<float>();
            viewer_->addCoordinateSystem(5, T, "vehicle");
            viewer_->spinOnce(1);
        }
        // local_map点云过多了
        if (local_map_->size() > 600000) {
            leaf_size_ *= 1.26;
            voxel_filter_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
            LOG(INFO) << "viewer set leaf size to: " << leaf_size_;
        }
    }
    /**
     * @brief 存储地图
     */
    void SaveMap(std::string path) {
        if (local_map_->size() > 0) {
            lh::SaveCloudToFile(path, *local_map_);
            LOG(INFO) << "save map to: " << path;
        } else {
            LOG(INFO) << "map is empty" << path;
        }
    }

    void Clean() {
        tmp_cloud_->clear();
        local_map_->clear();
    }

    void ClearAndResetLeafSize(const float leaf_size) {
        leaf_size_ = leaf_size;
        local_map_->clear();
        voxel_filter_.setLeafSize(leaf_size, leaf_size, leaf_size);
    }

   private:
    pcl::VoxelGrid<PointType> voxel_filter_;
    pcl::visualization::PCLVisualizer::Ptr viewer_ = nullptr;
    float leaf_size_ = 1.0;
    // 当前帧点云
    CloudPtr tmp_cloud_;
    // 局部地图
    CloudPtr local_map_;
};
}  // namespace lh

#endif