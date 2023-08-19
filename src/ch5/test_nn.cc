#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include "bfnn.h"
#include "common/point_cloud_utils.h"
#include "common/point_types.h"
#include "common/sys_utils.h"
#include "gridnn.hpp"

DEFINE_string(first_scan_path, "/home/slam_auto_driving/data/ch5/first.pcd", "第一个点云路径");
DEFINE_string(second_scan_path, "/home/slam_auto_driving/data/ch5/second.pcd", "第二个点云路径");

TEST(CH5_TEST, BFNN) {
    lh::CloudPtr first(new lh::PointCloudType);
    lh::CloudPtr second(new lh::PointCloudType);
    pcl::io::loadPCDFile(FLAGS_first_scan_path, *first);
    pcl::io::loadPCDFile(FLAGS_second_scan_path, *second);

    if (first->empty() || second->empty()) {
        LOG(ERROR) << "cannot load cloud";
        FAIL();
    }
    // 体素滤波
    lh::VoxelGrid(first);
    lh::VoxelGrid(second);

    LOG(INFO) << "points: " << first->size() << ", " << second->size();

    // 单线程和多线程的暴力匹配
    evaluate_and_call(
        [&first, &second]() {
            std::vector<std::pair<size_t, size_t>> mathces;
            lh::bfnn_cloud(first, second, mathces);
        },
        "暴力匹配(单线程)", 5);  // 18000个点，可以看到单次调用要2.1s
    evaluate_and_call(
        [&first, &second]() {
            std::vector<std::pair<size_t, size_t>> mathces;
            lh::bfnn_cloud_mt(first, second, mathces);
        },
        "暴力匹配(多线程)", 5);  // 使用多线程也需要0.198s
    SUCCEED();
}

/**
 * 评测最近邻的正确性
 * @param truth 真值
 * @param esti  估计
 */
void EvaluateMatches(const std::vector<std::pair<size_t, size_t>>& truth, const std::vector<std::pair<size_t, size_t>>& esti) {
    // 假阳性---- 在esti存在，但是truth中不存在
    int fp = 0;
    // 假阴性--- 在truth存在，esti不存在
    int fn = 0;

    LOG(INFO) << "truth: " << truth.size() << ", esti: " << esti.size();

    // 定义函数指针
    auto exist = [](const std::pair<size_t, size_t>& data, const std::vector<std::pair<size_t, size_t>>& vec) -> bool {
        return std::find(vec.begin(), vec.end(), data) != vec.end();
    };

    int effective_esti = 0;
    for (const auto& data : esti) {
        if (data.first != lh::math::kINVALID_ID && data.second != lh::math::kINVALID_ID) {
            effective_esti++;
            if (!exist(data, truth)) {
                fp++;  // 假阳性
            }
        }
    }

    for (const auto& data : truth) {
        if (!exist(data, esti)) {
            fn++;
        }
    }

    float precision = 1.0 - float(fp) / effective_esti;
    float recall = 1.0 - float(fn) / truth.size();
    LOG(INFO) << "precision: " << precision << ", recall: " << recall << ", fp: " << fp << ", fn: " << fn;
}

TEST(CH5_TEST, GRID_NN) {
    lh::CloudPtr first(new lh::PointCloudType);
    lh::CloudPtr second(new lh::PointCloudType);
    pcl::io::loadPCDFile(FLAGS_first_scan_path, *first);
    pcl::io::loadPCDFile(FLAGS_second_scan_path, *second);

    if (first->empty() || second->empty()) {
        LOG(ERROR) << "cannot load cloud";
        FAIL();
    }

    // 滤波
    lh::VoxelGrid(first);
    lh::VoxelGrid(second);

    LOG(INFO) << "points: " << first->size() << ", " << second->size();

    std::vector<std::pair<size_t, size_t>> truth_matches;
    lh::bfnn_cloud(first, second, truth_matches);

    // 2d栅格
    lh::GridNN<2> grid0(0.1, lh::GridNN<2>::NearbyType::CENTER), grid4(0.1, lh::GridNN<2>::NearbyType::NEARBY4),
        grid8(0.1, lh::GridNN<2>::NearbyType::NEARBY8);
    // 3d 栅格
    lh::GridNN<3> grid3(0.1, lh::GridNN<3>::NearbyType::NEARBY6);

    grid0.SetPointCloud(first);
    grid4.SetPointCloud(first);
    grid8.SetPointCloud(first);
    grid3.SetPointCloud(first);

    // 评价各种版本的Grid NN
    // sorry没有C17的template lambda... 下面必须写的啰嗦一些
    LOG(INFO) << "===================";
    std::vector<std::pair<size_t, size_t>> matches;
    evaluate_and_call([&first, &second, &grid0, &matches]() { grid0.GetClosestPointForCloud(first, second, matches); }, "Grid0 单线程", 10);
    EvaluateMatches(truth_matches, matches);

    LOG(INFO) << "===================";
    evaluate_and_call([&first, &second, &grid0, &matches]() { grid0.GetClosestPointForCloudMT(first, second, matches); }, "Grid0 多线程", 10);
    EvaluateMatches(truth_matches, matches);

    LOG(INFO) << "===================";
    evaluate_and_call([&first, &second, &grid4, &matches]() { grid4.GetClosestPointForCloud(first, second, matches); }, "Grid4 单线程", 10);
    EvaluateMatches(truth_matches, matches);

    LOG(INFO) << "===================";
    evaluate_and_call([&first, &second, &grid4, &matches]() { grid4.GetClosestPointForCloudMT(first, second, matches); }, "Grid4 多线程", 10);
    EvaluateMatches(truth_matches, matches);

    LOG(INFO) << "===================";
    evaluate_and_call([&first, &second, &grid8, &matches]() { grid8.GetClosestPointForCloud(first, second, matches); }, "Grid8 单线程", 10);
    EvaluateMatches(truth_matches, matches);

    LOG(INFO) << "===================";
    evaluate_and_call([&first, &second, &grid8, &matches]() { grid8.GetClosestPointForCloudMT(first, second, matches); }, "Grid8 多线程", 10);
    EvaluateMatches(truth_matches, matches);

    LOG(INFO) << "===================";
    evaluate_and_call([&first, &second, &grid3, &matches]() { grid3.GetClosestPointForCloud(first, second, matches); }, "Grid 3D 单线程", 10);
    EvaluateMatches(truth_matches, matches);

    LOG(INFO) << "===================";
    evaluate_and_call([&first, &second, &grid3, &matches]() { grid3.GetClosestPointForCloudMT(first, second, matches); }, "Grid 3D 多线程", 10);
    EvaluateMatches(truth_matches, matches);

    SUCCEED();
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;

    testing::InitGoogleTest(&argc, argv);
    google::ParseCommandLineFlags(&argc, &argv, true);

    return RUN_ALL_TESTS();
}