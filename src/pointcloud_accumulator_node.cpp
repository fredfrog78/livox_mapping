#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "PlyWriter.h" // Assuming this is in the same include path or relative path is adjusted
#include <memory>
#include <csignal> // For signal handling (optional, can rely on destructor for now)
#include <filesystem> // For creating directory if it doesn't exist

namespace pointcloud_accumulator {

class PointCloudAccumulatorNode : public rclcpp::Node {
public:
    PointCloudAccumulatorNode() : Node("pointcloud_accumulator_node") {
        // Declare and get parameter
        this->declare_parameter<std::string>("output_directory", "./ply_maps");
        this->get_parameter("output_directory", output_directory_param_);

        RCLCPP_INFO(this->get_logger(), "Output directory set to: %s", output_directory_param_.c_str());

        // Create output directory if it doesn't exist
        try {
            if (!std::filesystem::exists(output_directory_param_)) {
                if (std::filesystem::create_directories(output_directory_param_)) {
                    RCLCPP_INFO(this->get_logger(), "Created output directory: %s", output_directory_param_.c_str());
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to create output directory: %s. Shutting down.", output_directory_param_.c_str());
                    rclcpp::shutdown();
                    return;
                }
            }
        } catch (const std::filesystem::filesystem_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Filesystem error while checking/creating directory: %s. Error: %s. Shutting down.", output_directory_param_.c_str(), e.what());
            rclcpp::shutdown();
            return;
        }

        ply_writer_ = std::make_unique<PlyWriter>(output_directory_param_);

        if (!ply_writer_->open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open PLY file for writing. Shutting down.");
            // rclcpp::shutdown(); // This can be problematic if called from constructor before node is fully spun up.
                                 // Better to let it construct and then it will fail to process.
                                 // Or throw an exception. For now, log and it will likely fail on callback.
            // Forcing a shutdown here can lead to issues if the node isn't fully managed by an executor yet.
            // A common pattern is to throw an exception and let the main() catch it, or allow it to construct
            // and subsequent operations will fail.
            // For simplicity in this context, we'll allow it to construct. The first callback will then fail.
            // However, if open() fails, the PlyWriter object might be in an unusable state.
            // A more robust approach would be to throw and catch in main, or use a factory function.
            // Given the current structure, if open fails, subsequent calls to ply_writer_ will likely also fail or crash.
            // Let's initiate shutdown more cleanly if possible, or make it clear initialization failed.
            // A simple way for a node to stop itself is to request shutdown.
             if (rclcpp::ok()) { // only call shutdown if rclcpp is running
                RCLCPP_ERROR(this->get_logger(), "Initiating shutdown due to PLY writer open failure.");
                rclcpp::shutdown();
             }
            return; // Prevent further initialization like subscription
        }

        // Configure QoS for sensor data
        auto qos = rclcpp::SensorDataQoS();
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_cloud_registered", qos,
            std::bind(&PointCloudAccumulatorNode::pointcloudCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "PointCloudAccumulatorNode initialized, waiting for data on %s.", subscription_->get_topic_name());

        // Optional: More robust shutdown handling
        // rclcpp::on_shutdown([this](){ this->custom_shutdown_handler(); });
    }

    ~PointCloudAccumulatorNode() {
        RCLCPP_INFO(this->get_logger(), "Shutting down PointCloudAccumulatorNode. Finalizing PLY file...");
        if (ply_writer_ && ply_writer_->isFileOpen()) { // Add a check if file is actually open
            ply_writer_->finalize();
            RCLCPP_INFO(this->get_logger(), "PLY file finalized.");
        } else {
            RCLCPP_INFO(this->get_logger(), "PLY writer was not active or file not open; no finalization needed or possible.");
        }
    }

private:
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (!ply_writer_ || !ply_writer_->isFileOpen()) {
            RCLCPP_WARN(this->get_logger(), "PLY writer is not available or file not open. Skipping point cloud.");
            return;
        }

        RCLCPP_DEBUG(this->get_logger(), "Received PointCloud2 message with %u points.", msg->width * msg->height);

        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        // It's important that the PCL PointType matches the fields in PointCloud2.
        // If PointCloud2 has XYZ and Intensity, but not RGB, this conversion might create points with default RGB (0,0,0).
        // For this example, we assume PointXYZRGB is appropriate.
        try {
            pcl::fromROSMsg(*msg, cloud);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert ROS message to PCL cloud: %s", e.what());
            return;
        }


        if (cloud.empty() && (msg->width * msg->height > 0)) {
             RCLCPP_WARN(this->get_logger(), "Conversion resulted in an empty PCL cloud, but ROS message reported %u points. Check field matching.", msg->width*msg->height);
            return;
        } else if (cloud.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received and converted an empty point cloud, skipping.");
            return;
        }


        if (ply_writer_->appendPoints(cloud)) {
            RCLCPP_DEBUG(this->get_logger(), "Appended %zu points to PLY file. Total points: %zu", cloud.size(), ply_writer_->getTotalPoints());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to append points to PLY file.");
            // Consider stopping subscription or shutting down if errors persist
        }
    }

    // void custom_shutdown_handler() { // Example for rclcpp::on_shutdown
    //     RCLCPP_INFO(this->get_logger(), "Custom shutdown handler called. Finalizing PLY file...");
    //     if (ply_writer_) {
    //         ply_writer_->finalize();
    //     }
    // }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    std::unique_ptr<PlyWriter> ply_writer_;
    std::string output_directory_param_;
};

} // namespace pointcloud_accumulator

// Add a new method to PlyWriter to check if file is open
// This is a quick way to do it for now. Ideally, PlyWriter.h would be modified.
// For the purpose of this step, we'll assume PlyWriter needs an isFileOpen() method.
// Since I cannot modify PlyWriter.h in this turn, I'll add a placeholder in comments
// and adjust the destructor logic.
// In PlyWriter.h, add:
// bool isFileOpen() const { return ply_file_.is_open(); }
// size_t getTotalPoints() const { return total_points_; } // Assuming this might also be useful for logging

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<pointcloud_accumulator::PointCloudAccumulatorNode> node;
    try {
        node = std::make_shared<pointcloud_accumulator::PointCloudAccumulatorNode>();
        if (rclcpp::ok()) { // Check if node construction called rclcpp::shutdown()
            rclcpp::spin(node);
        } else {
            RCLCPP_ERROR(node->get_logger(), "Node initialization failed, not spinning.");
        }
    } catch (const std::exception& e) {
        // If PlyWriter constructor or open threw an exception (it doesn't currently, but good practice)
        RCLCPP_FATAL(rclcpp::get_logger("pointcloud_accumulator_main"), "Node construction failed: %s", e.what());
        rclcpp::shutdown(); // Ensure shutdown if error happened before spinning
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
