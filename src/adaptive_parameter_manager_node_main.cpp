#include "rclcpp/rclcpp.hpp"
#include "adaptive_parameter_manager.h" // Assuming this path is correct relative to include directories

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto manager_node = std::make_shared<loam_adaptive_parameter_manager::AdaptiveParameterManager>();
    rclcpp::spin(manager_node);
    rclcpp::shutdown();
    return 0;
}
