#include "PlyWriter.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <iostream> // For std::cout, std::endl

// Helper function to create a sample point cloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr createSampleCloud(int num_points, float offset_x, float offset_y, float offset_z, uint8_t r, uint8_t g, uint8_t b) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud->width = num_points;
    cloud->height = 1; // Unordered point cloud
    cloud->points.resize(cloud->width * cloud->height);

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        cloud->points[i].x = offset_x + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 5.0));
        cloud->points[i].y = offset_y + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 5.0));
        cloud->points[i].z = offset_z + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 5.0));
        cloud->points[i].r = r;
        cloud->points[i].g = g;
        cloud->points[i].b = b;
    }
    return cloud;
}

int main() {
    // Define the output directory for PLY files
    // Assuming the executable will be run from a directory where "output_ply" can be created or exists.
    // For simplicity, let's use a relative path.
    // In a real application, ensure this directory exists.
    const std::string output_dir = "./output_ply";

    // Attempt to create the directory.
    // This is a simple platform-dependent way for POSIX systems.
    // For Windows, you might need CreateDirectory.
    // Or use C++17 std::filesystem::create_directory.
    // For now, let's assume it might fail silently or succeed.
    // A robust solution would check and handle errors.
    // #include <sys/stat.h> // For mkdir on POSIX
    // mkdir(output_dir.c_str(), 0777); // Example for POSIX, needs <sys/stat.h>

    std::cout << "Attempting to use output directory: " << output_dir << std::endl;
    std::cout << "Please ensure this directory exists or can be created." << std::endl;


    // Create a PlyWriter instance
    pointcloud_accumulator::PlyWriter ply_writer(output_dir);

    // Open a new PLY file
    if (!ply_writer.open()) {
        std::cerr << "Failed to open PLY file. Exiting." << std::endl;
        return 1;
    }

    std::cout << "PLY file opened successfully." << std::endl;

    // Create and append first sample point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 = createSampleCloud(100, 0.0f, 0.0f, 0.0f, 255, 0, 0); // 100 red points
    std::cout << "Appending first point cloud (" << cloud1->size() << " points)..." << std::endl;
    if (!ply_writer.appendPoints(*cloud1)) {
        std::cerr << "Failed to append first point cloud." << std::endl;
        ply_writer.finalize(); // Try to finalize even if an append fails
        return 1;
    }
    std::cout << "First point cloud appended." << std::endl;


    // Create and append second sample point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 = createSampleCloud(150, 5.0f, 0.0f, 0.0f, 0, 255, 0); // 150 green points
    std::cout << "Appending second point cloud (" << cloud2->size() << " points)..." << std::endl;
    if (!ply_writer.appendPoints(*cloud2)) {
        std::cerr << "Failed to append second point cloud." << std::endl;
        ply_writer.finalize();
        return 1;
    }
    std::cout << "Second point cloud appended." << std::endl;

    // Create and append third sample point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud3 = createSampleCloud(120, 0.0f, 5.0f, 0.0f, 0, 0, 255); // 120 blue points
    std::cout << "Appending third point cloud (" << cloud3->size() << " points)..." << std::endl;
    if (!ply_writer.appendPoints(*cloud3)) {
        std::cerr << "Failed to append third point cloud." << std::endl;
        ply_writer.finalize();
        return 1;
    }
    std::cout << "Third point cloud appended." << std::endl;


    // Finalize the PLY file (updates header, closes file)
    std::cout << "Finalizing PLY file..." << std::endl;
    ply_writer.finalize();
    std::cout << "PLY file finalized." << std::endl;

    std::cout << "Program finished successfully. Check the '" << output_dir << "' directory for the PLY file." << std::endl;

    return 0;
}
