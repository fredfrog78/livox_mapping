#ifndef PLY_WRITER_H
#define PLY_WRITER_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <iomanip> // For std::put_time
#include <chrono>  // For system_clock

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pointcloud_accumulator {

class PlyWriter {
public:
    PlyWriter(const std::string& output_directory);
    ~PlyWriter();

    bool open();
    bool appendPoints(const pcl::PointCloud<pcl::PointXYZRGB>& cloud);
    void finalize();
    bool isFileOpen() const;
    size_t getTotalPoints() const;

private:
    bool writeHeader(size_t point_count); // Private method to write/rewrite header

    std::string output_directory_;
    std::string filepath_;
    std::fstream ply_file_; // Changed from std::ofstream to std::fstream
    size_t total_points_;
    bool header_written_; // To track if the initial header was written
};

} // namespace pointcloud_accumulator

#endif // PLY_WRITER_H
