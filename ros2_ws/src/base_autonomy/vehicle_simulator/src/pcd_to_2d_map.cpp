#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
struct Options
{
  std::string input;
  std::string output_prefix;
  double resolution = 0.05;
  double z_min = -1.0;
  double z_max = 0.5;
  double radius = 0.5;
  int min_neighbors = 10;
  double padding = 0.5;
};

void print_usage(const char * program)
{
  std::cout
    << "Usage: " << program << " --input MAP.pcd --output-prefix DIR/NAME [options]\n"
    << "Options:\n"
    << "  --resolution M       grid resolution (default 0.05)\n"
    << "  --z-min M            minimum obstacle height (default -1.0)\n"
    << "  --z-max M            maximum obstacle height (default 0.5)\n"
    << "  --radius M           radius outlier search (default 0.5)\n"
    << "  --min-neighbors N    minimum neighbors in radius (default 10)\n"
    << "  --padding M          XY padding around points (default 0.5)\n";
}

Options parse_arguments(int argc, char ** argv)
{
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--help" || arg == "-h") {
      print_usage(argv[0]);
      std::exit(0);
    }
    if (i + 1 >= argc) {
      throw std::runtime_error("missing value for " + arg);
    }
    const std::string value(argv[++i]);
    if (arg == "--input") {
      options.input = value;
    } else if (arg == "--output-prefix") {
      options.output_prefix = value;
    } else if (arg == "--resolution") {
      options.resolution = std::stod(value);
    } else if (arg == "--z-min") {
      options.z_min = std::stod(value);
    } else if (arg == "--z-max") {
      options.z_max = std::stod(value);
    } else if (arg == "--radius") {
      options.radius = std::stod(value);
    } else if (arg == "--min-neighbors") {
      options.min_neighbors = std::stoi(value);
    } else if (arg == "--padding") {
      options.padding = std::stod(value);
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }

  if (options.input.empty() || options.output_prefix.empty()) {
    throw std::runtime_error("--input and --output-prefix are required");
  }
  if (options.resolution <= 0.0 || options.radius <= 0.0 || options.padding < 0.0) {
    throw std::runtime_error("resolution/radius must be positive and padding non-negative");
  }
  if (options.z_min > options.z_max || options.min_neighbors < 1) {
    throw std::runtime_error("invalid Z interval or min-neighbors");
  }
  return options;
}

std::string basename(const std::string & path)
{
  const std::string::size_type slash = path.find_last_of("/\\");
  return slash == std::string::npos ? path : path.substr(slash + 1);
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Options options = parse_arguments(argc, argv);
    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile(options.input, *cloud) != 0) {
      throw std::runtime_error("failed to load PCD: " + options.input);
    }

    auto height_slice = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>());
    height_slice->reserve(cloud->size());
    for (const auto & point : cloud->points) {
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
        point.z >= options.z_min && point.z <= options.z_max)
      {
        height_slice->push_back(point);
      }
    }
    if (height_slice->empty()) {
      throw std::runtime_error("height slice is empty; inspect the PCD Z distribution");
    }

    auto filtered = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_filter;
    radius_filter.setInputCloud(height_slice);
    radius_filter.setRadiusSearch(options.radius);
    radius_filter.setMinNeighborsInRadius(options.min_neighbors);
    radius_filter.filter(*filtered);
    if (filtered->empty()) {
      throw std::runtime_error("radius filter removed every point; relax its parameters");
    }

    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    for (const auto & point : filtered->points) {
      min_x = std::min(min_x, static_cast<double>(point.x));
      min_y = std::min(min_y, static_cast<double>(point.y));
      max_x = std::max(max_x, static_cast<double>(point.x));
      max_y = std::max(max_y, static_cast<double>(point.y));
    }

    const double origin_x = min_x - options.padding;
    const double origin_y = min_y - options.padding;
    const std::size_t width = static_cast<std::size_t>(
      std::ceil((max_x - min_x + 2.0 * options.padding) / options.resolution)) + 1;
    const std::size_t height = static_cast<std::size_t>(
      std::ceil((max_y - min_y + 2.0 * options.padding) / options.resolution)) + 1;
    if (width == 0 || height == 0 || width > 100000 || height > 100000) {
      throw std::runtime_error("unreasonable map dimensions; check outliers and resolution");
    }

    // Nav2 trinary map: black=occupied, white=free. PGM rows are top-down,
    // while the map origin is at the lower-left, so Y is deliberately flipped.
    std::vector<std::uint8_t> pixels(width * height, 254);
    std::size_t occupied_cells = 0;
    for (const auto & point : filtered->points) {
      const auto x = static_cast<std::size_t>(
        std::floor((point.x - origin_x) / options.resolution));
      const auto y = static_cast<std::size_t>(
        std::floor((point.y - origin_y) / options.resolution));
      if (x >= width || y >= height) {
        continue;
      }
      const std::size_t row = height - 1 - y;
      auto & pixel = pixels[row * width + x];
      if (pixel != 0) {
        pixel = 0;
        ++occupied_cells;
      }
    }

    const std::string pgm_path = options.output_prefix + ".pgm";
    const std::string yaml_path = options.output_prefix + ".yaml";
    std::ofstream pgm(pgm_path, std::ios::binary);
    if (!pgm) {
      throw std::runtime_error("cannot create " + pgm_path);
    }
    pgm << "P5\n" << width << " " << height << "\n255\n";
    pgm.write(reinterpret_cast<const char *>(pixels.data()), pixels.size());
    pgm.close();

    std::ofstream yaml(yaml_path);
    if (!yaml) {
      throw std::runtime_error("cannot create " + yaml_path);
    }
    yaml << "image: " << basename(pgm_path) << "\n"
         << "mode: trinary\n"
         << "resolution: " << std::setprecision(10) << options.resolution << "\n"
         << "origin: [" << origin_x << ", " << origin_y << ", 0.0]\n"
         << "negate: 0\n"
         << "occupied_thresh: 0.65\n"
         << "free_thresh: 0.196\n"
         // Conversion metadata. Nav2 ignores these extra keys, but keeping
         // them beside the PGM makes the projection reproducible.
         << "projection_z_min: " << options.z_min << "\n"
         << "projection_z_max: " << options.z_max << "\n";
    yaml.close();

    std::cout << "PCD points: " << cloud->size() << "\n"
              << "Height-slice points: " << height_slice->size() << "\n"
              << "Radius-filtered points: " << filtered->size() << "\n"
              << "Grid: " << width << " x " << height << " at "
              << options.resolution << " m/cell\n"
              << "Origin: [" << origin_x << ", " << origin_y << ", 0.0]\n"
              << "Occupied cells: " << occupied_cells << "\n"
              << "Wrote: " << pgm_path << "\n"
              << "Wrote: " << yaml_path << "\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "pcd_to_2d_map: " << error.what() << "\n";
    print_usage(argv[0]);
    return 2;
  }
}
