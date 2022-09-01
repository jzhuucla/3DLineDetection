#include <stdio.h>
#include <fstream>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "../file_util.h"
#include "../LineDetection3D.h"
#include "../nanoflann.hpp"
#include "../utils.h"
#include "../Timer.h"

using namespace cv;
using namespace std;
using namespace nanoflann;

DEFINE_int32(k_nn, 20, "Specify the number of neighbor points used in normal estimation.");

DEFINE_string(cloud_name, "",
              "Specify the cloud file name (w/o extension name).");
DEFINE_string(input_dir,
              "/home/admin/work/data/semantic3d/reduced_8/train/",
              "Specify the directory path of input point cloud data.");
DEFINE_string(output_dir,
              "/home/admin/work/data/semantic3d/reduced_8/train/",
              "Specify the directory path of output data.");

static const size_t kNumPointInputCloudReserved = 100000000;

bool ReadDataFromFile(const std::string &file_path, PointCloud<double> &cloud)
{
  cloud.pts.reserve(kNumPointInputCloudReserved);
  LOG(INFO) << "Reading input cloud data from " << file_path;

  double x = 0, y = 0, z = 0;
  int intensity = 0;
  int r = 0, g = 0, b = 0;

  std::ifstream ptReader(file_path);
  if (ptReader.is_open())
  {
    while (!ptReader.eof())
    {
      ptReader >> x >> y >> z >> intensity >> r >> g >> b;
      cloud.pts.push_back(PointCloud<double>::PtData(x,y,z,r,g,b));
      if (cloud.pts.size() % 100000 == 0) {
        std::cout << "..";
      }
    }
    std::cout << std::endl;
    ptReader.close();
  }
  else
  {
    LOG(ERROR) << "Failed to open file: " << file_path;
    return false;
  }

  LOG(INFO) << "Total num of points: " << cloud.pts.size();
  return true;
}


int main(int argc, char* argv[])
{
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;

  LOG(INFO) << "k_nn: " << FLAGS_k_nn;
  LOG(INFO) << "cloud name:" << FLAGS_cloud_name;
  LOG(INFO) << "input directory:" << FLAGS_input_dir;
  LOG(INFO) << "output directory: " << FLAGS_output_dir;

  // Get paths
  const std::string input_cloud_file = FLAGS_input_dir + FLAGS_cloud_name + ".txt";
  const std::string input_ply_file = FLAGS_input_dir + FLAGS_cloud_name + ".ply";
  const std::string output_plane_file = FLAGS_output_dir + FLAGS_cloud_name + "_plane.ply";
  const std::string output_line_file = FLAGS_output_dir + FLAGS_cloud_name + "_line.ply";

  LOG(INFO) << "input_cloud_file: " << input_cloud_file;
  LOG(INFO) << "input_ply_file: " << input_ply_file;
  LOG(INFO) << "output_plane_file: " << output_plane_file;
  LOG(INFO) << "output_line_file: " << output_line_file;

  // Read input data
  PointCloud<double> input_cloud;
  if (!IsFileExist(input_ply_file)) {
    if (!ReadDataFromFile(input_cloud_file, input_cloud)) {
      return -1;
    }
    input_cloud.SaveToXYZRGBPlyFile(input_ply_file);
  } else {
    LOG(INFO) << "Read input cloud data from " << input_ply_file;
    input_cloud.ReadFromXYZRGBPlyFile(input_ply_file);
  }

  LOG(INFO) << "Run line detection ...";
  LineDetection3D detector;
  PlaneList planes;
  Line3DList lines;
  std::vector<double> ts;
  detector.run(input_cloud, FLAGS_k_nn, planes, lines, ts);
  LOG(INFO) << "Num of lines: " << lines.size();
  LOG(INFO) << "NUM of planes: " << planes.size();

  // WritePlanesToPLYFile(output_plane_file, planes, detector.scale);
  WriteLinesToPLYFile(output_line_file, lines, detector.scale);

  return 0;
}