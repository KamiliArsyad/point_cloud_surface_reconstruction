#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/alpha_wrap_3.h>
#include <CGAL/IO/read_points.h>
#include <CGAL/Real_timer.h>

#include <pangolin/pangolin.h>

#include <functional>
#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>

namespace AW3 = CGAL::Alpha_wraps_3;

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point_3 = K::Point_3;
using Mesh = CGAL::Surface_mesh<Point_3>;

class Visualizer
{
private:
  std::thread pointCloudConsumerThread;
  // The main visualizer thread. Pangolin is used.
  std::thread poseConsumerThread;

  // House keeping for threads ----------------
  // Note that I refrained from using more mutexes and condition variables than this for ease of development.

  // Mutex for all point cloud and mesh related data structures (including the draw function).
  std::mutex pointCloudMutex;
  std::condition_variable pointCloudSignal;
  // Mutex for all pose related data structures.
  std::mutex poseMutex;
  std::condition_variable poseSignal;

  bool pointCloudReady = false;
  bool poseReady = false;

  // Function pointer to hold the draw function
  std::function<void()> drawFunction;
  // ------------------------------------------

  double diag_length;
  double alpha;
  double offset;
  double relative_alpha;
  double relative_offset;

  std::vector<Point_3> pointsToProcess;
  std::vector<Point_3> pointsProcessed;

  Mesh previewMesh;
  Mesh finalMesh;

  Point_3 pose;

  // Functions -------------------------------
  void pointCloudConsumer();
  void poseConsumer();
  void processPointCloud();
  void initDiagonalLength(std::vector<Point_3> points);

  void drawPreviewMesh(Mesh mesh);
  void drawMesh(Mesh mesh);
  void drawPose(Point_3 pose);

public:
  Visualizer(double relative_alpha, double relative_offset);
  ~Visualizer();

  Mesh getFinalMesh();

  void addPointCloud(std::vector<Point_3> points);
  void setPointCloud(std::vector<Point_3> points);
  void updatePose(Point_3 pose);
  void triggerWrap();
};