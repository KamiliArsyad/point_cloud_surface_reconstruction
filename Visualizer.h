#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/alpha_wrap_3.h>
#include <CGAL/IO/read_points.h>
#include <CGAL/Real_timer.h>

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
  std::thread poseConsumerThread;

  // House keeping for threads ----------------
  std::mutex pointCloudMutex;
  std::mutex poseMutex;
  std::condition_variable pointCloudSignal;
  std::condition_variable poseSignal;

  bool pointCloudReady = false;
  bool poseReady = false;
  bool pointCloudConsumerRunning = false;
  bool poseConsumerRunning = false;
  bool pointCloudConsumerFinished = false;
  bool poseConsumerFinished = false;
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

  // Functions -------------------------------
  void pointCloudConsumer();
  void poseConsumer();
  void processPointCloud();
  void initDiagonalLength(std::vector<Point_3> points);
public:
  Visualizer(double relative_alpha, double relative_offset);
  ~Visualizer();

  void addPointCloud(std::vector<Point_3> points);
  void setPointCloud(std::vector<Point_3> points);
  void updatePose(Point_3 pose);
  void triggerWrap();
}