#include "Visualizer.h"

Visualizer::Visualizer(double relative_alpha, double relative_offset)
{
  this->relative_alpha = relative_alpha;
  this->relative_offset = relative_offset;
}

Visualizer::~Visualizer()
{
  if (pointCloudConsumerThread.joinable())
  {
    pointCloudConsumerThread.join();
  }

  if (poseConsumerThread.joinable())
  {
    poseConsumerThread.join();
  }
}

/// @brief Add a set of points to the existing point cloud to be processed
/// @param points 
void Visualizer::addPointCloud(std::vector<Point_3> points)
{
  std::unique_lock<std::mutex> lock(pointCloudMutex);
  pointsToProcess.insert(pointsToProcess.end(), points.begin(), points.end());

  pointCloudReady = true;
  pointCloudSignal.notify_one();
}

/// @brief Set the point cloud to be processed. This will overwrite any existing point cloud
/// @param points
void Visualizer::setPointCloud(std::vector<Point_3> points)
{
  std::unique_lock<std::mutex> lock(pointCloudMutex); pointsToProcess = points;

  pointCloudReady = true;
  pointCloudSignal.notify_one();
}

/// @brief Update the pose of the device
/// @param pose
void Visualizer::updatePose(Point_3 pose)
{
  std::unique_lock<std::mutex> lock(poseMutex);
  this->pose = pose;

  poseReady = true;
  poseSignal.notify_one();
}

/// @brief Trigger wrapping of the point cloud.
/// @details This process is asynchronous and will run in a separate thread.
void Visualizer::triggerWrap()
{
  pointCloudConsumerThread = std::thread(&Visualizer::pointCloudConsumer, this);
}

/// @brief The main point cloud consumer thread.
/// @details This thread will wait for the point cloud set to be ready, wrap them, display them, and then terminate.
void Visualizer::pointCloudConsumer()
{
  std::unique_lock<std::mutex> lock(pointCloudMutex);
  pointCloudSignal.wait(lock, [&] { return pointCloudReady; });

  processPointCloud();
  std::vector<Point_3> points = pointsProcessed;

  pointCloudReady = false;
  pointCloudSignal.notify_one();

  lock.unlock();

  initDiagonalLength(points);
  this->alpha = relative_alpha * diag_length;
  this->offset = relative_offset * diag_length;

  CGAL::alpha_wrap_3(points, alpha, offset, previewMesh);

  drawPreviewMesh(previewMesh);
}

/// @brief Semi-transparent preview of the wrapped point cloud.
/// @param mesh
void Visualizer::drawPreviewMesh(Mesh mesh)
{
  return;
}

/// @brief The main pose consumer thread.
/// @details This thread will wait for the pose to be ready, update the pose, and then display the wrapped point cloud.
void Visualizer::poseConsumer()
{
  // ------------------------------------------------
  // Pangolin stuff
  // ------------------------------------------------ 
  pangolin::CreateWindowAndBind("Main", 1024, 768);

  // 3D Mouse handler requires depth testing to be enabled
  glEnable(GL_DEPTH_TEST);

  // Issue specific OpenGl we might need
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // Menu stuff for later
  // ...

  // Define Camera Render Object (for view / scene browsing)
  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, 420, 420, 512, 389, 0.1, 1000),
      pangolin::ModelViewLookAt(-0, 0, -1, 0, 0, 1, pangolin::AxisNegY));

  // Add named OpenGL viewport to window and provide 3D Handler
  pangolin::View& d_cam = pangolin::CreateDisplay()
                              .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
                              .SetHandler(new pangolin::Handler3D(s_cam));
  
  // ------------------------------------------------
  // Main loop
  // ------------------------------------------------
  while (!pangolin::ShouldQuit())
  {
    // Clear screen and activate view to render into
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);

    // Render OpenGL Cube (TEST)
    pangolin::glDrawColouredCube();

    // Swap frames and Process Events
    pangolin::FinishFrame();
  }
}

// ------------------------------------------------
// Auxiliary functions ----------------------------
// ------------------------------------------------

/// @brief Calculate the diagonal length of the bounding box of the point cloud
/// @param points
void Visualizer::initDiagonalLength(std::vector<Point_3> points)
{
  CGAL::Bbox_3 bbox = CGAL::bbox_3(std::cbegin(points), std::cend(points));
  this->diag_length = std::sqrt(CGAL::square(bbox.xmax() - bbox.xmin()) +
                                CGAL::square(bbox.ymax() - bbox.ymin()) +
                                CGAL::square(bbox.zmax() - bbox.zmin()));
}

/// @brief Process the point cloud (e.g. remove outliers)
void Visualizer::processPointCloud()
{
  pointsProcessed = pointsToProcess;
}
