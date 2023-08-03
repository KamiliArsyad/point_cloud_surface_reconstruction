#include "Visualizer.h"

Visualizer::Visualizer(double relative_alpha, double relative_offset)
{
  this->relative_alpha = relative_alpha;
  this->relative_offset = relative_offset;

  poseConsumerThread = std::thread(&Visualizer::poseConsumer, this);
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
  std::unique_lock<std::mutex> lock(pointCloudMutex); 
  pointsToProcess = points;

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

  initDiagonalLength(points);
  this->alpha = diag_length / relative_alpha; 
  this->offset = diag_length / relative_offset;

  CGAL::alpha_wrap_3(points, alpha, offset, previewMesh);

  /**
   * Note: This is just binding the draw function to the drawPreviewMesh function. The lambda function is used
   * instead of std::bind such that the drawPreviewMesh function can be called with the current previewMesh
   * instead of the previewMesh captured by the bind function.
  */
  // this->drawFunction = std::bind(&Visualizer::drawPreviewMesh, this, previewMesh);
  this->drawFunction = [this]() { this->drawPreviewMesh(this->previewMesh); };
}

/// @brief Semi-transparent preview of the wrapped point cloud.
/// @param mesh
void Visualizer::drawPreviewMesh(Mesh mesh)
{
  // Draw the points of the mesh
  glPointSize(2.0);
  glBegin(GL_POINTS);
  glColor3f(1.0, 0.0, 0.0);

  for (auto v : mesh.vertices())
  {
    Point_3 p = mesh.point(v);
    glVertex3f(p.x(), p.y(), p.z());
  }

  glEnd();

  // // Draw the faces of the mesh
  // glBegin(GL_TRIANGLES);
  // glColor3f(0.0, 1.0, 0.0);

  // for (auto f : mesh.faces())
  // {
  //   // Random color
  //   glColor3f((float)rand() / RAND_MAX, (float)rand() / RAND_MAX, (float)rand() / RAND_MAX);

  //   auto h = mesh.halfedge(f);
  //   auto p1 = mesh.point(mesh.source(h));
  //   auto p2 = mesh.point(mesh.target(h));
  //   auto p3 = mesh.point(mesh.target(mesh.next(h)));

  //   glVertex3f(p1.x(), p1.y(), p1.z());
  //   glVertex3f(p2.x(), p2.y(), p2.z());
  //   glVertex3f(p3.x(), p3.y(), p3.z());
  // }

  // glEnd();

  // Draw the edges of the mesh
  glBegin(GL_LINES);
  glColor3f(0.0, 0.0, 1.0);

  for (auto e : mesh.edges())
  {
    CGAL::Surface_mesh<Point_3>::Halfedge_index h = mesh.halfedge(e, 0);
    Point_3 p1 = mesh.point(mesh.source(h));
    Point_3 p2 = mesh.point(mesh.target(h));

    glVertex3f(p1.x(), p1.y(), p1.z());
    glVertex3f(p2.x(), p2.y(), p2.z());

    h = mesh.halfedge(e, 1);
    p1 = mesh.point(mesh.source(h));
    p2 = mesh.point(mesh.target(h));

    glVertex3f(p1.x(), p1.y(), p1.z());
    glVertex3f(p2.x(), p2.y(), p2.z());
  }

  glEnd();
}

/// @brief Draw the pose of the device
/// @param pose
void Visualizer::drawPose(Point_3 pose)
{
  glPointSize(5.0);
  glBegin(GL_POINTS);
  glColor3f(1.0, 1.0, 1.0);

  glVertex3f(pose.x(), pose.y(), pose.z());

  glEnd();

  glLineWidth(2.0);
  glBegin(GL_LINES);
  glColor3f(1.0, 1.0, 1.0);

  glVertex3f(pose.x(), pose.y(), pose.z());
  glVertex3f(pose.x() + 1.0, pose.y(), pose.z());

  glVertex3f(pose.x(), pose.y(), pose.z());
  glVertex3f(pose.x(), pose.y() + 1.0, pose.z());

  glVertex3f(pose.x(), pose.y(), pose.z());
  glVertex3f(pose.x(), pose.y(), pose.z() + 1.0);

  glEnd();
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
  // Reusable variables
  // ------------------------------------------------
  Point_3 pose;
  
  // ------------------------------------------------
  // Main loop
  // ------------------------------------------------
  while (!pangolin::ShouldQuit())
  {
    // Acquire lock to the pose
    std::unique_lock<std::mutex> lock(poseMutex);
    poseSignal.wait(lock, [&] { return poseReady; });
    pose = this->pose;
    poseReady = false;  
    lock.unlock();

    // Clear screen and activate view to render into
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);

    std::unique_lock<std::mutex> meshLock(pointCloudMutex);
    this->drawFunction();
    meshLock.unlock();

    this->drawPose(pose);

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

/// @brief Get the final mesh
/// @return Mesh
Mesh Visualizer::getFinalMesh()
{
  std::unique_lock<std::mutex> lock(pointCloudMutex);

  return finalMesh;
}
