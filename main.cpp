#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/alpha_wrap_3.h>
#include <CGAL/IO/read_points.h>
#include <CGAL/Real_timer.h>
#include <iostream>
#include <string>
#include "./Visualizer.h"

namespace AW3 = CGAL::Alpha_wraps_3;

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point_3 = K::Point_3;
using Point_container = std::vector<Point_3>;
using Mesh = CGAL::Surface_mesh<Point_3>;

void saveMesh(const Mesh& mesh, const std::string& filename, double relative_alpha,
              double relative_offset)
{
  std::string input_name = std::string(filename);
  input_name = input_name.substr(input_name.find_last_of("/") + 1, input_name.length() - 1);
  input_name = input_name.substr(0, input_name.find_last_of("."));
  std::string output_name = input_name + "_" + std::to_string(static_cast<int>(relative_alpha))
                            + "_" + std::to_string(static_cast<int>(relative_offset)) + ".off";
  std::cout << "Writing to " << output_name << std::endl;

  CGAL::IO::write_polygon_mesh(output_name, mesh, CGAL::parameters::stream_precision(17));
}

int main(int argc, char** argv)
{
  std::cout.precision(17);

  // Read the input
  const std::string filename = (argc > 1) ? argv[1] : "../assets/mapPoints.xyz";
  std::cout << "Reading " << filename << "..." << std::endl;
  Point_container points;

  if(!CGAL::IO::read_points(filename, std::back_inserter(points)) || points.empty())
  {
    std::cerr << "Invalid input." << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << points.size() << " points" << std::endl;

  const double relative_alpha = (argc > 2) ? std::stod(argv[2]) : 10.;
  const double relative_offset = (argc > 3) ? std::stod(argv[3]) : 300.;

  Visualizer visualizer(relative_alpha, relative_offset);
  visualizer.setPointCloud(points);
  visualizer.triggerWrap();

  // Update the pose every 0.1 seconds for 20 seconds
  for (int i = 0; i < 200; i++)
  {
    visualizer.updatePose(Point_3(i * 0.1, 0.0, 0.0));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Construct the wrap
  Mesh wrap = visualizer.getFinalMesh();

  // Save the wrap
  saveMesh(wrap, filename, relative_alpha, relative_offset);

  return EXIT_SUCCESS;
}
