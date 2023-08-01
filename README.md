# Surface Reconstruction from Stream of Point Cloud
This project implements CGAL's 3D Alpha Wrapping algorithm to reconstruct a surface from a stream of point cloud. The stream is processed in batches to save computational resources. The project is implemented in C++ and uses CGAL library for the surface reconstruction. The main purpose of this project is to visualize the surface reconstruction in real-time; meaning that another input, the pose of the device, is visualized on the reconstructed object.
This project is still under development and this readme will be updated as the project progresses.

## Dependencies
* [CGAL](https://www.cgal.org/) - Computational Geometry Algorithms Library


## Updates
* 01/08/2023 - Initial commit - running `./main <path_to_point_cloud> <alpha> <offset>` will output the reconstructed object in a `.off` file.