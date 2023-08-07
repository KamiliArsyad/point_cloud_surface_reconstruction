# Surface Reconstruction from Stream of Point Cloud
This project implements CGAL's 3D Alpha Wrapping algorithm to reconstruct a surface from a stream of point cloud. The stream is processed in batches to save computational resources. The project is implemented in C++ and uses CGAL library for the surface reconstruction. The main purpose of this project is to visualize the surface reconstruction in real-time; meaning that another input, the pose of the device, is visualized on the reconstructed object.
This project is still under development and this readme will be updated as the project progresses.

<img src="https://github.com/KamiliArsyad/point_cloud_surface_reconstruction/assets/22293969/196f58e1-7a2d-4f0c-a4f0-aee2db79e1a4" width="640">

*initial load*

<img src="https://github.com/KamiliArsyad/point_cloud_surface_reconstruction/assets/22293969/2d18d37c-7d8f-4031-a663-de0213498b85" width="640">

*adjust relative alpha and offset values*

<img src="https://github.com/KamiliArsyad/point_cloud_surface_reconstruction/assets/22293969/087d75b1-03be-47bd-8363-5c123351d630" width="640">

*reconstructed surface visualization*

## Dependencies
* [CGAL](https://www.cgal.org/) - Computational Geometry Algorithms Library
* [Pangolin](https://github.com/stevenlovegrove/Pangolin) - Lightweight portable rapid development library for managing OpenGL display / interaction and abstracting video input. OpenGL is required to install Pangolin. Other version (at least 0.6 and above) of Pangolin should also work but I used version 0.8.

## Updates
* 01/08/2023 - Initial commit - running `./main <path_to_point_cloud> <alpha> <offset>` will output the reconstructed object in a `.off` file.
* 03/08/2023 - Successfully drawn the mesh using pangolin and set up the thread structure. Design improvement is needed.
* 04/08/2023 - Added menu, surface drawing, and other improvements.
