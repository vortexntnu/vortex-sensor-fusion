// #include <pcl_detector/pcl_processor.hpp>
// // #include <cuda_runtime.h>
// // #include <vector_types.h>
// // #include <helper_cuda.h>
// #include <thrust/host_vector.h>
// #include <thrust/device_vector.h>
// // #include <pcl_detector/geometry_processor.hpp>
// // #include </__w/1/s/cuda/filters/include/pcl/cuda/filters/filter.h>
// // #include </__w/1/s/cuda/sample_consensus/include/pcl/cuda/sample_consensus/msac.h>


// namespace pcl_detector {

// PclProcessor::PclProcessor(float voxel_leaf_size, float model_thresh, int model_iterations,
//                  float prev_line_thresh, float project_thresh, float wall_min_dist, int wall_min_points)
//     : voxel_leaf_size_(voxel_leaf_size), model_thresh_(model_thresh),
//       model_iterations_(model_iterations), prev_line_thresh_(prev_line_thresh),
//       project_thresh_(project_thresh), wall_min_dist_(wall_min_dist),
//       wall_min_points_(wall_min_points) {}




// void pcl_detector::PclProcessor::getPointsBehindWalls(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::vector<pcl::PointXYZ>& wall_poses, std::vector<int>& indices_to_remove){
//     float length = 150.0;
//     std::vector<int> indices;
//     std::vector<float2> h_points(cloud->size());
//     std::vector<float2> h_polygons(wall_poses.size()*2);
//     for(size_t i = 0; i < wall_poses.size(); i+=2){
//         float p1.x = wall_poses[i].x;
//         float p1.y = wall_poses[i].y;
//         float p2.x = wall_poses[i+1].x;
//         float p2.y = wall_poses[i+1].y;
    
//         float p1_magnitude = sqrt(p1.x * p1.x + p1.y * p1.y);
//         float p2_magnitude = sqrt(p2.x * p2.x + p2.y * p2.y);

//         float p1_ext.x = (p1.x / p1_magnitude) * length;
//         float p1_ext.y = (p1.y / p1_magnitude) * length;
//         float p2_ext.x = (p2.x / p2_magnitude) * length;
//         float p2_ext.y = (p2.y / p2_magnitude) * length;
        
//         h_polygons[i*2] = make_float2(p1.x, p1.y);
//         h_polygons[i*2+1] = make_float2(p2.x, p2.y);
//         h_polygons[i*2+2] = make_float2(p2_ext.x, p2_ext.y);
//         h_polygons[i*2+3] = make_float2(p1_ext.x, p1_ext.y);
//     }
//     thrust::device_vector<float2> d_polygons = h_polygons;
//     cudaMalloc()
    

    


// }


// bool pcl_detector::PclProcessor::isXYPointIn2DXYPolygon (const pcl::PointXYZ& point, const pcl::PointCloud<pcl::PointXYZ> &polygon)
// {
//   bool in_poly = false;
//   double x1, x2, y1, y2;

//   const auto nr_poly_points = polygon.size ();
//   // start with the last point to make the check last point<->first point the first one
//   double xold = polygon[nr_poly_points - 1].x;
//   double yold = polygon[nr_poly_points - 1].y;
//   for (std::size_t i = 0; i < nr_poly_points; i++)
//   {
//     double xnew = polygon[i].x;
//     double ynew = polygon[i].y;
//     if (xnew > xold)
//     {
//       x1 = xold;
//       x2 = xnew;
//       y1 = yold;
//       y2 = ynew;
//     }
//     else
//     {
//       x1 = xnew;
//       x2 = xold;
//       y1 = ynew;
//       y2 = yold;
//     }

//     if ( (xnew < point.x) == (point.x <= xold) && (point.y - y1) * (x2 - x1) < (y2 - y1) * (point.x - x1) )
//     {
//       in_poly = !in_poly;
//     }
//     xold = xnew;
//     yold = ynew;
//   }

//   return (in_poly);
// }

// __global__ void addArrays(const float* a, const float* b, float* c, int N) {
//     int index = threadIdx.x + blockIdx.x * blockDim.x;
//     if (index < N) {
//         c[index] = a[index] + b[index];
//     }
// }

// // Host code
// int main() {
//     int N = 1024;  // Example size
//     size_t size = N * sizeof(float);

//     float *h_a, *h_b, *h_c;  // Host arrays
//     float *d_a, *d_b, *d_c;  // Device arrays

//     // Allocate host memory
//     h_a = new float[N];
//     h_b = new float[N];
//     h_c = new float[N];

//     // Allocate device memory
//     cudaMalloc(&d_a, size);
//     cudaMalloc(&d_b, size);
//     cudaMalloc(&d_c, size);

//     // Initialize and copy data from host to device
//     // Initialize h_a and h_b here...
//     cudaMemcpy(d_a, h_a, size, cudaMemcpyHostToDevice);
//     cudaMemcpy(d_b, h_b, size, cudaMemcpyHostToDevice);

//     // Kernel launch
//     int threadsPerBlock = 256;
//     int blocksPerGrid = (N + threadsPerBlock - 1) / threadsPerBlock;
//     addArrays<<<blocksPerGrid, threadsPerBlock>>>(d_a, d_b, d_c, N);

//     // Copy result back to host
//     cudaMemcpy(h_c, d_c, size, cudaMemcpyDeviceToHost);

//     // Use h_c for further processing or verification

//     // Cleanup
//     cudaFree(d_a);
//     cudaFree(d_b);
//     cudaFree(d_c);
//     delete[] h_a;
//     delete[] h_b;
//     delete[] h_c;
// }



// }

