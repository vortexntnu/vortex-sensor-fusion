// __device__ bool isPointInTrapezoid(float px, float py, float *trapezoid) {
//     // Implement the point-in-trapezoid logic here, similar to the point-in-polygon test in the provided C++ code
//     // The 'trapezoid' parameter is an array of coordinates defining the trapezoid
//     // This function returns true if the point (px, py) is inside the trapezoid
// }

// __global__ void findPointsBehindWallsKernel(float3 *points, int numPoints, float *wallPoses, int numWalls, bool *output) {
//     int index = blockIdx.x * blockDim.x + threadIdx.x;
//     if (index < numPoints) {
//         float3 point = points[index];
//         bool isBehind = false;

//         // Each wall is represented by two points in 'wallPoses'. Thus 'numWalls' should be half the length of 'wallPoses'
//         for (int i = 0; i < numWalls; i += 2) {
//             float trapezoid[8]; // Store the trapezoid coordinates here
//             // Calculate trapezoid based on wallPoses[i] and wallPoses[i+1]
//             // You need to implement the trapezoid extension similar to your C++ 'createPolygon' function
            
//             if (isPointInTrapezoid(point.x, point.y, trapezoid)) {
//                 isBehind = true;
//                 break;
//             }
//         }
//         output[index] = isBehind;
//     }
// }

// // Host code to allocate memory, copy data, and launch the kernel
// // You need to set up `points`, `wallPoses`, and allocate memory for `output`
// // Then, you copy these data to device memory, call the kernel, and copy `output` back to host to check which points to remove
