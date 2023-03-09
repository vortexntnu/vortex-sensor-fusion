// Dependencies
#include <Eigen/Dense>
#include <math.h>
#include <vector>
#include <iostream>

// Declaring namespaces
using namespace std;
using namespace Eigen;


typedef vector<tuple<int,int>> line_list;


// Return line between two points
line_list lineBetweenPoints(int r0,int c0,int r1,int c1);

// Return line between two points reversed
void lineBetweenPoints2(int r0,int c0,int r1,int c1,vector<int>& x,vector<int>& y);

// Rotates a point around a center
Vector2i rotatePointAroundCenter(Vector2f point, Vector2f center, float yaw);

// Takes quaternions as input and givs the corresponding euler angels as output
Vector3f quaternion_to_euler(float x, float y, float z, float w);













