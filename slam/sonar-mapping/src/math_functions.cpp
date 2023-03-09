#include "math_functions.hpp"


// Return line between two points
line_list lineBetweenPoints(int r0,int c0,int r1,int c1) {
    line_list allPointsInLine;
    if (abs(c1-c0) < abs(r1-r0)) {
        
        return lineBetweenPoints(c0,r0,c1,r1);
    }
    if (c0 > c1) {

        return lineBetweenPoints(r1,c1,r0,c0);

    } 
    vector<int> x(c1-c0),y(c1-c0);
    float a, b;
    a = (float)(r1-r0)/(float)(c1-c0);
    b = ((float)(r0*c1 - r1*c0))/(float)(c1-c0);
    iota(x.begin(),x.end(),c0);   

    for (int i = 0; i < x.size(); i++) {
        
        y[i] = x[i]*a + b;
        allPointsInLine.push_back(tuple<int,int>(x[i],y[i]));
    }
    
    return allPointsInLine;

}

void lineBetweenPoints2(int r0,int c0,int r1,int c1,vector<int>& x,vector<int>& y) {
    
    
    if (abs(c1-c0) < abs(r1-r0)) {    
        lineBetweenPoints2(c0,r0,c1,r1,x,y);
        vector<int> tmp = y;
        y = x;
        x = tmp;
    }
    if (c0 > c1) {
        return lineBetweenPoints2(r1,c1,r0,c0,x,y);

    } 
    x.resize(c1-c0);
    y.resize(c1-c0);
    float a, b;
    a = (float)(r1-r0)/(float)(c1-c0);
    b = ((float)(r0*c1 - r1*c0))/(float)(c1-c0);
    iota(x.begin(),x.end(),c0);   
    for (int i = 0; i < x.size(); i++) {
        y[i] = x[i]*a + b;
    }
}

// Rotates a point around a center
Vector2i rotatePointAroundCenter(Vector2f point, Vector2f center, float yaw) {
    float c,s;
    Vector2i rotated_point;
    Matrix2f rotMatrix;
    s = sin(yaw);
    c = cos(yaw);
    rotMatrix.row(0) << c, -s;
    rotMatrix.row(1) << s, c;
    rotated_point = (rotMatrix*(point-center) + center).cast <int> ();
    return rotated_point;
 }


// Takes quaternions as input and givs the corresponding euler angels as output
Vector3f quaternion_to_euler(float x, float y, float z, float w) {
    float t0,t1,t2,t3,t4,roll,pitch,yaw;
    t0 = 2.0 * (w * x + y * z);
    t1 = 1.0 - 2.0 * (x * x + y * y);
    roll = atan2(t0,t1);
    t2 = 2.0 * (w * y - z * x);
    if (t2 > 1.0) {
        t2 = 1.0;
    } 
    else if (t2 < -1.0){
        t2 = -1.0;
    }
    pitch = asin(t2);
    t3 = 2.0 * (w * z + x * y);
    t4 = 1.0 - 2.0 * (y * y + z * z);
    yaw = atan2(t3,t4);
    Vector3f euler;
    euler << (Vector3f() << roll,pitch,yaw).finished();
    return euler; 
}
