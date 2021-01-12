#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

int main(){
    // given constants
    float angle = 45.0f;
    const float PI = 3.14159f;
    float radian = (angle / 180) * PI;
    // the point to be transformed
    Eigen::MatrixXf point(3, 1);
    point << 2, 1, 1;
    // rotation matrix
    Eigen::Matrix3f rMatrix;
    rMatrix <<  cos(radian),-sin(radian), 0,
                sin(radian), cos(radian), 0,
                0          ,0         , 1;
    // translation matrix
    Eigen::Matrix3f tMatrix;
    float tx = 1;
    float ty = 2;
    tMatrix <<  1, 0, tx,
                0, 1, ty,
                0, 0, 1;
    // rotation
    std::cout << rMatrix * point << std::endl;
    // rotation and translation
    std::cout << tMatrix * rMatrix * point << std::endl;
    /*
    // Basic Example of cpp
    std::cout << "Example of cpp \n";
    float a = 1.0, b = 2.0;
    std::cout << a << std::endl;
    std::cout << a/b << std::endl;
    std::cout << std::sqrt(b) << std::endl;
    std::cout << std::acos(-1) << std::endl;
    std::cout << std::sin(30.0/180.0*acos(-1)) << std::endl;

    // Example of vector
    std::cout << "Example of vector \n";
    // vector definition
    Eigen::Vector3f v(1.0f,2.0f,3.0f);
    Eigen::Vector3f w(1.0f,0.0f,0.0f);
    // vector output
    std::cout << "Example of vector output \n";
    std::cout << v << std::endl;
    // vector add
    std::cout << "Example of add \n";
    std::cout << v + w << std::endl;
    // vector scalar multiply
    std::cout << "Example of scalar multiply \n";
    std::cout << v * 3.0f << std::endl;
    std::cout << 2.0f * v << std::endl;

    // Example of matrix
    std::cout << "Example of matrix \n";
    // matrix definition
    Eigen::Matrix3f i,j;
    i << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
    j << 2.0, 3.0, 1.0, 4.0, 6.0, 5.0, 9.0, 7.0, 8.0;
    // matrix output
    std::cout << "Example of 3d matrix output \n";
    std::cout << i << std::endl;
    // matrix add i + j
    std::cout << "Example of matrix addtion output \n";
    std::cout << i + j << std::endl;
    // matrix scalar multiply i * 2.0
    std::cout << "Example of matrix scaler output \n";
    std::cout << i * 2.0 << std::endl;
    // matrix multiply i * j
    std::cout << "Example of matrix mult output \n";
    std::cout << i * j << std::endl;
    // matrix multiply vector i * v
    std::cout << "Example of matrix mult vector output \n";
    std::cout << i * v << std::endl;
    */
    return 0;
}