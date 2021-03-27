#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

int main(int argc, char** argv)
{
    //定义一个旋转矩阵，Matrix3d  旋转向量AngleAxisd
    //定义一个旋转向量，欧拉角 Vector3d
    //设置显示精度为3位
    //将旋转向量赋值给动态矩阵
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
    Eigen::AngleAxisd rotation_vector (M_PI/4, Eigen::Vector3d(0,0,1));
    cout.precision(3);
    cout<<"rotation matrix =\n"<<rotation_vector.matrix()<<endl; //输出1
    rotation_matrix = rotation_vector.toRotationMatrix();

    Eigen::Vector3d v(1,0,0);
    Eigen::Vector3d v_rotated = rotation_vector * v;
    cout<<"(1,0,0) after rotation ="<< v_rotated.transpose()<<endl;//输出2

    //欧拉角
    Eigen::Vector3d euler_angels = rotation_matrix.eulerAngles(2,1,0);
    cout<<"yaw pitch roll = "<< euler_angels.transpose()<<endl;//输出3

    //欧式变换矩阵
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.rotate( rotation_vector );
    T.pretranslate(Eigen::Vector3d(1,3,7));
    cout<<"Transform matrix = \n"<<T.matrix()<<endl;//输出4

    Eigen::Vector3d v_transformed = T*v;
    cout<<"v transformed = "<<v_transformed.transpose()<<endl;//输出5

    //四元素
    Eigen::Quaterniond q = Eigen::Quaterniond(rotation_vector);
    cout<<"quaternion = \n"<<q.coeffs()<<endl;//输出6
    
    q = Eigen::Quaterniond(rotation_matrix);
    cout<<"quaternion = \n"<<q.coeffs()<<endl;//输出7
    v_rotated = q*v;
    cout<<"(1,0,0) after rotation = "<<v_rotated.transpose()<<endl;//输出8

    return 0;
}