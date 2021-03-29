#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sophus/so3.hpp"
#include "sophus/se3.hpp"

int main(int argc, char** argv)
{
    //沿Z轴旋转90度的旋转矩阵
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0,0,1)).toRotationMatrix();
    Sophus::SO3d SO3_R(R);                      //直接从矩阵构造
    //Sophus::SO3d SO3_v(0, 0, M_PI/2);
    Eigen::Quaterniond q(R);                    //转换成四元素
    Sophus::SO3d SO3_q(q);                      //从四元素构造

    cout<<"R = \n"<<R<<endl;
    cout<<"q(R) = \n"<<q.coeffs().transpose()<<endl;
    cout<<"SO(3) from matrix: \n"<<SO3_R.matrix()<<endl;
    //cout<<"SO(3) from vector: \n"<<SO3_v.log().transpose()<<endl;
    cout << "SO(3) from quaternion: \n" << SO3_q.matrix() << endl;      //两个结果都是相同的

    Eigen::Vector3d so3 = SO3_R.log();          //对数映射求李代数
    cout << "so3 = \n" << so3.transpose() << endl;
    cout << "so3 hat=\n" << Sophus::SO3d::hat(so3) << endl;     //hat 反对称矩阵变形
    cout <<"so3 hat vee = \n"<<Sophus::SO3d::vee( Sophus::SO3d::hat(so3)).transpose()<<endl;        //反对称到向量

    //增量扰动模型
    Eigen::Vector3d update_so3(1e-4,0,0);       //假设的更新量
    Sophus::SO3d SO3_update = Sophus::SO3d::exp(update_so3)*SO3_R;      //即书中所说的旋转矩阵的导数也称增量，y'=f(x)'*dx
    cout<<"SO3 updated = \n"<<SO3_update.matrix()<<endl;


    //对SE（3）的操作
    Eigen::Vector3d t(1,0,0);       //假设沿x轴平移1
    Sophus::SE3d SE3_Rt(R,t);       //从R t构造SE3
    Sophus::SE3d SE3_qt(q,t);       //从q t构造SE3
    cout<<"SE3 form R,t = "<<endl<<SE3_Rt.matrix()<<endl;
    cout<<"SE3 form q,t = "<<endl<<SE3_qt.matrix()<<endl;

    typedef Eigen::Matrix<double,6,1> Vector6d;
    Vector6d se3 = SE3_Rt.log();        //求出对数映射的李代数
    cout<<"se3 = \n"<<se3.transpose()<<endl;

    cout<<"se3 hat = " <<endl<<Sophus::SE3d::hat(se3)<<endl;
    cout<<"se3 hat vee = " <<endl<<Sophus::SE3d::vee(Sophus::SE3d::hat(se3)).transpose()<<endl;


    //增量扰动模型
    Vector6d update_se3;
    update_se3.setZero();
    update_se3(0,0) = 1e-4;
    Sophus::SE3<double> SE3_updated = Sophus::SE3d::exp(update_se3)*SE3_Rt;
    cout<<"SE3_updete = " <<endl<< SE3_updated.matrix()<<endl;
    
    return 0;
}