#include <iostream>
#include <ctime>
using namespace std;

//eigen part
#include <Eigen/Core>
//稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>

#define MATRIX_SIZE 50

int main(int argc, char** argv)
{
    //2*3的float矩阵
    Eigen::Matrix<float,2,3>matrix_23;
    //3*1的列向量
    Eigen::Vector3d v_3d;
    //3*3矩阵
    Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero();
    //动态矩阵
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> matrix_dynamic;
    //简单的动态矩阵
    Eigen::MatrixXd matrix_x;

    //给矩阵输入数据
    matrix_23 << 1,2,3,4,5,6;
    //输出
    cout << matrix_23 << endl;

    for(int i=0; i<1; i++)
        for(int j=0; j<2; j++)
            cout<<matrix_23(i,j)<<endl;
    
    v_3d << 3,2,1;

    //矩阵相乘,矩阵和向量相乘时候需要转换形式，维度也不能搞错了
    Eigen::Matrix<double,2,1> result = matrix_23.cast<double>() * v_3d;
    cout << result << endl;

    //随机一个3*3矩阵，然后转换
    matrix_33 = Eigen::Matrix3d::Random();
    cout<<matrix_33<<endl<<endl;

    cout << matrix_33.transpose() << endl;
    cout << matrix_33.sum()<<endl;
    cout << matrix_33.trace()<<endl;
    cout << 10*matrix_33<<endl;
    cout << matrix_33.inverse()<<endl;
    cout << matrix_33.determinant()<<endl;

    //实数对角化
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver (matrix_33.transpose()*matrix_33);
    cout<<"Eigen values =" << eigen_solver.eigenvalues() << endl;
    cout<<"Eigen vectors =" << eigen_solver.eigenvectors() << endl;

    //解方程
    Eigen::Matrix< double, MATRIX_SIZE, MATRIX_SIZE > matrix_NN;
    matrix_NN = Eigen::MatrixXd::Random( MATRIX_SIZE, MATRIX_SIZE );
    Eigen::Matrix< double, MATRIX_SIZE, 1 > v_Nd;
    v_Nd = Eigen::MatrixXd::Random(MATRIX_SIZE,1);

    clock_t time_stt = clock();

    Eigen::Matrix<double,MATRIX_SIZE,1> x = matrix_NN.inverse()*v_Nd;
    cout<<"time use in normal invers is" << 1000*(clock()-time_stt)/(double)CLOCKS_PER_SEC<<"ms"<<endl;

    time_stt = clock();
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    cout<<"time use in Qr compsition is "<< 1000*(clock()-time_stt)/(double)CLOCKS_PER_SEC<<"ms"<<endl;

    return 0 ;
}
