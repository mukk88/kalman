#include <iostream>
#include <Eigen/Dense>
using Eigen::MatrixXd;

int main(){
  MatrixXd m(2,2);
  MatrixXd m2(2,2);
  m << 1,2,3,4;
  MatrixXd m3;
  m3 = m.inverse();
  m2 << 3,4,5,6;
  MatrixXd m4 = Eigen::Matrix<double, 6, 6>::Identity();

  std::cout << m << std::endl << m3 << std::endl;
}