#include <Eigen/Dense>

int main() {
  // Create a matrix A of size 4 x 4
  Eigen::Matrix4d A;
  A << 1, 2, 3, 4,
       5, 6, 7, 8,
       9, 10, 11, 12,
       13, 14, 15, 16;

  // Set the submatrix A(1:2, 1:2) to zero
  A.block(1, 1, 2, 2).setZero();

  // Print the matrix
  std::cout << A << std::endl;

  return 0;
}