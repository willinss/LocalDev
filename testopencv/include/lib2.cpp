#include <Eigen/Dense>
#include "lib2.h"
#include "lib1.h"
#include <iostream>

using namespace std;
using namespace Eigen;

void MatrixMultiply(Matrix3d m,Matrix3d n)
{ 
    Matrix3d p = m * n;
    cout << "m*n = " << endl;
    PrintMatrix(p);
}


