//ex2-linear_equation.cpp
//선형 방정식 Ax = b를 푸는 예제
// 3x3 행렬과 3D 벡터를 사용
#include <iostream>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;

int main() {
    // 3x3 행렬과 3D 벡터 생성
    Matrix3d A;
    A << 2, -1, 0,
         -1, 2, -1,
         0, -1, 2;

    Vector3d b(1, 0, 1); 

    // 선형 방정식 Ax = b 풀기
    Vector3d x = A.colPivHouseholderQr().solve(b);

    // 출력
    cout << "행렬 A:\n" << A << "\n";
    cout << "벡터 b:\n" << b << "\n";
    cout << "Result x:\n" << x << "\n";

    return 0;
}