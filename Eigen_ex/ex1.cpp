#include <iostream>
#include <Eigen/Dense> // Eigen의 주요 모듈
using namespace std;
using namespace Eigen;

int main() {
    // 2x2 행렬 생성 및 초기화
    Matrix2d mat; 
    mat(0, 0) = 1; mat(0, 1) = 2;
    mat(1, 0) = 3; mat(1, 1) = 4;

    // 벡터 생성
    Vector2d vec(5, 6); // 2D 벡터

    // 행렬 * 벡터 연산
    Vector2d result = mat * vec;

    // 출력
    cout << "행렬:\n" << mat << "\n";
    cout << "벡터:\n" << vec << "\n";
    cout << "(Matrix * Vector):\n" << result << "\n";

    return 0;
}
