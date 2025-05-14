#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <cassert>

using namespace std;
using namespace Eigen;

tuple<MatrixXd, MatrixXd, MatrixXd, MatrixXd> calc_splines(
    const MatrixXd &path,       // 경로 점 (x, y 좌표)
    const VectorXd &el_lengths, // 각 점 간의 거리
    double psi_s,               // 시작점의 헤딩 각도
    double psi_e,               // 끝점의 헤딩 각도
    bool use_dist_scaling = true // 거리 차이에 따른 스케일링 여부
) {
    // 경로 점 개수 및 스플라인 구간 수
    int num_points = path.rows();
    int no_splines = num_points - 1;

    // 거리 스케일링 계산
    VectorXd scaling;
    if (use_dist_scaling) {
        scaling = el_lengths.head(no_splines - 1).array() / el_lengths.tail(no_splines - 1).array();
    } else {
        scaling = VectorXd::Ones(no_splines - 1);
    }

    // 계수 행렬 M 및 결과 벡터 b_x, b_y 초기화
    MatrixXd M = MatrixXd::Zero(no_splines * 4, no_splines * 4);
    VectorXd b_x = VectorXd::Zero(no_splines * 4);
    VectorXd b_y = VectorXd::Zero(no_splines * 4);

    // 제약 조건 템플릿
    MatrixXd template_M(4, 8);
    template_M << 1, 0, 0, 0, 0, 0, 0, 0,
                  1, 1, 1, 1, 0, 0, 0, 0,
                  0, 1, 2, 3, 0, -1, 0, 0,
                  0, 0, 2, 6, 0, 0, -2, 0;

    // M 행렬과 b_x, b_y 벡터 채우기
    for (int i = 0; i < no_splines; ++i) {
        int j = i * 4;

        if (i < no_splines - 1) {
            M.block(j, j, 4, 8) = template_M;
            M(j + 2, j + 5) *= scaling[i];
            M(j + 3, j + 6) *= pow(scaling[i], 2);
        } else {
            M.block(j, j, 2, 4) << 1, 0, 0, 0,
                                   1, 1, 1, 1;
        }

        b_x.segment(j, 2) << path(i, 0), path(i + 1, 0);
        b_y.segment(j, 2) << path(i, 1), path(i + 1, 1);
    }

    // 경계 조건 추가
    M(no_splines * 4 - 2, 1) = 1; // 시작점 헤딩
    b_x(no_splines * 4 - 2) = cos(psi_s + M_PI / 2) * el_lengths(0);
    b_y(no_splines * 4 - 2) = sin(psi_s + M_PI / 2) * el_lengths(0);

    M(no_splines * 4 - 1, no_splines * 4 - 4) = 0;
    M(no_splines * 4 - 1, no_splines * 4 - 3) = 1;
    M(no_splines * 4 - 1, no_splines * 4 - 2) = 2;
    M(no_splines * 4 - 1, no_splines * 4 - 1) = 3;
    b_x(no_splines * 4 - 1) = cos(psi_e + M_PI / 2) * el_lengths(no_splines - 1);
    b_y(no_splines * 4 - 1) = sin(psi_e + M_PI / 2) * el_lengths(no_splines - 1);

    // 선형 방정식 풀이
    VectorXd x_les = M.fullPivLu().solve(b_x);
    VectorXd y_les = M.fullPivLu().solve(b_y);

    // 스플라인 계수 재구성
    MatrixXd coeffs_x(no_splines, 4);
    MatrixXd coeffs_y(no_splines, 4);
    for (int i = 0; i < no_splines; ++i) {
        coeffs_x.row(i) = x_les.segment(i * 4, 4).transpose();
        coeffs_y.row(i) = y_les.segment(i * 4, 4).transpose();
    }

    // 법선 벡터 계산 및 정규화
    MatrixXd normvec(no_splines, 2);
    normvec.col(0) = coeffs_y.col(1);
    normvec.col(1) = -coeffs_x.col(1);

    VectorXd norm_factors = 1.0 / normvec.rowwise().norm();
    MatrixXd normvec_normalized = normvec.array().colwise() * norm_factors.array();

    return make_tuple(coeffs_x, coeffs_y, M, normvec_normalized);
}

int main() {
    // 경로 데이터
    MatrixXd path(3, 2);
    path << 50.0, 10.0,
            10.0, 4.0,
             0.0, 0.0;

    // 시작점과 끝점의 헤딩 각도
    double psi_s = M_PI / 2.0;
    double psi_e = M_PI / 1.3;

    // 점 간 거리 계산
    VectorXd el_lengths(2);
    el_lengths(0) = sqrt(pow(50.0 - 10.0, 2) + pow(10.0 - 4.0, 2));
    el_lengths(1) = sqrt(pow(10.0 - 0.0, 2) + pow(4.0 - 0.0, 2));

    // 스플라인 계산
    auto [coeffs_x, coeffs_y, M, normvec_normalized] = calc_splines(path, el_lengths, psi_s, psi_e);

    // 결과 출력
    cout << "X Coefficients:\n" << coeffs_x << endl;
    cout << "Y Coefficients:\n" << coeffs_y << endl;
    cout << "M Matrix:\n" << M << endl;
    cout << "Normalized Normal Vectors:\n" << normvec_normalized << endl;

    return 0;
}
