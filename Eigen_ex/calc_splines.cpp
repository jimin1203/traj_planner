#include <Eigen/Dense>
#include <iostream>
#include <cmath>

using namespace std;

std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> calc_splines(
    const Eigen::MatrixXd& path,
    const Eigen::VectorXd& el_lengths = Eigen::VectorXd(),
    double psi_s = NAN,
    double psi_e = NAN,
    bool use_dist_scaling = true) {
    
    bool closed = false;
    if ((path.row(0) - path.row(path.rows() - 1)).norm() < 1e-6 && std::isnan(psi_s)) {
        closed = true;
    }

    if (!closed && (std::isnan(psi_s) || std::isnan(psi_e))) {
        throw std::runtime_error("Headings must be provided for unclosed spline calculation!");
    }

    if (el_lengths.size() > 0 && path.rows() != el_lengths.size() + 1) {
        throw std::runtime_error("el_lengths input must be one element smaller than path input!");
    }

    Eigen::VectorXd el_lengths_copy;
    if (use_dist_scaling && el_lengths.size() == 0) {
        el_lengths_copy = (path.bottomRows(path.rows() - 1) - path.topRows(path.rows() - 1)).rowwise().norm();
    } else if (el_lengths.size() > 0) {
        el_lengths_copy = el_lengths;
    }

    if (use_dist_scaling && closed) {
        el_lengths_copy.conservativeResize(el_lengths_copy.size() + 1);
        el_lengths_copy(el_lengths_copy.size() - 1) = el_lengths_copy(0);
    }

    int no_splines = path.rows() - 1;

    Eigen::VectorXd scaling = Eigen::VectorXd::Ones(no_splines - 1);
    if (use_dist_scaling) {
        scaling = el_lengths_copy.head(no_splines - 1).array() / el_lengths_copy.tail(no_splines - 1).array();
    }

    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(no_splines * 4, no_splines * 4);
    Eigen::VectorXd b_x = Eigen::VectorXd::Zero(no_splines * 4);
    Eigen::VectorXd b_y = Eigen::VectorXd::Zero(no_splines * 4);

    Eigen::MatrixXd template_M(4, 8);
    template_M << 1, 0, 0, 0, 0, 0, 0, 0,
                  1, 1, 1, 1, 0, 0, 0, 0,
                  0, 1, 2, 3, 0, -1, 0, 0,
                  0, 0, 2, 6, 0, 0, -2, 0;

    for (int i = 0; i < no_splines; ++i) {
        int j = i * 4;

        if (i < no_splines - 1) {
            M.block(j, j, 4, 8) = template_M;
            M(j + 2, j + 5) *= scaling(i);
            M(j + 3, j + 6) *= std::pow(scaling(i), 2);
        } else {
            M.block(j, j, 2, 4) << 1, 0, 0, 0,
                                   1, 1, 1, 1;
        }

        b_x.segment(j, 2) << path(i, 0), path(i + 1, 0);
        b_y.segment(j, 2) << path(i, 1), path(i + 1, 1);
    }

    if (!closed) {
        M(no_splines * 4 - 2, 1) = 1;
        double el_length_s = el_lengths.size() == 0 ? 1.0 : el_lengths(0);
        b_x(no_splines * 4 - 2) = std::cos(psi_s + M_PI / 2) * el_length_s;
        b_y(no_splines * 4 - 2) = std::sin(psi_s + M_PI / 2) * el_length_s;

        M(no_splines * 4 - 1, no_splines * 4 - 4) = 0;
        M(no_splines * 4 - 1, no_splines * 4 - 3) = 1;
        M(no_splines * 4 - 1, no_splines * 4 - 2) = 2;
        M(no_splines * 4 - 1, no_splines * 4 - 1) = 3;
        double el_length_e = el_lengths.size() == 0 ? 1.0 : el_lengths(el_lengths.size() - 1);
        b_x(no_splines * 4 - 1) = std::cos(psi_e + M_PI / 2) * el_length_e;
        b_y(no_splines * 4 - 1) = std::sin(psi_e + M_PI / 2) * el_length_e;
    } else {
        M(no_splines * 4 - 2, 1) = scaling(no_splines - 2);
        M(no_splines * 4 - 2, no_splines * 4 - 3) = -1;
        M(no_splines * 4 - 2, no_splines * 4 - 2) = -2;
        M(no_splines * 4 - 2, no_splines * 4 - 1) = -3;

        M(no_splines * 4 - 1, 2) = 2 * std::pow(scaling(no_splines - 2), 2);
        M(no_splines * 4 - 1, no_splines * 4 - 2) = -2;
        M(no_splines * 4 - 1, no_splines * 4 - 1) = -6;
    }

    Eigen::VectorXd x_les = M.colPivHouseholderQr().solve(b_x);
    Eigen::VectorXd y_les = M.colPivHouseholderQr().solve(b_y);
    cout << b_x << endl;
    Eigen::MatrixXd coeffs_x = Eigen::Map<Eigen::MatrixXd>(x_les.data(), 4, no_splines).transpose();
    Eigen::MatrixXd coeffs_y = Eigen::Map<Eigen::MatrixXd>(y_les.data(), 4, no_splines).transpose();

    Eigen::MatrixXd normvec(no_splines, 2);
    normvec.col(0) = coeffs_y.col(1);
    normvec.col(1) = -coeffs_x.col(1);

    Eigen::VectorXd norm_factors = 1.0 / normvec.rowwise().norm().array();
    Eigen::MatrixXd normvec_normalized = normvec.array().rowwise() * norm_factors.transpose().array();

    return std::make_tuple(coeffs_x, coeffs_y, M, normvec_normalized);
}

int main() {
    Eigen::MatrixXd path(3, 2);
    path << 50.0, 10.0,
            10.0, 4.0,
            0.0, 0.0;

    double psi_s = 0.0; // 시작점의 헤딩 각도
    double psi_e = 0.0; // 끝점의 헤딩 각도

    auto [coeffs_x, coeffs_y, M, normvec_normalized] = calc_splines(path, Eigen::VectorXd(), psi_s, psi_e);

    std::cout << "Coefficients X:\n" << coeffs_x << "\n";
    std::cout << "Coefficients Y:\n" << coeffs_y << "\n";
    std::cout << "Matrix M:\n" << M << "\n";
    // std::cout << "Norm Vectors:\n" << normvec_normalized << "\n";

    return 0;
}
