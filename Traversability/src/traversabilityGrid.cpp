//
// Created by d.vivet on 19/04/23.
//
#include "../include/traversabilityGrid.h"
#include <Eigen/Eigenvalues>
#include <mutex>
#include <numeric>

double CalcMedian(std::vector<double> scores) {
    size_t size = scores.size();

    if (size == 0) {
        return 0; // Undefined, really.
    } else {
        sort(scores.begin(), scores.end());
        if (size % 2 == 0) {
            return (scores[size / 2 - 1] + scores[size / 2]) / 2;
        } else {
            return scores[size / 2];
        }
    }
}

double CalcMean(std::vector<double> scores) {
    double sum = std::accumulate(std::begin(scores), std::end(scores), 0.0);
    return sum / scores.size();
}

void traversabilityGrid::insertPoint(Eigen::Vector3d &p3d) {

    // Are we at maximal subdivision lvl?
    Eigen::Vector2i ind = meter2ind(Eigen::Vector2d(p3d.x(), p3d.y()));

    if (ind.x() >= size_x_ || ind.y() >= size_y_ || ind.x() < 0 || ind.y() < 0)
        return;
    _grid.at(ind.x()).at(ind.y()).insert(p3d);
}

void traversabilityGrid::computeHaz(Eigen::Vector2i ind) {

    // Init haz
    Eigen::VectorXd haz(6);

    double delta  = _security_distance / _resolution;
    int delta_ind = std::min(1.0, std::ceil(delta));

    // Point goodness cannot be calculated (border of the area)
    if (ind.x() < delta_ind || ind.x() > (size_x_ - delta_ind) || ind.y() < delta_ind ||
        ind.y() > (size_y_ - delta_ind)) {
        return;
    }

    // Init params
    uint border_hazard = 0;
    double zM = -100., zm = 100.;

    uint nb_min = 0.25 * (2 * delta_ind + 1) * (2 * delta_ind + 1);

    // Get all points to be considered for the current cell and get min/max altitude
    std::vector<Eigen::Vector3d> P3Ds;
    Eigen::Vector3d bary = Eigen::Vector3d::Zero(); 
    std::vector<double> traceCOVs;
    for (int i = std::max(0, int(ind.x() - delta_ind)); i < std::min(int(ind.x() + delta_ind), size_x_); i++) {
        for (int j = std::max(0, int(ind.y() - delta_ind)); j < std::min(int(ind.y() + delta_ind), size_y_); j++) {
            if (_grid.at(i).at(j).N < nb_min) {
                border_hazard++;
            } else {
                Eigen::Vector3d center =
                    Eigen::Vector3d(_grid.at(i).at(j).sx, _grid.at(i).at(j).sy, _grid.at(i).at(j).sz) /
                    _grid.at(i).at(j).N;
                bary += center;

                double traceCov = _grid.at(i).at(j).sx2 + _grid.at(i).at(j).sx / _grid.at(i).at(j).N +
                                  _grid.at(i).at(j).sy2 + _grid.at(i).at(j).sy / _grid.at(i).at(j).N +
                                  _grid.at(i).at(j).sz2 + _grid.at(i).at(j).sz / _grid.at(i).at(j).N;

                P3Ds.push_back(center);
                traceCOVs.push_back(traceCov);
                zM = std::max(zM, _grid.at(i).at(j).z_max);
                zm = std::min(zm, _grid.at(i).at(j).z_min);
            }
        }
    }

    if (border_hazard > 0) {
        return;
    }

    // Check if delta z is bigger than the robot ground clearance (how to deals with grass?)
    double step_hazard = ((zM - zm) / _ground_clearance);
    if (step_hazard > 1)
        step_hazard = 1;

    // Process surface normal

    // Method n°1 : classical LMS with cells barycenters
    // AX=B
    // Eigen::Vector3f X = A.colPivHouseholderQr().solve(B);
    // Eigen::MatrixXd P3Ds_M(3, P3Ds.size());
    // for (uint i = 0; i < P3Ds.size(); ++i)
    //     P3Ds_M.block<3, 1>(0, i) = P3Ds.at(i);
    // Eigen::VectorXd ONE      = Eigen::VectorXd::Ones(P3Ds.size());
    // Eigen::Vector3d planeLMS = P3Ds_M.transpose().colPivHouseholderQr().solve(ONE);
    // planeLMS.normalize();

    // double roughness = (Eigen::VectorXd::Ones(P3Ds.size()) - P3Ds_M.transpose() * planeLMS).norm() / P3Ds.size();

    // Method n°2 : using covariance ponderation (trace of P3D covariance as weight)
    // Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
    // for (uint i = 0; i < P3Ds.size(); ++i) {
    //     A(0, 0) += traceCOVs.at(i) * P3Ds.at(i).x() * P3Ds.at(i).x();
    //     A(0, 1) += traceCOVs.at(i) * P3Ds.at(i).x() * P3Ds.at(i).y();
    //     A(0, 2) += traceCOVs.at(i) * P3Ds.at(i).x() * P3Ds.at(i).z();
    //     A(1, 0) += traceCOVs.at(i) * P3Ds.at(i).y() * P3Ds.at(i).x();
    //     A(1, 1) += traceCOVs.at(i) * P3Ds.at(i).y() * P3Ds.at(i).y();
    //     A(1, 2) += traceCOVs.at(i) * P3Ds.at(i).y() * P3Ds.at(i).z();
    //     A(2, 0) += traceCOVs.at(i) * P3Ds.at(i).z() * P3Ds.at(i).x();
    //     A(2, 1) += traceCOVs.at(i) * P3Ds.at(i).z() * P3Ds.at(i).y();
    //     A(2, 2) += traceCOVs.at(i) * P3Ds.at(i).z() * P3Ds.at(i).z();
    // }
    // Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(A);
    // Eigen::Vector3d planeLMS = es.eigenvectors().col(0).normalized();

    // if (planeLMS.z() < 0)
    //     planeLMS = -planeLMS;

    // Method no 3 : compute covariance
    bary /= (double)P3Ds.size();
    Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
    for (uint i = 0; i < P3Ds.size(); ++i) {
        Eigen::Vector3d dp3d = P3Ds.at(i) - bary;
        P3Ds.at(i)           = dp3d;
        A += dp3d * dp3d.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(A);
    Eigen::Vector3d planeLMS = es.eigenvectors().col(0).normalized();

    if (planeLMS.z() < 0)
        planeLMS = -planeLMS;

    double roughness = 0.0;
    for (uint i = 0; i < P3Ds.size(); ++i) {
        roughness += double(P3Ds.at(i).transpose() * planeLMS) * double(P3Ds.at(i).transpose() * planeLMS);
    }
    roughness = std::sqrt(roughness) / P3Ds.size();

    // Check if ground roughness is bigger than the robot ground clearance (how to deals with grass?)
    double roughness_hazard = 3. * roughness / _ground_clearance;
    if (roughness_hazard > 1)
        roughness_hazard = 1;

    // Check if ground slope is bigger than the robot admissive max slope
    double pitch        = std::abs(std::acos(planeLMS.dot(Eigen::Vector3d(0., 0., 1.))));
    double pitch_hazard = pitch / _max_slope;

    haz(0) = std::max(std::max(step_hazard, roughness_hazard), pitch_hazard);
    haz(1) = step_hazard;
    haz(2) = roughness_hazard;
    haz(3) = pitch_hazard;
    haz(4) = _grid.at(ind.x()).at(ind.y()).sz / _grid.at(ind.x()).at(ind.y()).N;

    _vec_cell_haz.at(ind.x()).at(ind.y()).push_back(haz);
}

void traversabilityGrid::computeHazGrid() {

    // Compute hazard for each cell of the grid
    for (int i = 0; i < size_x_; ++i) {
        for (int j = 0; j < size_y_; ++j) {
            computeHaz(Eigen::Vector2i(i, j));
        }
    }
}

void traversabilityGrid::resetHaz(Eigen::Vector2i ind) {

    _grid.at(ind.x()).at(ind.y()).reset();
    _vec_cell_haz.at(ind.x()).at(ind.y()).clear();
}

Eigen::VectorXd const traversabilityGrid::getHaz(Eigen::Vector2i ind) {
    std::mutex mtx;
    std::lock_guard lock(mtx);

    std::vector<Eigen::VectorXd> haz_vec = _vec_cell_haz.at(ind.x()).at(ind.y());

    if (haz_vec.empty()) {
        Eigen::VectorXd X(5);
        X.setZero();
        return X;
    } else {

        // return the median of the hazard
        Eigen::VectorXd X(5);
        X.setZero();

        // Compute the median value for each hazard
        for (int i = 0; i < 5; i++) {
            std::vector<double> val_vec;

            for (auto haz : haz_vec)
                val_vec.push_back(haz(i));

            double val = CalcMedian(val_vec);
            X(i)       = val;
        }

        // Recompute the global hazard value
        X(0) = std::max(std::max(X(1), X(2)), X(3));

        return X;
    }
}

Eigen::VectorXd const traversabilityGrid::getHazMeters(Eigen::Vector2d ind_m) { return getHaz(meter2ind(ind_m)); }


void traversabilityGrid::fuseWithGrid(const std::shared_ptr<traversabilityGrid> grid) {

    for (int i = 0; i < size_x_; ++i) {
        for (int j = 0; j < size_y_; ++j) {
            Eigen::Vector2i ind(i, j);
            Eigen::VectorXd haz = grid->getHaz(ind);

            if (!haz.isZero())
                _vec_cell_haz.at(i).at(j).push_back(haz);
        }
    }
}