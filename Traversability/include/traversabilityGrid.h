//
// Created by d.vivet on 19/04/23.
//

#ifndef TRAVERSABILITY_TRAVERSABILITYGRID_H
#define TRAVERSABILITY_TRAVERSABILITYGRID_H

#include <Eigen/Dense>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <memory>

class NodeMetaData {

  public:
    NodeMetaData() {
        z_min = 100.;
        z_max = -100.;
        sx    = 0;
        sy    = 0;
        sz    = 0;
        sx2   = 0;
        sy2   = 0;
        sz2   = 0;
        sxy   = 0;
        sxz   = 0;
        syz   = 0;
        N     = 0;
    }

    void reset() {
        z_min = 100.;
        z_max = -100.;
        sx    = 0;
        sy    = 0;
        sz    = 0;
        sx2   = 0;
        sy2   = 0;
        sz2   = 0;
        sxy   = 0;
        sxz   = 0;
        syz   = 0;
        N     = 0;
    }

    void insert(Eigen::Vector3d &p3d) {
        N++;
        // deal with min max
        z_min = std::min(p3d.z(), z_min);
        z_max = std::max(p3d.z(), z_max);

        // update momentums
        sx += p3d.x();
        sy += p3d.y();
        sz += p3d.z();
        sx2 += p3d.x() * p3d.x();
        sy2 += p3d.y() * p3d.y();
        sz2 += p3d.z() * p3d.z();
        sxy += p3d.x() * p3d.y();
        sxz += p3d.x() * p3d.z();
        syz += p3d.y() * p3d.z();
    }

    unsigned int N = 0;
    double z_min   = 0.;
    double z_max   = 0.;
    double sx      = 0.;
    double sy      = 0.;
    double sz      = 0.;
    double sx2     = 0.;
    double sy2     = 0.;
    double sz2     = 0.;
    double sxy     = 0.;
    double sxz     = 0.;
    double syz     = 0.;
};

class traversabilityGrid {
  public:
    traversabilityGrid(double resolution,
                       Eigen::Vector2d ahalfside,
                       double max_slope,
                       double security_distance,
                       double ground_clearance)
        : _resolution(resolution), _halfside(ahalfside), _max_slope(max_slope), _security_distance(security_distance),
          _ground_clearance(ground_clearance) {

        // resize grid
        NodeMetaData default_value;
        double Nd_x = _halfside.x() / _resolution;
        double Nd_y = _halfside.y() / _resolution;
        size_x_     = 2. * (std::ceil(Nd_x)) + 1;
        size_y_     = 2. * (std::ceil(Nd_y)) + 1;
        _grid.resize(size_x_, std::vector<NodeMetaData>(size_y_, default_value));

        // resize hazard grid
        std::vector<Eigen::VectorXd> haz_vec;
        _vec_cell_haz.resize(size_x_, std::vector<std::vector<Eigen::VectorXd>>(size_y_, haz_vec));
    }

    void insertPoint(Eigen::Vector3d &p3d);

    void reset() {

        for (int i = 0; i < size_x_; ++i)
            for (int j = 0; j < size_y_; ++j)
                resetHaz(Eigen::Vector2i(i, j));
    }

    void computeHaz(Eigen::Vector2i ind);
    void computeHazGrid();

    Eigen::VectorXd const getHaz(Eigen::Vector2i ind);
    Eigen::VectorXd const getHazMeters(Eigen::Vector2d ind_m);
    void resetHaz(Eigen::Vector2i ind);

    uint getNbCells() { return size_x_ * size_y_; }

    Eigen::Vector2i meter2ind(Eigen::Vector2d meter) {
        Eigen::Vector2d idx = (meter + _halfside) / _resolution;
        return Eigen::Vector2i(std::floor(idx.x()), std::floor(idx.y()));
    }

    Eigen::Vector2d ind2meter(Eigen::Vector2d ind) { return (ind * _resolution - _halfside); }

    void fuseWithGrid(const std::shared_ptr<traversabilityGrid> grid);

  private:
    double _resolution;        /// cell size
    Eigen::Vector2d _center;   /// Center
    Eigen::Vector2d _halfside; /// Half-size of the cube
    double _max_slope;         /// Maximum slope traversable
    double _security_distance; /// Define the resolution of the grid
    double _ground_clearance;  /// What height can be crossed by the robot

    NodeMetaData _meta_data; /// Meta-data container

    std::vector<std::vector<std::vector<Eigen::VectorXd>>> _vec_cell_haz;

    int size_x_;
    int size_y_;

    std::vector<std::vector<NodeMetaData>> _grid;
};

#endif // TRAVERSABILITY_TRAVERSABILITYGRID_H