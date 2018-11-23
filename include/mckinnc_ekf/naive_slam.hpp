#pragma once

#include "mckinnc_ekf/common.hpp"
#include "mckinnc_ekf/ekf.hpp"

#include <vector>

namespace mckinnc_ekf {

class NaiveSLAM {
  public:
    NaiveSLAM(const Covariance& modelCovariance,
        const Observation& observationVariance,
        double newLandmarkUncertaintyThreshold);

    void run(const Control& control,
        const std::vector<Observation>& observations,
        double newLandmarkUncertaintyThreshold,
        double landmarkLikelihoodRadius);

    //! \see EKF::mean()
    const Pose& mean() { return ekf_.mean(); }
    //! \see EKF::covariance()
    const Covariance& covariance() { return ekf_.covariance(); }
    //! Current set of map landmarks.
    const std::vector<Landmark>& landmarks() { return landmarks_; }

  protected:
    EKF ekf_;
    std::vector<Landmark> landmarks_;
};

} // namespace
