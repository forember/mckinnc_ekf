#pragma once

#include "mckinnc_ekf/common.hpp"

#include <vector>
#include <Eigen/Dense>

/*! \file
 *  \brief EKF localization algorithm with support for na&iuml;ve SLAM.
 */

namespace mckinnc_ekf {

//! 3x3 matrix of doubles representing covariance of a 3-vector.
typedef Eigen::Matrix3d Covariance;

class EKF {
  public:
    EKF(const Covariance& modelCovariance,
        const Observation& observationVariance);

    //! Run a single update of the EKF localization.
    /*!
     *  \param control
     *  Linear and angular robot controls for the period since the last run,
     *  and the elapsed time since the last run.
     *  \param observations
     *  Observations of the landmarks.
     *  \param likelyLandmarks
     *  Landmarks that the observations are likely to correspond to.
     *  \param newLandmarkUncertaintyThreshold
     *  Uncertainty above which an observation will be considered a new
     *  landmark, and will not be used for correction. Set to `INFINITY` to
     *  always find a landmark for an observation.
     *  \return
     *  New landmarks from unmatched observations.
     */
    std::vector<Landmark> run(const Control& control,
        const std::vector<Observation>& observations,
        const std::vector<Landmark>& likelyLandmarks,
        double newLandmarkUncertaintyThreshold);

    //! Current mean of the pose estimation.
    const Pose& mean() { return mean_; }
    //! Current covariance of the pose estimation.
    const Covariance& covariance() { return covariance_; }

  protected:
    Pose mean_;
    Covariance covariance_;
    Covariance modelCovariance_;
    Eigen::DiagonalMatrix<double, 3> observationCovariance_;
};

} // namespace
