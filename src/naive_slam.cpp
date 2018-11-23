#include "mckinnc_ekf/naive_slam.hpp"

namespace mckinnc_ekf {

NaiveSLAM::NaiveSLAM(const Covariance& modelCovariance,
    const Observation& observationVariance)
  : ekf_(modelCovariance, observationVariance)
{ }

void NaiveSLAM::run(const Control& control,
    const std::vector<Observation>& observations,
    double newLandmarkUncertaintyThreshold,
    double landmarkLikelihoodRadius)
{
  std::vector<Landmark> likelyLandmarks;
  for (const Landmark& landmark : landmarks_) {
    double distance = sqrt(landmark.x*landmark.x + landmark.y*landmark.y);
    if (distance < landmarkLikelihoodRadius) {
      likelyLandmarks.push_back(landmark);
    }
  }
  std::vector<Landmark> newLandmarks = ekf_.run(control, observations,
      likelyLandmarks, newLandmarkUncertaintyThreshold);
  for (const Landmark& landmark : newLandmarks) {
    landmarks_.push_back(landmark);
  }
}

} // namespace
