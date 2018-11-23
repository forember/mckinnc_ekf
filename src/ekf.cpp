#include "mckinnc_ekf/ekf.hpp"

namespace mckinnc_ekf {

// 3x3 matrix of doubles
typedef Covariance Jacobian;

EKF::EKF(const Covariance& modelCovariance,
    const Observation& observationVariance)
  : mean_({0.0, 0.0, 0.0}),
    covariance_(Covariance::Zero()),
    modelCovariance_(modelCovariance),
    observationCovariance_(observationVariance.range,
        observationVariance.angle, observationVariance.signature)
{ }

// ### STATIC FUNCTIONS AND STRUCTS FOR EKF RUN ###############################

//  Predict the robot motion (change in pose) since the last EKF run.
static Pose predictMotion(const Pose& mean, const Control& control)
{
  Pose motion;
  if (control.angular == 0.0) {
    motion.heading = 0.0;
    motion.x = control.linear * control.timeDelta * cos(mean.heading);
    motion.y = control.linear * control.timeDelta * sin(mean.heading);
  } else {
    double controlQuotient = control.linear / control.angular;
    motion.heading = control.angular * control.timeDelta;
    motion.x = controlQuotient
      * (sin(mean.heading + motion.heading) - sin(mean.heading));
    motion.y = controlQuotient
      * (cos(mean.heading) - cos(mean.heading + motion.heading));
  }
  return motion;
}

//  Add a pose representing an adjustment to a pose.
static Pose sumPose(const Pose& pose, const Pose& delta)
{
  Pose sum;
  sum.x = pose.x + delta.x;
  sum.y = pose.y + delta.y;
  sum.heading = reduceAngle(pose.heading + delta.heading);
  return sum;
}

//  Predict the new pose covariance after a robot motion.
static Covariance predictCovariance(const Covariance& covariance,
    const Pose& motion, const Covariance& modelCovariance)
{
  Jacobian jacobian;
  jacobian
    <<  1.0, 0.0, motion.y,
        0.0, 1.0, motion.x,
        0.0, 0.0, 1.0;
  return ((jacobian * covariance) * jacobian.transpose()) + modelCovariance;
}

//  Predicted observation with its Jacobian and the inverse of its
//  innovation covariance.
struct PredictedObservation {
  double range;
  double angle;
  double signature;
  Jacobian jacobian;
  Covariance inverseInnovation;
};

//  Predict the observation of a landmark.
static PredictedObservation predictObservation(const Landmark& landmark,
    const Pose& predictedMean, const Covariance& predictedCovariance,
    const Covariance& observationCovariance)
{
  double relativeX = landmark.x - predictedMean.x;
  double relativeY = landmark.y - predictedMean.y;
  double rangeSquared = (relativeX * relativeX) + (relativeY * relativeY);
  double range = sqrt(rangeSquared);

  PredictedObservation predicted;
  predicted.range = range;
  predicted.angle = reduceAngle(atan2(relativeY, relativeX)
      - predictedMean.heading);
  predicted.signature = landmark.signature;
  predicted.jacobian
    <<  range * relativeX, -range * relativeY, 0.0,
        relativeY, relativeX, -1.0,
        0.0, 0.0, 0.0;
  predicted.jacobian /= rangeSquared;
  predicted.inverseInnovation = (((predicted.jacobian * predictedCovariance)
        * predicted.jacobian.transpose()) + observationCovariance).inverse();
  return predicted;
}

/*
 *  Addend to the mean and covariance. May be invalid if its corresponding
 *  observation cannot be associated with a known landmark with sufficient
 *  certainty. Do not access any fields other than the `valid` flag on an
 *  invalid correction term.
 */
struct CorrectionTerm {
  bool valid;
  Eigen::Vector3d mean;
  Covariance covariance;
};

//  Calculate the correction term for an observation, if a landmark can be
//  associated with it with sufficient certainty.
static CorrectionTerm calculateCorrectionTerm(const Observation& observation,
    const std::vector<PredictedObservation>& predictedObservations,
    const Covariance& predictedCovariance,
    double newLandmarkUncertaintyThreshold)
{
  // Find the likeliest landmark
  size_t likeliestLandmarkIndex;
  Eigen::Vector3d likeliestLandmarkError;
  double likeliestLandmarkUncertainty = INFINITY;
  for (size_t j = 0; j < predictedObservations.size(); ++j) {
    const PredictedObservation& predicted = predictedObservations[j];
    Eigen::Vector3d error;
    error
      <<  observation.range - predicted.range,
          reduceAngle(observation.angle - predicted.angle),
          observation.signature - predicted.signature;
    double uncertainty = (error.transpose() * predicted.inverseInnovation) * error;
    if (uncertainty < likeliestLandmarkUncertainty) {
      likeliestLandmarkIndex = j;
      likeliestLandmarkError = error;
      likeliestLandmarkUncertainty = uncertainty;
    }
  }
  // Calculate the correction term
  CorrectionTerm correctionTerm;
  if (likeliestLandmarkUncertainty > newLandmarkUncertaintyThreshold) {
    correctionTerm.valid = false;
    return correctionTerm;
  }
  correctionTerm.valid = true;
  const PredictedObservation& predicted
    = predictedObservations[likeliestLandmarkIndex];
  Covariance kalmanGain = (predictedCovariance * predicted.jacobian.transpose())
    * predicted.inverseInnovation;
  correctionTerm.mean = kalmanGain * likeliestLandmarkError;
  correctionTerm.covariance = kalmanGain * predicted.jacobian;
  return correctionTerm;
}

//  Convert an observation into a map-coordinates landmark using the new pose.
static Landmark newLandmarkFromObservation(const Observation& observation,
    const Pose& mean)
{
  Landmark landmark;
  double observationHeading = mean.heading + observation.angle;
  landmark.x = mean.x + (observation.range * cos(observationHeading));
  landmark.y = mean.y + (observation.range * sin(observationHeading));
  landmark.signature = observation.signature;
  return landmark;
}

// ############################################################################

std::vector<Landmark> EKF::run(const Control& control,
    const std::vector<Observation>& observations,
    const std::vector<Landmark>& likelyLandmarks,
    double newLandmarkUncertaintyThreshold)
{
  // Calculate Prediction
  Pose motion = predictMotion(mean_, control);
  Pose predictedMean = sumPose(mean_, motion);
  Covariance predictedCovariance
    = predictCovariance(covariance_, motion, modelCovariance_);

  std::vector<PredictedObservation> predictedObservations(likelyLandmarks.size());
  for (size_t i = 0; i < likelyLandmarks.size(); ++i) {
    predictedObservations[i] = predictObservation(likelyLandmarks[i],
        predictedMean, predictedCovariance, observationCovariance_);
  }

  // Calculate Correction
  Pose correction = {0.0, 0.0, 0.0};
  Covariance covarianceCorrection = Covariance::Zero();
  std::vector<size_t> newLandmarkObservationIndicies;
  for (size_t i = 0; i < observations.size(); ++i) {
    CorrectionTerm correctionTerm = calculateCorrectionTerm(observations[i],
        predictedObservations, predictedCovariance,
        newLandmarkUncertaintyThreshold);
    if (!correctionTerm.valid) {
      newLandmarkObservationIndicies.push_back(i);
      continue;
    }
    correction.x += correctionTerm.mean(0);
    correction.y += correctionTerm.mean(1);
    correction.heading += correctionTerm.mean(2);
    covarianceCorrection += correctionTerm.covariance;
  }

  // Apply Correction
  mean_ = sumPose(predictedMean, correction);
  covariance_ = (Covariance::Identity() - covarianceCorrection)
    * predictedCovariance;

  // Return new landmarks
  std::vector<Landmark> newLandmarks(newLandmarkObservationIndicies.size());
  for (size_t i = 0; i < newLandmarkObservationIndicies.size(); ++i) {
    newLandmarks[i] = newLandmarkFromObservation(
        observations[newLandmarkObservationIndicies[i]], mean_);
  }
  return newLandmarks;
}

} // namespace
