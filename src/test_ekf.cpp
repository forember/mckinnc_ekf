#include "mckinnc_ekf/ekf.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace mckinnc_ekf;

#define PANIC(MESSAGE) \
{ \
  traceStream("PANIC: " << MESSAGE); \
  std::exit(1); \
}

static std::vector<std::vector<double>> readCSVFile(const std::string& filePath)
{
  std::vector<std::vector<double>> records;
  std::ifstream fileStream(filePath);
  while (fileStream.good()) {
    // Read a line
    std::string line;
    std::getline(fileStream, line);
    if (line.empty()) {
      continue;
    }
    // Split and parse the fields
    std::vector<double> record;
    size_t start = 0;
    size_t end;
    do {
      end = line.find(',', start);
      if (end == std::string::npos) {
        end = line.size();
      }
      std::string field = line.substr(start, end - start);
      traceStream("Field in " << filePath << ": " << field);
      record.push_back(std::stod(field));
      start = end + 1;
    } while (end < line.size());
    if (!records.empty() && records[0].size() != record.size()) {
      PANIC("Mismatched CSV record lengths.");
    }
    records.push_back(record);
  }
  if (!fileStream.eof()) {
    PANIC("I/O error.");
  }
  return records;
}

static std::vector<std::vector<double>> transpose(
    const std::vector<std::vector<double>>& mat)
{
  std::vector<std::vector<double>> transposed;
  for (size_t i = 0; i < mat[0].size(); ++i) {
    std::vector<double> row;
    for (size_t j = 0; j < mat.size(); ++j) {
      row.push_back(mat[j][i]);
    }
    transposed.push_back(row);
  }
  return transposed;
}

static bool fuzzyEquals(double a, double b)
{
  return std::abs(a - b) <= 0.0001;
}

int main(int argc, char** argv)
{
  if (argc != 2) {
    std::cerr << "usage: " << argv[0] << " <testdata dir>\n";
    return 2;
  }
  std::string testDataDirectory(argv[1]);
  // Load observations
  std::vector<std::vector<double>> observationRecords
    = readCSVFile(testDataDirectory + "/Observations.csv");
  std::vector<std::vector<Observation>> observations;
  for (size_t i = 0; i < observationRecords[0].size(); ++i) {
    std::vector<Observation> currentObservations;
    for (size_t j = 0; j + 1 < observationRecords.size(); j += 2) {
      Observation observation = {
        .range = observationRecords[j][i],
        .angle = observationRecords[j + 1][i],
        .signature = static_cast<double>(i),
      };
      currentObservations.push_back(observation);
    }
    observations.push_back(currentObservations);
  }
  // Load control actions
  std::vector<std::vector<double>> controlActions
    = transpose(readCSVFile(testDataDirectory + "/ControlActions.csv"));
  // Load estimations
  std::vector<std::vector<double>> estimations
    = transpose(readCSVFile(testDataDirectory + "/Estimations.csv"));
  // Load realities
  std::vector<std::vector<double>> realities
    = transpose(readCSVFile(testDataDirectory + "/Realities.csv"));

  if (observations.empty()
      || observations.size() != controlActions.size()
      || controlActions.size() != estimations.size()
      || estimations.size() != realities.size()
      || controlActions[0].size() != 2
      || estimations[0].size() != 3
      || realities[0].size() != 3) {
    PANIC("CSV files have different record lengths.");
  }

  std::vector<Landmark> landmarks;
#if 0
  for (size_t i = 0; i < observations[0].size(); ++i) {
    const Observation& observation = observations[0][i];
    Landmark landmark = {
      .x = observation.range * cos(observation.angle),
      .y = observation.range * sin(observation.angle),
      .signature = observation.signature,
    };
    landmarks.push_back(landmark);
  }
#else
  landmarks.push_back({
      .x = 2.0,
      .y = 2.0,
      .signature = 0.0,
      });
  landmarks.push_back({
      .x = -3.0,
      .y = 5.0,
      .signature = 1.0,
      });
  landmarks.push_back({
      .x = 1.0,
      .y = -2.0,
      .signature = 2.0,
      });
  landmarks.push_back({
      .x = -4.0,
      .y = -4.0,
      .signature = 3.0,
      });
#endif

  Covariance modelCovariance;
  modelCovariance
    <<  1e-6, 0.0, 0.0,
        0.0, 1e-6, 0.0,
        0.0, 0.0, 1e-4;
  Observation observationVariance = {
    .range = 1e-6,
    .angle = 1e-6,
    .signature = 1e-8,
  };

  Pose initialPose = {
    .x = realities[0][0],
    .y = realities[0][1],
    .heading = realities[0][2],
  };
  EKF ekf(initialPose, modelCovariance, observationVariance);

  for (size_t i = 0; i < observations.size(); ++i) {
    Control control = {
      .linear = controlActions[i][0],
      .angular = controlActions[i][1],
      .timeDelta = 1e-3,
    };
    ekf.run(control, observations[i], landmarks, INFINITY);
    Pose mean = ekf.mean();
    if (!fuzzyEquals(mean.x, estimations[i][0])
        || !fuzzyEquals(mean.y, estimations[i][1])
        || !fuzzyEquals(mean.heading, estimations[i][2])) {
      PANIC("Behavior mismatch.");
    }
  }
}
