#ifndef AIKIDO_TRAJECTORY_TRAJECTORY_HPP_
#define AIKIDO_TRAJECTORY_TRAJECTORY_HPP_

#include <fstream>
#include <iostream>
#include <Eigen/Core>
#include <ros/ros.h>
#include "aikido/common/pointers.hpp"
#include <aikido/trajectory/TrajectoryMetadata.hpp>
#include "../statespace/StateSpace.hpp"

namespace aikido {
namespace trajectory {

AIKIDO_DECLARE_POINTERS(Trajectory)

/// Time-parameterized path in a \c StateSpace. The parameterization, number of
/// derivatives available, and continuity of this trajectory is defined by the
/// concrete implementation of this class. The interpretation of the time
/// parameter is also implementation defined: it may represent an actual time
/// time or some other value (e.g. arc length under a distance metric).
class Trajectory {
 public:
  virtual ~Trajectory() = default;

  /// Gets the \c StateSpace that this trajectory is defined in.
  ///
  /// \return state space this trajectory is defined in.
  virtual statespace::ConstStateSpacePtr getStateSpace() const = 0;

  /// Gets an upper bound on the number of non-zero derivatives available in
  /// this parameterization. Note that \c evaluateDerivative may return zero
  /// before this value for some trajectories.
  ///
  /// \return upper bound on the number of non-zero derivatives
  virtual std::size_t getNumDerivatives() const = 0;

  /// Duration of the trajectory. Note that \c getStartTime() may not be zero.
  ///
  /// \return duration of the trajectory
  virtual double getDuration() const = 0;

  /// Time at which the trajectory starts. This may not be zero.
  ///
  /// \return time at which the trajectory starts
  virtual double getStartTime() const = 0;

  /// Time at which the trajectory ends. This may not be \c getDuration() if
  /// \c getStartTime() is not zero.
  ///
  /// \return time at which the trajectory ends
  virtual double getEndTime() const = 0;

  /// Evaluates the state of the trajectory at time \c _t and store the result
  /// in a \c _state allocated by \c getStateSpace(). The output of this
  /// function is implementation-defined if \c _t is not between
  /// \c getStartTime() and \c getEndTime().
  ///
  /// \param _t time parameter
  /// \param[out] _state output state of the trajectory at time \c _t
  virtual void evaluate(
      double _t, statespace::StateSpace::State *_state) const = 0;

  /// Evaluates the state of the trajectory at time \c _t and store the result
  ///
  void evaluate(double _t, Eigen::VectorXd &_vector) {
    auto stateSpace = getStateSpace();
    auto state = stateSpace->createState();
    this->evaluate(_t, state);
    stateSpace->logMap(state, _vector);
  }

  /// Save the waypoints on current trajectory
  /// This function is mainly used for debugging
  /// \param[in] path The path to the file to write in.
  void save(const std::string& filePath) {
    auto start = this->getStartTime();
    auto end = this->getEndTime();
    std::vector<std::vector<double>> traj;
    auto curr = start;
    auto delta = 0.03;
    while (true) {
      Eigen::VectorXd currPos;
      this->evaluate(curr, currPos);
      std::vector<double> vecCurrPos;
      vecCurrPos.resize(currPos.size());
      Eigen::VectorXd::Map(&vecCurrPos[0], currPos.size()) = currPos;
      traj.emplace_back(vecCurrPos);
      curr += delta;
      if (curr > end) {
        break;
      }
    }
    std::ofstream cachingFile;
    cachingFile.open(filePath);
    for (const auto &positions : traj) {
      for (const auto &position : positions) {
        cachingFile << position << ' ';
      }
      cachingFile << '\n';
    }
    cachingFile.close();
    ROS_INFO_STREAM("[Trajectory::save]: One trajectory was saved in " << filePath);
  }

  /// Evaluates the derivative of the trajectory at time \c _t. The
  /// \c _tangentVector is defined in the local frame (i.e. "body frame") and
  /// is implementation-defined if not between \c getStartTime() and
  /// \c getEndTime(). Derivatives of order higher than \c getNumDerivatives
  /// are guaranteed to be zero.
  ///
  /// \param _t time parameter
  /// \param _derivative order of derivative
  /// \param[out] _tangentVector output tangent vector in the local frame
  virtual void evaluateDerivative(
      double _t, int _derivative, Eigen::VectorXd &_tangentVector) const = 0;

  /// Trajectory metadata
  // TODO: Is metadata required anymore? Delete if not.
  TrajectoryMetadata metadata;
};

} // namespace trajectory
} // namespace aikido

#endif // ifndef AIKIDO_TRAJECTORY_TRAJECTORY_HPP_
