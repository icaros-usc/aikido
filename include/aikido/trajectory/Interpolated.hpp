#ifndef AIKIDO_TRAJECTORY_PIECEWISELINEAR_TRAJECTORY_HPP_
#define AIKIDO_TRAJECTORY_PIECEWISELINEAR_TRAJECTORY_HPP_

#include "aikido/common/pointers.hpp"
#include "../statespace/GeodesicInterpolator.hpp"
#include "Trajectory.hpp"

namespace aikido {
namespace trajectory {

AIKIDO_DECLARE_POINTERS(Interpolated)

/// Trajectory that uses an \c Interpolator to interpolate between waypoints.
class Interpolated : public Trajectory
{
public:
  /// Constructs an empty trajectory.
  ///
  /// \param _stateSpace state space this trajectory is defined in
  /// \param _interpolator interpolator used to interpolate between waypoints
  Interpolated(
      statespace::ConstStateSpacePtr _stateSpace,
      statespace::ConstInterpolatorPtr _interpolator);

  virtual ~Interpolated();

  /// Add a waypoint to the trajectory at the given time.
  ///
  /// \param _t time of the waypoint
  /// \param _state state at the waypoint
  void addWaypoint(double _t, const statespace::StateSpace::State* _state);

  /// Insert a waypoint to the trajectory at the given time with a direct copy
  /// without bounding
  ///
  /// \param _t time of the waypoint
  /// \param _state state at the waypoint
  void addWaypointUnbounded(double _t, statespace::StateSpace::State* _state);

  /// Gets a waypoint.
  ///
  /// \param _index waypoint index
  /// \return state of the waypoint at index \c _index
  const statespace::StateSpace::State* getWaypoint(std::size_t _index) const;

  /// Gets the time of a waypoint.
  /// \param _index waypoint index
  /// \return time of the waypoint at index \c _index
  double getWaypointTime(std::size_t _index) const;

  /// Gets the number of waypoints.
  std::size_t getNumWaypoints() const;

  // Documentation inherited
  statespace::ConstStateSpacePtr getStateSpace() const override;

  /// Gets the interpolator used to interpolate between waypoints.
  statespace::ConstInterpolatorPtr getInterpolator() const;

  // Documentation inherited
  std::size_t getNumDerivatives() const override;

  // Documentation inherited
  double getStartTime() const override;

  // Documentation inherited
  double getEndTime() const override;

  // Documentation inherited
  double getDuration() const override;

  // Documentation inherited
  void evaluate(
      double _t, statespace::StateSpace::State* _state) const override;

  // Documentation inherited
  void evaluateDerivative(
      double _t,
      int _derivative,
      Eigen::VectorXd& _tangentVector) const override;

  /// Save the waypoints on current trajectory
  /// This function is mainly used for debugging
  /// \param[in] path The path to the file to write in.
  void save(const std::string& filePath) {
    std::vector<std::vector<double>> traj;
    Eigen::VectorXd outVec;
    for (auto waypoint : mWaypoints) {
      mStateSpace->logMap(waypoint.state, outVec);
      std::vector<double> vecCurrPos;
      vecCurrPos.resize(outVec.size());
      Eigen::VectorXd::Map(&vecCurrPos[0], outVec.size()) = outVec;
      traj.emplace_back(vecCurrPos);
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
    ROS_INFO_STREAM("[Interpolated::save]: One trajectory was saved in " << filePath);
  }

private:
  /// Waypoint in the trajectory.
  struct Waypoint
  {
    Waypoint(double _t, statespace::StateSpace::State* _state);

    /// Comparator to allow sorting waypoints based on time
    bool operator<(const Waypoint& rhs) const;

    /// Comparator to allow sorting waypoints based on time
    bool operator<(double rhs) const;

    /// Free state. Should be used for destruction.
    /// \param[in] stateSpace StateSpace for the state this Waypoint contains.
    void free(const statespace::ConstStateSpacePtr& stateSpace);

    double t;
    statespace::StateSpace::State* state;
  };

  /// Get the index of the first waypoint whose time value is larger than _t.
  /// Throws std::domain_error if _t is larger than last waypoint in the
  /// trajectory.
  int getWaypointIndexAfterTime(double _t) const;

  statespace::ConstStateSpacePtr mStateSpace;
  statespace::ConstInterpolatorPtr mInterpolator;
  std::vector<Waypoint> mWaypoints;
};

} // namespace trajectory
} // namespace aikido

#endif // ifndef AIKIDO_TRAJECTORY_PIECEWISELINEAR_TRAJECTORY_HPP_
