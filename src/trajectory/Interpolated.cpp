#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/statespace/dart/JointStateSpace.hpp>
#include <aikido/statespace/CartesianProduct.hpp>
#include <aikido/statespace/dart/SO2Joint.hpp>

using aikido::statespace::GeodesicInterpolator;

namespace aikido {
namespace trajectory {

using State = aikido::statespace::StateSpace::State;

//==============================================================================
Interpolated::Interpolated(
    statespace::ConstStateSpacePtr _stateSpace,
    statespace::ConstInterpolatorPtr _interpolator)
  : mStateSpace(std::move(_stateSpace)), mInterpolator(std::move(_interpolator))
{
  // Do nothing
}

//==============================================================================
Interpolated::~Interpolated()
{
  for (auto& waypoint : mWaypoints)
    waypoint.free(mStateSpace);
}

//==============================================================================
statespace::ConstStateSpacePtr Interpolated::getStateSpace() const
{
  return mStateSpace;
}

//==============================================================================
statespace::ConstInterpolatorPtr Interpolated::getInterpolator() const
{
  return mInterpolator;
}

//==============================================================================
std::size_t Interpolated::getNumDerivatives() const
{
  return mInterpolator->getNumDerivatives();
}

//==============================================================================
double Interpolated::getStartTime() const
{
  if (mWaypoints.empty())
    throw std::domain_error("Requested getEndTime on empty trajectory.");

  return mWaypoints.front().t;
}

//==============================================================================
double Interpolated::getEndTime() const
{
  if (mWaypoints.empty())
    throw std::domain_error("Requested getEndTime on empty trajectory.");

  return mWaypoints.back().t;
}

//==============================================================================
double Interpolated::getDuration() const
{
  if (!mWaypoints.empty())
    return getEndTime() - getStartTime();
  else
    return 0.;
}

//==============================================================================
void Interpolated::evaluate(double _t, State* _state) const
{
  if (mWaypoints.empty())
    throw std::invalid_argument(
        "Requested trajectory point from an empty trajectory");

  try
  {
    int idx = getWaypointIndexAfterTime(_t);
    if (idx == 0)
    {
      // Time before beginning of trajectory - return first waypoint
      mStateSpace->copyState(mWaypoints[0].state, _state);
    }
    else
    {
      Waypoint currentWpt = mWaypoints[idx];
      Waypoint prevWpt = mWaypoints[idx - 1];
      mInterpolator->interpolate(
          prevWpt.state,
          currentWpt.state,
          (_t - prevWpt.t) / (currentWpt.t - prevWpt.t),
          _state);
    }
  }
  catch (const std::domain_error& e)
  {
    // Time past end of trajectory - return last waypoint
    mStateSpace->copyState(mWaypoints.back().state, _state);
  }
}

//==============================================================================
void Interpolated::evaluateDerivative(
    double _t, int _derivative, Eigen::VectorXd& _tangentVector) const
{
  if (_derivative == 0)
    throw std::invalid_argument(
        "0th derivative not available. Use evaluate(t, state).");

  if (static_cast<std::size_t>(_derivative)
      > mInterpolator->getNumDerivatives())
  {
    _tangentVector.resize(mStateSpace->getDimension());
    _tangentVector.setZero();
  }

  // Compute the interpolated state at time t
  auto iPt = mStateSpace->createState();
  evaluate(_t, iPt);

  // Compute the endpoint of the segment
  try
  {
    int idx = getWaypointIndexAfterTime(_t);

    // Time before beginning of trajectory - return 0
    if (idx == 0)
      throw std::domain_error("Time is before the trajectory starts.");

    const auto segmentTime = mWaypoints[idx].t - mWaypoints[idx - 1].t;
    const auto alpha = (_t - mWaypoints[idx - 1].t) / segmentTime;

    mInterpolator->getDerivative(
        mWaypoints[idx - 1].state,
        mWaypoints[idx].state,
        _derivative,
        alpha,
        _tangentVector);

    _tangentVector /= segmentTime;
  }
  // Time past end of trajectory - return zero
  catch (const std::domain_error& e)
  {
    _tangentVector.resize(mStateSpace->getDimension());
    _tangentVector.setZero();
  }
}

//==============================================================================
void Interpolated::addWaypoint(double _t, const State* _state)
{
  State* state = mStateSpace->allocateState();
  mStateSpace->copyState(_state, state);

  // Maintain a sorted list of waypoints
  auto it = std::lower_bound(mWaypoints.begin(), mWaypoints.end(), _t);
  mWaypoints.insert(it, Waypoint(_t, state));
}

//==============================================================================
void Interpolated::addWaypointUnbounded(double _t, State* _state)
{
  const auto *_stateSpace = dynamic_cast<const aikido::statespace::CartesianProduct *>(mStateSpace.get());
  auto newState = _stateSpace->allocateState();
  const auto n = _stateSpace->getNumSubspaces();
  Eigen::VectorXd subVec;
  for (std::size_t i = 0; i < n; ++i) {
    auto subspace = _stateSpace->getSubspace<aikido::statespace::dart::JointStateSpace>(i);
    auto _subState = _stateSpace->getSubState<>(static_cast<aikido::statespace::CartesianProduct::State *>(_state), i);
    auto newSubState = _stateSpace->getSubState<>(static_cast<aikido::statespace::CartesianProduct::State *>(newState), i);
    subspace->logMap(_subState, subVec);
    if (dynamic_cast<const aikido::statespace::dart::SO2Joint *>(subspace.get())) {
      dynamic_cast<const aikido::statespace::dart::SO2Joint *>(subspace.get())->expMapUnbounded(subVec, newSubState);
    } else {
      subspace->expMap(subVec, newSubState);
    }
  }

  // Maintain a sorted list of waypoints
  auto it = std::lower_bound(mWaypoints.begin(), mWaypoints.end(), _t);
  mWaypoints.insert(it, Waypoint(_t, newState));
}

//==============================================================================
const statespace::StateSpace::State* Interpolated::getWaypoint(
    std::size_t _index) const
{
  if (_index < mWaypoints.size())
    return mWaypoints[_index].state;
  else
    throw std::domain_error("Waypoint index is out of bounds.");
}

//==============================================================================
double Interpolated::getWaypointTime(std::size_t _index) const
{
  if (_index < mWaypoints.size())
    return mWaypoints[_index].t;
  else
    throw std::domain_error("Waypoint index is out of bounds.");
}

//==============================================================================
std::size_t Interpolated::getNumWaypoints() const
{
  return mWaypoints.size();
}

//==============================================================================
int Interpolated::getWaypointIndexAfterTime(double _t) const
{
  auto it = std::lower_bound(mWaypoints.begin(), mWaypoints.end(), _t);
  if (it == mWaypoints.end())
  {
    throw std::domain_error(
        "_t is larger than the time value on the last waypoint.");
  }

  return std::distance(mWaypoints.begin(), it);
}

//==============================================================================
Interpolated::Waypoint::Waypoint(
    double _t, statespace::StateSpace::State* _state)
  : t(_t), state(_state)
{
}

//==============================================================================
bool Interpolated::Waypoint::operator<(const Waypoint& rhs) const
{
  return t < rhs.t;
}

//==============================================================================
bool Interpolated::Waypoint::operator<(double rhs) const
{
  return t < rhs;
}

//==============================================================================
void Interpolated::Waypoint::free(
    const statespace::ConstStateSpacePtr& stateSpace)
{
  if (state)
  {
    stateSpace->freeState(state);
    state = nullptr;
  }
}

} // namespace trajectory
} // namespace aikido
