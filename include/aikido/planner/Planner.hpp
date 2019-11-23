#ifndef AIKIDO_PLANNER_PLANNER_HPP_
#define AIKIDO_PLANNER_PLANNER_HPP_

#include <string>

#include "aikido/common/pointers.hpp"
#include "aikido/planner/Problem.hpp"
#include "aikido/statespace/StateSpace.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace planner {

AIKIDO_DECLARE_POINTERS(Planner)

/// Base class for a meta-planner
class Planner
{
public:
  class Result;

  /// Constructs from a state space.
  ///
  /// \param[in] stateSpace State space that this planner associated with.
  /// \param[in] rng RNG that planner uses. If nullptr, a default is created.
  explicit Planner(
      statespace::ConstStateSpacePtr stateSpace, common::RNG* rng = nullptr);

  /// Default destructor.
  virtual ~Planner() = default;

  /// Returns const state space.
  statespace::ConstStateSpacePtr getStateSpace() const;

  /// Returns RNG.
  common::RNG* getRng();

  /// Returns true if this planner can solve \c problem.
  virtual bool canSolve(const Problem& problem) const = 0;

  /// Solves \c problem returning the result to \c result.
  ///
  /// \param[in] problem Planning problem to be solved by the planner.
  /// \param[out] result Result of planning procedure.
  virtual trajectory::TrajectoryPtr plan(
      const Problem& problem, Result* result = nullptr)
      = 0;

protected:
  /// State space associated with this planner.
  statespace::ConstStateSpacePtr mStateSpace;

  /// RNG the planner uses.
  std::unique_ptr<common::RNG> mRng;
};

/// Base class for planning result of various planning problems.
class Planner::Result
{
public:
  /// The possible values of the status returned by a planner
  /// corresponding to ompl PlannerStatus
  enum StatusType
  {
    /// Uninitialized status
        UNKNOWN = 0,
    /// Invalid start state or no start state specified
        INVALID_START,
    /// Invalid goal state
        INVALID_GOAL,
    /// The goal is of a type that a planner does not recognize
        UNRECOGNIZED_GOAL_TYPE,
    /// The planner failed to find a solution
        TIMEOUT,
    /// The planner found an approximate solution or an exact solution
        SUCCEEED,
    /// The planner crashed
        CRASH,
    /// The planner did not find a solution for some other reason
        ABORT,
    /// The number of possible status values
        TYPE_COUNT
  };

  /// Constructor.
  ///
  /// \param[in] message Planning result message.
  explicit Result(const std::string& message = "");

  /// Destructor.
  virtual ~Result() = default;

  /// Sets message.
  void setMessage(const std::string& message);

  /// Returns message.
  const std::string& getMessage() const;

  /// Sets status.
  void setStatus(const StatusType& status);

  /// Returns status.
  const StatusType& getStatus() const;

protected:
  /// Message.
  std::string mMessage;
  StatusType mStatus;
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_PLANNER_HPP_
