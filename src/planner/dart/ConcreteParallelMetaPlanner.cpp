#include "aikido/planner/dart/ConcreteParallelMetaPlanner.hpp"
#include "aikido/planner/dart/util.hpp"
#include "aikido/planner/dart/DartProblem.hpp"
#include "aikido/planner/dart/DartPlanner.hpp"

#include <thread>
#include <future>
#include <boost/timer.hpp>
#include <fstream>
#include <iostream>

namespace aikido {
namespace planner {
namespace dart{

// Planning call for individual planner.
static void _plan(
      const PlannerPtr& planner,
      const std::shared_ptr<std::promise<trajectory::TrajectoryPtr>>& promise,
      const ConstProblemPtr& problem,
      const std::shared_ptr<Planner::Result>& result,
      const std::string& filename,
      std::size_t planner_id
      )
{
  boost::timer planningTimer;
  trajectory::TrajectoryPtr trajectory = planner->plan(*problem, result.get());
  if (trajectory)
  {
    promise->set_value(trajectory);
    // Record the planning time here.
    std::ofstream logFile;
    logFile.open(filename, std::ios_base::app);
    logFile << planner_id << "th planner: " << planningTimer.elapsed() << std::endl;
    return;
  }

  std::ofstream logFile;
  logFile.open(filename, std::ios_base::app);
  logFile << planner_id << "th planner [fail]: " << planningTimer.elapsed() << std::endl;
  promise->set_value(nullptr);
}

//==============================================================================
ConcreteParallelMetaPlanner::ConcreteParallelMetaPlanner(
    statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
    std::vector<::dart::collision::CollisionDetectorPtr> collisionDetectors,
    const std::vector<PlannerPtr>& planners)
: ParallelMetaPlanner(std::move(stateSpace), planners)
, mMetaSkeleton(metaSkeleton)
, mCollisionDetectors(std::move(collisionDetectors))
, mRunning(false)
{
  // Do nothing
  // TODO: need to acquire metaSkeletons from the planners
}

//==============================================================================
ConcreteParallelMetaPlanner::ConcreteParallelMetaPlanner(
      statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
      ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
      std::vector<::dart::collision::CollisionDetectorPtr> collisionDetectors,
      const PlannerPtr& planner,
      const std::vector<common::RNG*> rngs)
: ParallelMetaPlanner(std::move(stateSpace))
, mMetaSkeleton(metaSkeleton)
, mCollisionDetectors(std::move(collisionDetectors))
, mRunning(false)
{
  if (rngs.size() > 0 && mCollisionDetectors.size() != rngs.size())
  {
    std::stringstream ss;
    ss << "Number of collision detectors [" << mCollisionDetectors.size() << "] does not match number of RNGs [ "
      << rngs.size() << "]."  << std::endl;
    throw std::invalid_argument(ss.str());
  }

  auto castedPlanner = std::dynamic_pointer_cast<const DartPlanner>(planner);

  auto numCopies = mCollisionDetectors.size();
  mPlanners.reserve(numCopies);
  mClonedMetaSkeletons.reserve(numCopies);

  for (std::size_t i = 0; i < mCollisionDetectors.size(); ++i)
  {
    auto clonedMetaSkeleton = util::clone(mMetaSkeleton);
    mClonedMetaSkeletons.emplace_back(clonedMetaSkeleton);
    if (rngs.size() == numCopies)
    {
      if (castedPlanner)
        mPlanners.emplace_back(castedPlanner->clone(clonedMetaSkeleton, rngs[i]));
      else
        mPlanners.emplace_back(planner->clone(rngs[i]));
    }
    else
    {
      if (castedPlanner)
        mPlanners.emplace_back(castedPlanner->clone(clonedMetaSkeleton));
      else
        mPlanners.emplace_back(planner->clone());
    }
  }

}

//==============================================================================
trajectory::TrajectoryPtr ConcreteParallelMetaPlanner::plan(
    const Problem& problem, Result* result)
{
  {
    std::lock_guard<std::mutex> lock(mMutex);
    if (mRunning)
    {
      throw std::runtime_error("Currently planning another problem");
    }

    mRunning = true;
  }

  std::vector<std::future<trajectory::TrajectoryPtr>> futures;
  std::vector<std::thread> threads;
  std::vector<ProblemPtr> clonedProblems;
  std::vector<std::shared_ptr<Result>> results;

  // Check if problem is DartProblem
  const DartProblem* dart_problem = dynamic_cast<const DartProblem*>(&problem);
  std::vector<std::shared_ptr<Problem>> shared_problems;
  std::shared_ptr<Problem> shared_problem;
  if (!dart_problem)
    shared_problem = problem.clone();

  results.reserve(mPlanners.size());

  std::string filename = std::to_string(mPlanners.size()) + "_planners.txt";

  for (std::size_t i = 0; i < mPlanners.size(); ++i)
  {
    if (!mPlanners[i]->canSolve(problem))
      continue;

    auto result = std::make_shared<Result>();

    results.push_back(result);
    auto promise = std::make_shared<std::promise<trajectory::TrajectoryPtr>>();


    if (dart_problem)
    {
      auto shared_problem = dart_problem->clone(mCollisionDetectors[i],
          mClonedMetaSkeletons[i]);

      auto thread = std::thread(_plan, mPlanners[i], promise, shared_problem, result, filename, i);
      thread.detach();
    }
    else
    {
      std::cout << "Not dart problem" << std::endl;
      throw std::runtime_error("Not supported");
      threads.push_back(std::thread(_plan, mPlanners[i], promise, shared_problem, result, filename, i));
    }

    futures.push_back(promise->get_future());
  }

  std::cout << "Total of [" << futures.size() << "] threads running." << std::endl;

  if (futures.size() == 0)
  {
    std::lock_guard<std::mutex> lock(mMutex);
    mRunning = false;
    return nullptr;
  }

  std::vector<bool> future_retrieved(results.size(), false);
  do
  {
    std::size_t num_failures = 0;
    std::this_thread::sleep_for(std::chrono::nanoseconds(1));
    for(std::size_t i = 0; i < futures.size(); ++i)
    {
      if (future_retrieved[i])
        continue;

      auto status = futures[i].wait_for(std::chrono::milliseconds(0));
      if (status == std::future_status::ready)
      {
        future_retrieved[i] = true;
        auto trajectory = futures[i].get();
        if (trajectory)
        {
          std::cout << i << "th future succeeded." << std::endl;
          // TODO: kill them forcefully (need a stopable thread)
          for(auto planner : mPlanners)
            planner->stopPlanning();

          // Return
          {
            std::lock_guard<std::mutex> lock(mMutex);
            mRunning = false;
          // TODO:: copy result
            return trajectory;
          }
        }
        else
          num_failures++;
      }

    }
    if (num_failures == futures.size())
    {
      {
        std::lock_guard<std::mutex> lock(mMutex);
        mRunning = false;
      }
      std::cout << "Failed by all planners" << std::endl;
      return nullptr;
    }
  }while(true);

  {
    std::lock_guard<std::mutex> lock(mMutex);
    mRunning = false;
  }

  return nullptr;

}

//==============================================================================
PlannerPtr ConcreteParallelMetaPlanner::clone(common::RNG* rng) const
{
  throw std::runtime_error("Cloning MetaPlanner is not suported.");
}

//==============================================================================
PlannerPtr ConcreteParallelMetaPlanner::clone(
      ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
      common::RNG* rng) const
{
  throw std::runtime_error("Cloning MetaPlanner is not suported.");
}

//==============================================================================
bool ConcreteParallelMetaPlanner::stopPlanning()
{
  return false;
}

} // namespace dart
} // namespace planner
} // namespace aikido
