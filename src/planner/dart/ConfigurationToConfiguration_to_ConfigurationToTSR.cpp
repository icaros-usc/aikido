#include "aikido/planner/dart/ConfigurationToConfiguration_to_ConfigurationToTSR.hpp"

#include <dart/dynamics/dynamics.hpp>
#include "aikido/common/RNG.hpp"
#include "aikido/constraint/dart/InverseKinematicsSampleable.hpp"
#include "aikido/constraint/dart/JointStateSpaceHelpers.hpp"
#include "aikido/planner/dart/util.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSaver.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

using aikido::common::cloneRNGFrom;
using aikido::constraint::dart::InverseKinematicsSampleable;
using aikido::constraint::dart::TSR;
using aikido::constraint::dart::createSampleableBounds;
using aikido::statespace::dart::MetaSkeletonStateSaver;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using ::dart::dynamics::BodyNode;
using ::dart::dynamics::InverseKinematics;
using ::dart::dynamics::ConstBodyNodePtr;
using ::dart::dynamics::BodyNodePtr;

namespace aikido {
namespace planner {
namespace dart {

//==============================================================================
ConfigurationToConfiguration_to_ConfigurationToTSR::
    ConfigurationToConfiguration_to_ConfigurationToTSR(
        std::shared_ptr<planner::ConfigurationToConfigurationPlanner> planner,
        ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
        ::dart::dynamics::ConstBodyNodePtr endEffectorBodyNode)
  : PlannerAdapter<planner::ConfigurationToConfigurationPlanner,
                   ConfigurationToTSRPlanner>(
        std::move(planner), std::move(metaSkeleton))
{
  if (endEffectorBodyNode)
  {
    setEndEffectorBodyNode(endEffectorBodyNode);
  }
}

//==============================================================================
trajectory::TrajectoryPtr
ConfigurationToConfiguration_to_ConfigurationToTSR::plan(
    const ConfigurationToTSR& problem, Planner::Result* result)
{
  {
    std::lock_guard<std::mutex> lock(mMutex);
    mPlanning = true;
  }

  if(!mEndEffectorBodyNode)
  {
    std::lock_guard<std::mutex> lock(mMutex);
    mPlanning = false;
    throw std::runtime_error(
        "ConfigurationToConfiguration_to_ConfigurationToTSR needs to set mEndEffectorBodyNode");
  }
  // TODO: Check equality between state space of this planner and given problem.

  // TODO: DART may be updated to check for single skeleton
  if (mMetaSkeleton->getNumDofs() == 0)
  {
    std::lock_guard<std::mutex> lock(mMutex);
    mPlanning = false;
    throw std::invalid_argument("MetaSkeleton has 0 degrees of freedom.");
  }

  auto skeleton = mMetaSkeleton->getDof(0)->getSkeleton();
  for (std::size_t i = 1; i < mMetaSkeleton->getNumDofs(); ++i)
  {
    if (mMetaSkeleton->getDof(i)->getSkeleton() != skeleton)
    {
      std::lock_guard<std::mutex> lock(mMutex);
      mPlanning = false;
      throw std::invalid_argument("MetaSkeleton has more than 1 skeleton.");
    }
  }

  // Create an IK solver with MetaSkeleton DOFs
  auto matchingNodes
      = skeleton->getBodyNodes(mEndEffectorBodyNode->getName());
  if (matchingNodes.empty())
  {
    std::lock_guard<std::mutex> lock(mMutex);
    mPlanning = false;
    throw std::invalid_argument(
        "End-effector BodyNode not found in Planner's MetaSkeleton.");
  }
  ::dart::dynamics::BodyNodePtr endEffectorBodyNode = matchingNodes.front();

  auto ik = InverseKinematics::create(endEffectorBodyNode);
  ik->setDofs(mMetaSkeleton->getDofs());

  // Get the start state from the MetaSkeleton, since this is a DART planner.
  auto startState = mMetaSkeletonStateSpace->createState();
  mMetaSkeletonStateSpace->getState(mMetaSkeleton.get(), startState);

  auto rng = std::move(cloneRNGFrom(*mDelegate->getRng())[0]);
  // Convert TSR constraint into IK constraint.
  // NOTE: Const-casting should be removed once InverseKinematicsSampleable is
  // changed to take const constraints!
  InverseKinematicsSampleable ikSampleable(
      mMetaSkeletonStateSpace,
      mMetaSkeleton,
      std::const_pointer_cast<TSR>(problem.getGoalTSR()),
      createSampleableBounds(mMetaSkeletonStateSpace, std::move(rng)),
      ik,
      problem.getMaxSamples());
  auto generator = ikSampleable.createSampleGenerator();

  auto saver = MetaSkeletonStateSaver(mMetaSkeleton);
  DART_UNUSED(saver);

  auto goalState = mMetaSkeletonStateSpace->createState();
  while (generator->canSample())
  {
    {
      std::lock_guard<std::mutex> lock(mMutex);
      if (!mPlanning)
      {
        std::cout << "ConfigurationToConfiguration_to_ConfigurationToTSR stop planning" << std::endl;
        return nullptr;
      }
    }

    // Sample from TSR
    {
      std::lock_guard<std::mutex> lock(skeleton->getMutex());
      bool sampled = generator->sample(goalState);
      if (!sampled)
        continue;
    }

    // Create ConfigurationToConfiguration Problem.
    // NOTE: This is done here because the ConfigurationToConfiguration
    // problem stores a *cloned* scoped state of the passed state.
    auto delegateProblem = ConfigurationToConfiguration(
        mMetaSkeletonStateSpace,
        startState,
        goalState,
        problem.getConstraint());

    auto traj = mDelegate->plan(delegateProblem, result);
    if (traj)
    {
      std::lock_guard<std::mutex> lock(mMutex);
      mPlanning = false;
      return traj;
    }
  }

  {
    std::lock_guard<std::mutex> lock(mMutex);
    mPlanning = false;
    return nullptr;
  }
}

//==============================================================================
PlannerPtr ConfigurationToConfiguration_to_ConfigurationToTSR::clone(
    common::RNG* rng) const
{
  throw std::runtime_error(
      "ConfigurationToConfiguration_to_ConfigurationToTSR should clone with metaSkeleton");
}

//==============================================================================
PlannerPtr ConfigurationToConfiguration_to_ConfigurationToTSR::clone(
    ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
    common::RNG* rng) const
{
  // TODO: assert metaSkeleton is a clone of mMetaSkeleton

  using aikido::planner::ConfigurationToConfiguration;

  auto castedDelegate = std::dynamic_pointer_cast<DartPlanner>(mDelegate);
  PlannerPtr clonedDelegate;

  if (castedDelegate)
    clonedDelegate = castedDelegate->clone(metaSkeleton, rng);
  else
    clonedDelegate = mDelegate->clone(rng);

  auto clonedCastedDelegate = std::dynamic_pointer_cast<
    ConfigurationToConfigurationPlanner>(clonedDelegate);

  if (!clonedCastedDelegate)
  {
    // GL: This shouldn't happen, so I think we can use static pointer cast above.
    throw std::runtime_error("Delegate has incorrect type.");
  }

  auto clonedBodyNode = metaSkeleton->getBodyNode(0)->getSkeleton()->getBodyNode(
      mEndEffectorBodyNode->getName())->getBodyNodePtr();

  std::cout << "ConfigurationToConfiguration_to_ConfigurationToTSR has metaSkeleton " <<
    mMetaSkeleton->getBodyNode(0)->getSkeleton()->getName() << " and uses cloned MetaSkeleton: "
    << metaSkeleton->getBodyNode(0)->getSkeleton()->getName() << std::endl;
  std::cout << "Cloned bodyNode " << clonedBodyNode->getName() << std::endl;

  assert(mMetaSkeleton->getBodyNode(0)->getSkeleton() != metaSkeleton->getBodyNode(0)->getSkeleton());

  if (!clonedBodyNode)
  {
    std::stringstream ss;
    ss << "Metaskeleton does not have "
      << mEndEffectorBodyNode->getName() << std::endl;
    throw std::invalid_argument(ss.str());
  }

  return std::make_shared<ConfigurationToConfiguration_to_ConfigurationToTSR>(
      clonedCastedDelegate,
      metaSkeleton,
      clonedBodyNode);
}

//==============================================================================
bool ConfigurationToConfiguration_to_ConfigurationToTSR::stopPlanning()
{
  //  Does not allow stop planning
  std::lock_guard<std::mutex> lock(mMutex);
  mPlanning = false;
  return true;
}


} // namespace dart
} // namespace planner
} // namespace aikido
