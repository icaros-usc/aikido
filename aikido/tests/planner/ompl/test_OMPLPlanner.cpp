#include "OMPLTestHelpers.hpp"
#include "../../constraint/MockConstraints.hpp"
#include <aikido/planner/ompl/Planner.hpp>
#include <aikido/constraint/uniform/RnBoxConstraint.hpp>
#include <aikido/constraint/CartesianProductSampleable.hpp>
#include <aikido/constraint/TestableSubspace.hpp>
#include <aikido/constraint/JointStateSpaceHelpers.hpp>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

using StateSpace = aikido::statespace::dart::MetaSkeletonStateSpace;
using Rn = aikido::statespace::Rn;
using aikido::planner::ompl::getSpaceInformation;

TEST_F(PlannerTest, PlanToConfiguration)
{
  Eigen::Vector3d startPose(-5, -5, 0);
  Eigen::Vector3d goalPose(5, 5, 0);

  auto startState = stateSpace->createState();
  auto subState1 =
      stateSpace->getSubStateHandle<Rn>(startState, 0);
  subState1.setValue(startPose);

  auto goalState = stateSpace->createState();
  auto subState2 =
      stateSpace->getSubStateHandle<Rn>(goalState, 0);
  subState2.setValue(goalPose);

  // Plan
  auto traj = aikido::planner::ompl::planOMPL<ompl::geometric::RRTConnect>(
      startState, goalState, stateSpace, interpolator, std::move(dmetric),
      std::move(sampler), std::move(collConstraint),
      std::move(boundsConstraint), std::move(boundsProjection), 5.0);

  // Check the first waypoint
  auto s0 = stateSpace->createState();
  traj->evaluate(0, s0);
  auto r0 = s0.getSubStateHandle<Rn>(0);
  EXPECT_TRUE(r0.getValue().isApprox(startPose));

  // Check the last waypoint
  traj->evaluate(traj->getDuration(), s0);
  r0 = s0.getSubStateHandle<Rn>(0);
  EXPECT_TRUE(r0.getValue().isApprox(goalPose));
}

TEST_F(PlannerTest, PlanToGoalRegion)
{
  auto startState = stateSpace->createState();
  Eigen::Vector3d startPose(-5, -5, 0);

  auto subState1 =
      stateSpace->getSubStateHandle<Rn>(startState, 0);
  subState1.setValue(startPose);

  auto boxConstraint =
      std::make_shared<aikido::constraint::RnBoxConstraint>(
          stateSpace->getSubSpace<Rn>(0), make_rng(),
          Eigen::Vector3d(4, 4, 0), Eigen::Vector3d(5, 5, 0));
  std::vector<std::shared_ptr<aikido::constraint::Sampleable>>
      sConstraints;
  sConstraints.push_back(boxConstraint);
  aikido::constraint::SampleablePtr goalSampleable =
      std::make_shared<aikido::constraint::CartesianProductSampleable>(stateSpace,
                                                               sConstraints);
  std::vector<std::shared_ptr<aikido::constraint::Testable>>
      tConstraints;
  tConstraints.push_back(boxConstraint);
  aikido::constraint::TestablePtr goalTestable =
      std::make_shared<aikido::constraint::TestableSubspace>(stateSpace,
                                                             tConstraints);

  // Plan
  auto traj = aikido::planner::ompl::planOMPL<ompl::geometric::RRTConnect>(
      startState, goalTestable, goalSampleable, stateSpace, interpolator,
      std::move(dmetric), std::move(sampler), std::move(collConstraint),
      std::move(boundsConstraint), std::move(boundsProjection), 5.0);

  // Check the first waypoint
  auto s0 = stateSpace->createState();
  traj->evaluate(0, s0);
  auto r0 = s0.getSubStateHandle<Rn>(0);
  EXPECT_TRUE(r0.getValue().isApprox(startPose));

  // Check the last waypoint
  traj->evaluate(traj->getDuration(), s0);
  EXPECT_TRUE(goalTestable->isSatisfied(s0));
}

TEST_F(PlannerTest, PlanThrowsOnNullGoalTestable)
{
  auto startState = stateSpace->createState();
  Eigen::Vector3d startPose(-5, -5, 0);

  auto subState1 =
      stateSpace->getSubStateHandle<Rn>(startState, 0);
  subState1.setValue(startPose);

  // Easiest sampleable to get
  auto goalSampleable =
      aikido::constraint::createSampleableBounds(stateSpace, make_rng());

  // Plan
  EXPECT_THROW(aikido::planner::ompl::planOMPL<ompl::geometric::RRTConnect>(
                   startState, nullptr, std::move(goalSampleable), stateSpace,
                   interpolator, std::move(dmetric), std::move(sampler),
                   std::move(collConstraint), std::move(boundsConstraint),
                   std::move(boundsProjection), 5.0),
               std::invalid_argument);
}

TEST_F(PlannerTest, PlanThrowsOnGoalTestableMismatch)
{
  auto startState = stateSpace->createState();
  Eigen::Vector3d startPose(-5, -5, 0);

  auto subState1 =
      stateSpace->getSubStateHandle<Rn>(startState, 0);
  subState1.setValue(startPose);

  // Easiest sampleable to get
  auto goalSampleable =
      aikido::constraint::createSampleableBounds(stateSpace, make_rng());

  // Easiest testable to get
  auto ss = std::make_shared<StateSpace>(robot);
  auto goalTestable = std::make_shared<PassingConstraint>(ss);

  // Plan
  EXPECT_THROW(
      aikido::planner::ompl::planOMPL<ompl::geometric::RRTConnect>(
          startState, goalTestable, std::move(goalSampleable), stateSpace,
          interpolator, std::move(dmetric), std::move(sampler),
          std::move(collConstraint), std::move(boundsConstraint),
          std::move(boundsProjection), 5.0),
      std::invalid_argument);
}

TEST_F(PlannerTest, PlanThrowsOnNullGoalSampler)
{
  auto startState = stateSpace->createState();
  Eigen::Vector3d startPose(-5, -5, 0);

  auto subState1 =
      stateSpace->getSubStateHandle<Rn>(startState, 0);
  subState1.setValue(startPose);

  // Dummy testable
  auto goalTestable = std::make_shared<PassingConstraint>(stateSpace);

  // Plan
  EXPECT_THROW(
      aikido::planner::ompl::planOMPL<ompl::geometric::RRTConnect>(
          startState, goalTestable, nullptr, stateSpace, interpolator,
          std::move(dmetric), std::move(sampler), std::move(collConstraint),
          std::move(boundsConstraint), std::move(boundsProjection), 5.0),
      std::invalid_argument);
}

TEST_F(PlannerTest, PlanThrowsOnGoalSamplerMismatch)
{
  auto startState = stateSpace->createState();
  Eigen::Vector3d startPose(-5, -5, 0);

  auto subState1 =
      stateSpace->getSubStateHandle<Rn>(startState, 0);
  subState1.setValue(startPose);

  // Easiest sampleable to get
  auto ss = std::make_shared<StateSpace>(robot);
  auto goalSampleable =
      aikido::constraint::createSampleableBounds(ss, make_rng());

  // Dummy testable
  auto goalTestable = std::make_shared<PassingConstraint>(stateSpace);

  // Plan
  EXPECT_THROW(
      aikido::planner::ompl::planOMPL<ompl::geometric::RRTConnect>(
          startState, goalTestable, std::move(goalSampleable), stateSpace,
          interpolator, std::move(dmetric), std::move(sampler),
          std::move(collConstraint), std::move(boundsConstraint),
          std::move(boundsProjection), 5.0),
      std::invalid_argument);
}

TEST_F(PlannerTest, GetSpaceInformationThrowsOnNullStateSpace)
{
  EXPECT_THROW(getSpaceInformation(
                   nullptr, std::move(interpolator), std::move(dmetric),
                   std::move(sampler), std::move(collConstraint),
                   std::move(boundsConstraint), std::move(boundsProjection)),
               std::invalid_argument);
}

TEST_F(PlannerTest, GetSpaceInformationThrowsOnNullInterpolator)
{
  EXPECT_THROW(getSpaceInformation(
                   std::move(stateSpace), nullptr, std::move(dmetric),
                   std::move(sampler), std::move(collConstraint),
                   std::move(boundsConstraint), std::move(boundsProjection)),
               std::invalid_argument);
}

TEST_F(PlannerTest, GetSpaceInformationThrowsOnInterpolatorMismatch)
{
  auto ss = std::make_shared<aikido::statespace::SO2>();
  auto binterpolator =
      std::make_shared<aikido::statespace::GeodesicInterpolator>(ss);

  EXPECT_THROW(
      getSpaceInformation(
          std::move(stateSpace), std::move(binterpolator), std::move(dmetric),
          std::move(sampler), std::move(collConstraint),
          std::move(boundsConstraint), std::move(boundsProjection)),
      std::invalid_argument);
}

TEST_F(PlannerTest, GetSpaceInformationThrowsOnNullDistanceMetric)
{
  EXPECT_THROW(getSpaceInformation(
                   std::move(stateSpace), std::move(interpolator), nullptr,
                   std::move(sampler), std::move(collConstraint),
                   std::move(boundsConstraint), std::move(boundsProjection)),
               std::invalid_argument);
}

TEST_F(PlannerTest, GetSpaceInformationThrowsOnDistanceMetricMismatch)
{
  auto ss = std::make_shared<StateSpace>(robot);
  auto dm = aikido::distance::createDistanceMetric(ss);

  EXPECT_THROW(getSpaceInformation(
                   std::move(stateSpace), std::move(interpolator),
                   std::move(dm), std::move(sampler), std::move(collConstraint),
                   std::move(boundsConstraint), std::move(boundsProjection)),
               std::invalid_argument);
}

TEST_F(PlannerTest, GetSpaceInformationThrowsOnNullSampler)
{
  EXPECT_THROW(getSpaceInformation(
                   std::move(stateSpace), std::move(interpolator),
                   std::move(dmetric), nullptr, std::move(collConstraint),
                   std::move(boundsConstraint), std::move(boundsProjection)),
               std::invalid_argument);
}

TEST_F(PlannerTest, GetSpaceInformationThrowsOnSamplerMismatch)
{
  auto ss = std::make_shared<StateSpace>(robot);
  auto ds = aikido::constraint::createSampleableBounds(ss, make_rng());

  EXPECT_THROW(getSpaceInformation(
                   std::move(stateSpace), std::move(interpolator),
                   std::move(dmetric), std::move(ds), std::move(collConstraint),
                   std::move(boundsConstraint), std::move(boundsProjection)),
               std::invalid_argument);
}

TEST_F(PlannerTest, GetSpaceInformationThrowsOnNullValidityConstraint)
{
  EXPECT_THROW(getSpaceInformation(
                   std::move(stateSpace), std::move(interpolator),
                   std::move(dmetric), std::move(sampler), nullptr,
                   std::move(boundsConstraint), std::move(boundsProjection)),
               std::invalid_argument);
}

TEST_F(PlannerTest, GetSpaceInformationThrowsOnValidityConstraintMismatch)
{
  auto ss = std::make_shared<StateSpace>(robot);
  auto dv = std::make_shared<MockTranslationalRobotConstraint>(
      ss, Eigen::Vector3d(-0.1, -0.1, -0.1), Eigen::Vector3d(0.1, 0.1, 0.1));
  EXPECT_THROW(getSpaceInformation(
                   std::move(stateSpace), std::move(interpolator),
                   std::move(dmetric), std::move(sampler), std::move(dv),
                   std::move(boundsConstraint), std::move(boundsProjection)),
               std::invalid_argument);
}

TEST_F(PlannerTest, GetSpaceInformationThrowsOnNullBoundsConstraint)
{
  EXPECT_THROW(
      getSpaceInformation(std::move(stateSpace), std::move(interpolator),
                          std::move(dmetric), std::move(sampler),
                          std::move(collConstraint), nullptr,
                          std::move(boundsProjection)),
      std::invalid_argument);
}

TEST_F(PlannerTest, GetSpaceInformationThrowsOnBoundsConstraintMismatch)
{
  auto ss = std::make_shared<StateSpace>(robot);
  auto ds = aikido::constraint::createTestableBounds(ss);

  EXPECT_THROW(
      getSpaceInformation(std::move(stateSpace), std::move(interpolator),
                          std::move(dmetric), std::move(sampler),
                          std::move(collConstraint), std::move(ds),
                          std::move(boundsProjection)),
      std::invalid_argument);
}

TEST_F(PlannerTest, GetSpaceInformationThrowsOnNullBoundsProjector)
{
  EXPECT_THROW(
      getSpaceInformation(std::move(stateSpace), std::move(interpolator),
                          std::move(dmetric), std::move(sampler),
                          std::move(collConstraint),
                          std::move(boundsConstraint), nullptr),
      std::invalid_argument);
}

TEST_F(PlannerTest, GetSpaceInformationThrowsOnBoundsProjectorMismatch)
{
  auto ss = std::make_shared<StateSpace>(robot);
  auto ds = aikido::constraint::createProjectableBounds(ss);

  EXPECT_THROW(
      getSpaceInformation(std::move(stateSpace), std::move(interpolator),
                          std::move(dmetric), std::move(sampler),
                          std::move(collConstraint),
                          std::move(boundsConstraint), std::move(ds)),
      std::invalid_argument);
}

TEST_F(PlannerTest, GetSpaceInformationNotNull)
{
  auto si = getSpaceInformation(
      std::move(stateSpace), std::move(interpolator), std::move(dmetric),
      std::move(sampler), std::move(collConstraint),
      std::move(boundsConstraint), std::move(boundsProjection));
  EXPECT_FALSE(si == nullptr);
}

TEST_F(PlannerTest, GetSpaceInformationCreatesGeometricStateSpace)
{
  auto si = getSpaceInformation(
      std::move(stateSpace), std::move(interpolator), std::move(dmetric),
      std::move(sampler), std::move(collConstraint),
      std::move(boundsConstraint), std::move(boundsProjection));

  auto ss = boost::dynamic_pointer_cast<aikido::planner::ompl::GeometricStateSpace>(
      si->getStateSpace());
  EXPECT_FALSE(ss == nullptr);
}

TEST_F(PlannerTest, GetSpaceInformationCreatesValidityChecker)
{
  auto si = getSpaceInformation(
      std::move(stateSpace), std::move(interpolator), std::move(dmetric),
      std::move(sampler), std::move(collConstraint),
      std::move(boundsConstraint), std::move(boundsProjection));

  auto vc = boost::dynamic_pointer_cast<aikido::planner::ompl::StateValidityChecker>(
      si->getStateValidityChecker());
  EXPECT_FALSE(vc == nullptr);
}

