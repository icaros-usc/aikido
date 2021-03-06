#include <gtest/gtest.h>

#include <aikido/statespace/Rn.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/trajectory/util.hpp>

using namespace aikido::statespace;
using aikido::trajectory::Interpolated;
using std::make_shared;
using std::shared_ptr;

class InterpolatedTest : public ::testing::Test
{
public:
  void SetUp()
  {
    rvss = make_shared<R2>();
    interpolator = make_shared<GeodesicInterpolator>(rvss);

    auto s1 = rvss->createState();
    rvss->setValue(s1, Eigen::Vector2d(0, 0));
    auto s2 = rvss->createState();
    rvss->setValue(s2, Eigen::Vector2d(3, 3));
    auto s3 = rvss->createState();
    rvss->setValue(s3, Eigen::Vector2d(8, 1));

    traj = make_shared<Interpolated>(rvss, interpolator);
    traj->addWaypoint(1, s1);
    traj->addWaypoint(3, s2);
    traj->addWaypoint(7, s3);
  }

  shared_ptr<R2> rvss;
  shared_ptr<Interpolator> interpolator;
  shared_ptr<Interpolated> traj;
};

TEST_F(InterpolatedTest, AddWaypoint)
{
  EXPECT_DOUBLE_EQ(1, traj->getStartTime());
  EXPECT_DOUBLE_EQ(7, traj->getEndTime());
  EXPECT_DOUBLE_EQ(6, traj->getDuration());
  EXPECT_EQ(1, traj->getNumDerivatives());
}

TEST_F(InterpolatedTest, EvaluatePt)
{
  auto istate = rvss->createState();

  // Point before first time on traj
  traj->evaluate(-0.001, istate);
  EXPECT_TRUE(rvss->getValue(istate).isApprox(Eigen::Vector2d(0, 0)));

  // Point after last time on traj
  traj->evaluate(8, istate);
  EXPECT_TRUE(rvss->getValue(istate).isApprox(Eigen::Vector2d(8, 1)));

  traj->evaluate(1.5, istate);
  EXPECT_TRUE(rvss->getValue(istate).isApprox(Eigen::Vector2d(.75, .75)));

  traj->evaluate(6, istate);
  EXPECT_TRUE(rvss->getValue(istate).isApprox(
      Eigen::Vector2d(3 + 5 * 0.75, 3 - 2 * .75)));

  traj->evaluate(7, istate);
  EXPECT_TRUE(rvss->getValue(istate).isApprox(Eigen::Vector2d(8, 1)));

  traj->evaluate(3, istate);
  EXPECT_TRUE(rvss->getValue(istate).isApprox(Eigen::Vector2d(3, 3)));
}

TEST_F(InterpolatedTest, EvaluateDerivative)
{
  Eigen::VectorXd tangentVector;

  EXPECT_THROW(
      traj->evaluateDerivative(1.5, 0, tangentVector), std::invalid_argument);

  traj->evaluateDerivative(1.5, 2, tangentVector);
  EXPECT_TRUE(tangentVector.isApprox(Eigen::Vector2d::Zero()));

  traj->evaluateDerivative(1.5, 1, tangentVector);
  EXPECT_TRUE(tangentVector.isApprox(Eigen::Vector2d(1.5, 1.5)));

  traj->evaluateDerivative(6, 1, tangentVector);
  EXPECT_TRUE(tangentVector.isApprox(Eigen::Vector2d(5. / 4, -2. / 4)));
}

TEST_F(InterpolatedTest, ConcatenateTwoTrajectories)
{
  auto rvss = make_shared<R2>();
  auto interpolator = make_shared<GeodesicInterpolator>(rvss);

  auto s1 = rvss->createState();
  rvss->setValue(s1, Eigen::Vector2d(0, 0));
  auto s2 = rvss->createState();
  rvss->setValue(s2, Eigen::Vector2d(3, 3));
  auto s3 = rvss->createState();
  rvss->setValue(s3, Eigen::Vector2d(4, 3));
  auto s4 = rvss->createState();
  rvss->setValue(s4, Eigen::Vector2d(8, 1));

  Interpolated traj1(rvss, interpolator);
  traj1.addWaypoint(0.0, s1);
  traj1.addWaypoint(1.0, s2);

  Interpolated traj2(rvss, interpolator);
  traj2.addWaypoint(2.0, s3);
  traj2.addWaypoint(3.0, s4);

  auto newTraj = aikido::trajectory::concatenate(traj1, traj2);

  Eigen::VectorXd positions;
  auto state = rvss->createState();

  newTraj->evaluate(0.0, state);
  rvss->logMap(state, positions);
  EXPECT_TRUE(Eigen::Vector2d(0., 0.).isApprox(positions));

  newTraj->evaluate(1.0, state);
  rvss->logMap(state, positions);
  EXPECT_TRUE(Eigen::Vector2d(4., 3.).isApprox(positions));

  newTraj->evaluate(2.0, state);
  rvss->logMap(state, positions);
  EXPECT_TRUE(Eigen::Vector2d(8., 1.).isApprox(positions));

  EXPECT_DOUBLE_EQ(
      traj1.getDuration() + traj2.getDuration(), newTraj->getDuration());
}
