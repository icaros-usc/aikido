#include "aikido/constraint/dart/NominalConfigurationStrategy.hpp"

namespace aikido {
namespace constraint {
namespace dart {

using statespace::dart::ConstMetaSkeletonStateSpacePtr;
using ::dart::dynamics::ConstMetaSkeletonPtr;

//==============================================================================
NominalConfigurationStrategy::NominalConfigurationStrategy(
    ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
    ConstMetaSkeletonPtr metaSkeleton,
    std::size_t numIKSolutions)
  : IKRankingStrategy(metaSkeletonStateSpace, metaSkeleton, numIKSolutions)
{
  // Do nothing
}

//==============================================================================
double NominalConfigurationStrategy::evaluateIKSolution(
    statespace::StateSpace::State* solution) const
{
  DART_UNUSED(solution);
  // TODO (avk): Find the distance from the current state.
  // Use distance metrics and not euclidean.
  return 0;
}

} // namespace dart
} // namespace constraint
} // namespace aikido
