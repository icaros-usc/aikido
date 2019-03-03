#include "aikido/distance/NominalConfigurationRanker.hpp"

namespace aikido {
namespace distance {

using statespace::dart::ConstMetaSkeletonStateSpacePtr;
using ::dart::dynamics::ConstMetaSkeletonPtr;

// //==============================================================================
// NominalConfigurationRanker::NominalConfigurationRanker(
//     ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
//     ConstMetaSkeletonPtr metaSkeleton,
//     std::vector<double> weights,
//     const statespace::CartesianProduct::State* nominalConfiguration)
//   : ConfigurationRanker(
//         std::move(metaSkeletonStateSpace), std::move(metaSkeleton), weights)
//   , mNominalConfiguration(nominalConfiguration)
// {
//   if (!mNominalConfiguration)
//     mNominalConfiguration
//         = mMetaSkeletonStateSpace->getScopedStateFromMetaSkeleton(
//             mMetaSkeleton.get());
// }

//==============================================================================
NominalConfigurationRanker::NominalConfigurationRanker(
      statespace::dart::ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
      ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton,
      std::vector<double> weights,
      statespace::CartesianProduct::ScopedState nominalConfiguration)
: ConfigurationRanker(
        std::move(metaSkeletonStateSpace), std::move(metaSkeleton), weights)
  , mNominalConfiguration(std::move(nominalConfiguration))
{
  // do nothing
}

//==============================================================================
double NominalConfigurationRanker::evaluateConfiguration(
    const statespace::dart::MetaSkeletonStateSpace::State* solution) const
{
  return mDistanceMetric->distance(solution, mNominalConfiguration);
}

} // namespace distance
} // namespace aikido
