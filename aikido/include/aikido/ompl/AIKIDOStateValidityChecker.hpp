#ifndef AIKIDO_STATE_VALIDITY_CHECKER_H
#define AIKIDO_STATE_VALIDITY_CHECKER_H

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/SpaceInformation.h>
#include <aikido/statespace/StateSpace.hpp>
#include <aikido/constraint/TestableConstraint.hpp>

namespace aikido {
    namespace ompl_bindings {

        class AIKIDOStateValidityChecker : public ompl::base::StateValidityChecker {

        public:
            
            AIKIDOStateValidityChecker(const ompl::base::SpaceInformationPtr &_si,
                                     std::vector<std::shared_ptr<
                                     aikido::constraint::TestableConstraint> > _constraints);
            
            /// Return true if the state state is valid. Usually, this means at 
            /// least collision checking and bounds checking
            virtual bool isValid(const ompl::base::State* _state) const;

        private:
            std::vector<std::shared_ptr<aikido::constraint::TestableConstraint> > mConstraints;

        };
    }
}
#endif