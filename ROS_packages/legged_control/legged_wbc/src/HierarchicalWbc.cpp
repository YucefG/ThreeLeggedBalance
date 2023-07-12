//
// Created by qiayuan on 22-12-23.
//

#include "legged_wbc/HierarchicalWbc.h"

#include "legged_wbc/HoQp.h"

namespace legged {
#ifdef WBC_CONTROL
vector_t HierarchicalWbc::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                                 scalar_t period, const vector3_t& effector_position) {
  WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period, effector_position);

  Task task0 = formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask() + formulateNoContactMotionTask();
  Task task1 = formulateBaseAccelTask(stateDesired, inputDesired, period) + formulateSwingLegTask(effector_position);
  Task task2 = formulateContactForceTask(inputDesired);
  HoQp hoQp(task2, std::make_shared<HoQp>(task1, std::make_shared<HoQp>(task0)));

  // Tested in experiments: Modified WBC to increase weight on swinging leg tracking
  /*
  Task task0 = formulateTorqueLimitsTask() + formulateFrictionConeTask() + formulateNoContactMotionTask()+ formulateSwingLegTask(effector_position);
  Task task1 = formulateBaseAccelTask(stateDesired, inputDesired, period) ;
  Task task2 = formulateContactForceTask(inputDesired)+formulateFloatingBaseEomTask();
  HoQp hoQp(task2, std::make_shared<HoQp>(task1, std::make_shared<HoQp>(task0)));
  */

  return hoQp.getSolutions();
}
#else
vector_t HierarchicalWbc::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                                 scalar_t period) {
  WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period);

  Task task0 = formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask() + formulateNoContactMotionTask();
  Task task1 = formulateBaseAccelTask(stateDesired, inputDesired, period) + formulateSwingLegTask();
  Task task2 = formulateContactForceTask(inputDesired);
  HoQp hoQp(task2, std::make_shared<HoQp>(task1, std::make_shared<HoQp>(task0)));

  return hoQp.getSolutions();
}
#endif

}  // namespace legged

