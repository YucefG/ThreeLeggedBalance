//
// Created by qiayuan on 22-12-23.
//

#include "legged_wbc/WbcBase.h"

namespace legged {

class WeightedWbc : public WbcBase {
 public:
  using WbcBase::WbcBase;


  #ifdef WBC_CONTROL
  vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                  scalar_t period, const vector3_t& effector_position) override;
  #else
  vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
               scalar_t period) override;
  #endif
  void loadTasksSetting(const std::string& taskFile, bool verbose) override;

 protected:
  virtual Task formulateConstraints();
  #ifdef WBC_CONTROL
  virtual Task formulateWeightedTasks(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period, const vector3_t& effector_position);
  #else
  virtual Task formulateWeightedTasks(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period);
  #endif
 private:
  scalar_t weightSwingLeg_, weightBaseAccel_, weightContactForce_;
};

}  // namespace legged