//
// Created by qiayuan on 22-12-23.
//
#pragma once

#include "legged_wbc/WbcBase.h"

namespace legged {

class HierarchicalWbc : public WbcBase {
 public:
  using WbcBase::WbcBase;

//  vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
//                  scalar_t period) override;
//new added xxx
#ifdef WBC_CONTROL
  vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                  scalar_t period, const vector3_t& effector_position) override;
#else
  vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                  scalar_t period) override;
#endif
};



}  // namespace legged
