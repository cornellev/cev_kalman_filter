#pragma once

#include "estimator.h"

class Updateable {
  public:
    /**
     * Update the state of the model with an estimate
     * 
     * @param estimate Estimate to update with
     */
    virtual void estimate_update(Estimator estimate) = 0;
};