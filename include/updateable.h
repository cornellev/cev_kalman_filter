#pragma once

#include "estimator.h"

class Updateable {
  public:
    virtual void estimate_update(Estimator estimate) = 0;
};