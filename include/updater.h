#pragma once

#include <vector>
#include <memory>
#include "updateable.h"

using Listener = std::shared_ptr<Updateable>;

class Updater : public Estimator {
  protected:
    std::vector<Listener> models;

  public:
    Updater(
      V state, 
      M covariance,
      std::vector<Listener> dependents = {}
    );

    /**
     * Bind a model to this updater
     * 
     * @param model Model to bind
     */
    void bind_to(Listener model);

    /**
     * Update all bound models
     */
    void update_dependents();
};