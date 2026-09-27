#ifndef QCAR_GAZEBO__CRITICS__EARLY_COMMIT_CRITIC_HPP_
#define QCAR_GAZEBO__CRITICS__EARLY_COMMIT_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

class EarlyCommitCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;

protected:
  size_t offset_from_furthest_{3};
  size_t early_time_steps_{10};
  float max_lead_distance_{0.3f};
  bool forward_preference_{false};

  unsigned int power_{1};
  float weight_{0};
};

}  // namespace mppi::critics

#endif  // QCAR_GAZEBO__CRITICS__EARLY_COMMIT_CRITIC_HPP_
