// SPDX-License-Identifier: Apache-2.0
#ifndef NAV2_IMM_PPO_CRITIC__INTERACTION_CRITIC_HPP_
#define NAV2_IMM_PPO_CRITIC__INTERACTION_CRITIC_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_imm_ppo_critic/msg/interaction_context.hpp"

namespace mppi::critics
{

// Simulation only. Adds nonnegative costs; never publishes commands or replaces
// stock static/Unknown critics. Finite penalties are not collision-free proof.
class InteractionCritic : public CriticFunction
{
public:
  using Context = nav2_imm_ppo_critic::msg::InteractionContext;
  using SteadyClock = std::chrono::steady_clock;
  using SteadyTime = SteadyClock::time_point;

  InteractionCritic();
  void initialize() override;
  void score(CriticData & data) override;

protected:
  // Protected injection points support node-free gtests of the actual score()
  // and receive path; production uses the lifecycle parent's ROS clock and a
  // real steady clock. No alternate scoring implementation is used by tests.
  virtual int64_t rosNowNs() const;
  virtual SteadyTime steadyNow() const;
  bool receiveContext(const Context & context, int64_t ros_now_ns, SteadyTime receipt);

  struct Snapshot
  {
    Context context;
    int64_t query_stamp_ns{0};
    int64_t receipt_ros_ns{0};
    double receipt_query_age_s{0.0};
    SteadyTime receipt_steady;
    double wait_started_sim_s{0.0};
    double wait_limit_s{0.0};
    SteadyTime wait_deadline_steady;
  };

  bool validateMessage(
    const Context & context, int64_t ros_now_ns, int64_t & query_stamp_ns,
    double & query_age_s, std::string & reason) const;
  bool fresh(
    const Snapshot & snapshot, int64_t ros_now_ns, SteadyTime now,
    double & query_age_s, std::string & reason) const;
  void invalidate(const std::string & reason, const Snapshot * expected = nullptr);

  static constexpr std::size_t kMaxActors = 8;
  static constexpr std::size_t kMaxSamples = 64;
  static constexpr std::size_t kMaxBatch = 1000;
  static constexpr std::size_t kMaxSteps = 30;
  static constexpr std::size_t kMaxEpochs = 4096;
  static constexpr double kRobotRadiusM = 0.49;
  static constexpr double kHorizonS = 3.0;
  static constexpr double kCoverageS = 3.5;
  static constexpr double kMaxObservationAgeS = 0.5;
  static constexpr double kMaxTtlS = 0.30;
  static constexpr double kMaxWaitS = 1.8;
  static constexpr uint64_t kMaxSequence = 9007199254740991ULL;

  std::string scene_id_{"reset10042_scene1369567760"};
  std::string frame_id_{"map"};
  double collision_penalty_{100000.0};
  double near_weight_{100.0};
  double near_distance_{0.50};
  double preference_weight_{10.0};
  double slowdown_speed_{0.10};
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Subscription<Context>::SharedPtr subscription_;

  mutable std::mutex context_mutex_;
  std::shared_ptr<const Snapshot> snapshot_;
  std::string last_rejection_{"no_context"};
  bool have_sequence_{false};
  std::string epoch_id_;
  std::unordered_set<std::string> seen_epochs_;
  uint64_t last_packet_seq_{0}, last_pose_seq_{0}, last_observation_seq_{0};
  int64_t last_query_stamp_ns_{0}, last_receipt_ros_ns_{0};
  double last_source_sim_s_{0.0}, last_observation_sim_s_{0.0};
  std::unordered_map<std::string, double> last_actor_observation_times_;

  // Sender simulation wait ledger and an independent steady safety deadline.
  // Their clocks stay separate. Only a requested non-WAIT or new bridge-verified
  // epoch ends the run; a CRUISE *effective fallback* does not replenish it.
  std::string last_requested_action_;
  std::optional<double> wait_started_sim_s_, wait_limit_s_;
  std::optional<SteadyTime> wait_deadline_steady_;
};

}  // namespace mppi::critics
#endif  // NAV2_IMM_PPO_CRITIC__INTERACTION_CRITIC_HPP_
