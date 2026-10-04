// SPDX-License-Identifier: Apache-2.0
#include "nav2_imm_ppo_critic/interaction_critic.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace
{
constexpr double kNsToSeconds = 1.0e-9;
constexpr double kTimeTolerance = 1.0e-9;

bool identifier(const std::string & value)
{
  if (value.empty() || value.size() > 128) {
    return false;
  }
  const auto alnum = [](unsigned char c) {
      return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
             (c >= '0' && c <= '9');
    };
  if (!alnum(static_cast<unsigned char>(value.front()))) {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [&](unsigned char c) {
    return alnum(c) || c == '_' || c == '.' || c == ':' || c == '/' || c == '-';
  });
}

bool bounded(double value, double minimum, double maximum)
{
  return std::isfinite(value) && value >= minimum && value <= maximum;
}

bool stampNs(const builtin_interfaces::msg::Time & stamp, int64_t & ns)
{
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000U) {
    return false;
  }
  ns = static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
  return ns > 0;
}

bool knownAction(const std::string & action)
{
  return action == "CRUISE" || action == "AVOID_LEFT" || action == "AVOID_RIGHT" ||
         action == "SLOWDOWN" || action == "WAIT_YIELD";
}

double squaredPreference(double value)
{
  const double clamped = std::clamp(value, 0.0, 1.0);
  return clamped * clamped;
}

struct TubeSample
{
  double x, y, radius;
};
}  // namespace

namespace mppi::critics
{

InteractionCritic::InteractionCritic()
{
  enabled_ = true;
}

void InteractionCritic::initialize()
{
  auto node = parent_.lock();
  if (!node || !parameters_handler_) {
    throw std::runtime_error("InteractionCritic requires a lifecycle parent and parameters");
  }
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(scene_id_, "scene_id", std::string("reset10042_scene1369567760"), ParameterType::Static);
  getParam(frame_id_, "frame_id", std::string("map"), ParameterType::Static);
  getParam(collision_penalty_, "collision_penalty", 100000.0, ParameterType::Static);
  getParam(near_weight_, "near_weight", 100.0, ParameterType::Static);
  getParam(near_distance_, "near_distance", 0.50, ParameterType::Static);
  getParam(preference_weight_, "preference_weight", 10.0, ParameterType::Static);
  getParam(slowdown_speed_, "slowdown_speed", 0.10, ParameterType::Static);
  if (!identifier(scene_id_) || !identifier(frame_id_) ||
    !bounded(collision_penalty_, 100000.0, 1.0e9) ||
    !bounded(near_weight_, 0.0, 1000.0) || !bounded(near_distance_, 0.001, 2.0) ||
    !bounded(preference_weight_, 0.0, 20.0) || !bounded(slowdown_speed_, 0.0, 0.279))
  {
    throw std::runtime_error("InteractionCritic invalid static configuration; no softened fallback");
  }
  clock_ = node->get_clock();
  subscription_ = node->create_subscription<Context>(
    "/simulation/interaction_context", rclcpp::QoS(1).reliable().durability_volatile(),
    [this](Context::ConstSharedPtr message) {
      if (!receiveContext(*message, rosNowNs(), steadyNow())) {
        std::lock_guard<std::mutex> lock(context_mutex_);
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 1000, "InteractionCritic invalid context: %s", last_rejection_.c_str());
      }
    });
}

int64_t InteractionCritic::rosNowNs() const
{
  return clock_ ? clock_->now().nanoseconds() : 0;
}

InteractionCritic::SteadyTime InteractionCritic::steadyNow() const
{
  return SteadyClock::now();
}

bool InteractionCritic::validateMessage(
  const Context & c, int64_t ros_now_ns, int64_t & query_stamp_ns,
  double & query_age_s, std::string & reason) const
{
  const auto reject = [&](const char * why) {reason = why; return false;};
  if (c.schema_version != 1 || c.prediction_semantics != "fused_mean_scalar_uncertainty_v1") {
    return reject("schema_or_prediction_semantics");
  }
  if (!identifier(c.epoch_id) || !identifier(c.scene_id) || !identifier(c.frame_id) ||
    !identifier(c.header.frame_id) || c.scene_id != scene_id_ || c.frame_id != frame_id_ ||
    c.header.frame_id != c.frame_id)
  {
    return reject("scene_or_frame");
  }
  if (!c.sensing_valid) {
    return reject("missing_sensing");
  }
  if (!knownAction(c.action_id)) {
    return reject("unknown_action");
  }
  if (c.packet_seq > kMaxSequence || c.pose_seq > kMaxSequence ||
    c.observation_seq > kMaxSequence)
  {
    return reject("sequence_range");
  }
  if (!bounded(c.source_sim_time_s, 0.0, std::numeric_limits<double>::max()) ||
    !bounded(c.observation_sim_time_s, 0.0, c.source_sim_time_s) ||
    c.source_sim_time_s - c.observation_sim_time_s > kMaxObservationAgeS)
  {
    return reject("observation_time");
  }
  if (!bounded(c.ttl_s, std::numeric_limits<double>::min(), kMaxTtlS) ||
    !std::isfinite(c.controller_horizon_s) || c.controller_horizon_s != kHorizonS ||
    !bounded(c.wait_elapsed_s, 0.0, std::numeric_limits<double>::max()) ||
    !bounded(c.wait_duration_limit_s, std::numeric_limits<double>::min(), kMaxWaitS))
  {
    return reject("ttl_horizon_or_wait_bounds");
  }
  if (!stampNs(c.header.stamp, query_stamp_ns) || ros_now_ns < query_stamp_ns) {
    return reject("future_or_invalid_query_stamp");
  }
  query_age_s = (ros_now_ns - query_stamp_ns) * kNsToSeconds;
  if (query_age_s >= c.ttl_s) {
    return reject("expired_query");
  }
  if (c.source_sim_time_s - c.observation_sim_time_s + query_age_s > kMaxObservationAgeS) {
    return reject("stale_sensing");
  }
  const std::size_t actors = c.actor_id.size();
  if (actors > kMaxActors || c.confidence.size() != actors || c.body_radius_m.size() != actors ||
    c.observation_age_s.size() != actors || c.model_weights.size() != actors * 3 ||
    c.sample_counts.size() != actors)
  {
    return reject("actor_array_bounds");
  }
  std::size_t total = 0;
  for (const auto count : c.sample_counts) {
    if (count < 2 || count > kMaxSamples) {
      return reject("sample_count_bounds");
    }
    total += count;
  }
  if (total > kMaxActors * kMaxSamples || c.prediction_times_s.size() != total ||
    c.trajectory_world_x_m.size() != total || c.trajectory_world_y_m.size() != total ||
    c.uncertainty_radius_m.size() != total)
  {
    return reject("prediction_array_bounds");
  }
  std::unordered_set<std::string> actor_ids;
  std::size_t offset = 0;
  for (std::size_t a = 0; a < actors; ++a) {
    if (!identifier(c.actor_id[a]) || !actor_ids.insert(c.actor_id[a]).second) {
      return reject("actor_id_duplicate_or_invalid");
    }
    if (!bounded(c.confidence[a], 0.0, 1.0) ||
      !bounded(c.body_radius_m[a], std::numeric_limits<double>::min(), 2.0) ||
      !bounded(c.observation_age_s[a], 0.0, kMaxObservationAgeS) ||
      c.observation_age_s[a] + kTimeTolerance < c.source_sim_time_s - c.observation_sim_time_s ||
      c.observation_age_s[a] + query_age_s > kMaxObservationAgeS)
    {
      return reject("confidence_radius_or_track_age");
    }
    double weight_sum = 0.0;
    for (std::size_t model = 0; model < 3; ++model) {
      const double weight = c.model_weights[3 * a + model];
      if (!bounded(weight, 0.0, 1.0)) {
        return reject("model_weight_bounds");
      }
      weight_sum += weight;
    }
    if (std::abs(weight_sum - 1.0) > 1.0e-6) {
      return reject("model_weight_normalization");
    }
    const std::size_t count = c.sample_counts[a];
    for (std::size_t j = 0; j < count; ++j) {
      const auto i = offset + j;
      if (!bounded(c.prediction_times_s[i], 0.0, std::numeric_limits<double>::max()) ||
        (j == 0 && c.prediction_times_s[i] != 0.0) ||
        (j > 0 && c.prediction_times_s[i] <= c.prediction_times_s[i - 1]) ||
        !std::isfinite(c.trajectory_world_x_m[i]) || !std::isfinite(c.trajectory_world_y_m[i]) ||
        !bounded(c.uncertainty_radius_m[i], 0.0, 10.0))
      {
        return reject("prediction_samples_nonfinite_or_unordered");
      }
    }
    if (c.prediction_times_s[offset + count - 1] < kCoverageS ||
      c.prediction_times_s[offset + count - 1] < query_age_s + kHorizonS)
    {
      return reject("insufficient_prediction_horizon");
    }
    offset += count;
  }
  return true;
}

bool InteractionCritic::receiveContext(
  const Context & c, int64_t ros_now_ns, SteadyTime receipt)
{
  std::lock_guard<std::mutex> lock(context_mutex_);
  // Rejection invalidates usable data, but never consumes watermarks, epochs,
  // observation provenance or WAIT budget. Acceptance is one transaction.
  const auto reject = [&](const std::string & reason) {
      snapshot_.reset(); last_rejection_ = reason; return false;
    };
  int64_t stamp = 0;
  double age = 0.0;
  std::string reason;
  if (!validateMessage(c, ros_now_ns, stamp, age, reason)) {
    return reject(reason);
  }
  const bool new_epoch = !have_sequence_ || c.epoch_id != epoch_id_;
  if (have_sequence_ && (c.pose_seq <= last_pose_seq_ || stamp <= last_query_stamp_ns_ ||
    ros_now_ns < last_receipt_ros_ns_))
  {
    return reject("global_pose_or_ros_time_replay");
  }
  if (new_epoch && (seen_epochs_.count(c.epoch_id) || seen_epochs_.size() >= kMaxEpochs)) {
    return reject("epoch_replay_or_history_capacity");
  }
  std::unordered_map<std::string, double> actor_observation_times;
  for (std::size_t i = 0; i < c.actor_id.size(); ++i) {
    actor_observation_times.emplace(c.actor_id[i], c.source_sim_time_s - c.observation_age_s[i]);
  }
  if (!new_epoch) {
    if (c.packet_seq <= last_packet_seq_ || c.source_sim_time_s < last_source_sim_s_ ||
      c.observation_seq < last_observation_seq_ || c.observation_sim_time_s < last_observation_sim_s_)
    {
      return reject("packet_or_simulation_time_replay");
    }
    const bool same_observation = c.observation_seq == last_observation_seq_;
    if ((same_observation && c.observation_sim_time_s != last_observation_sim_s_) ||
      (!same_observation && c.observation_sim_time_s <= last_observation_sim_s_) ||
      (same_observation && actor_observation_times.size() != last_actor_observation_times_.size()))
    {
      return reject("observation_restamped_or_replayed");
    }
    for (const auto & observed : actor_observation_times) {
      auto previous = last_actor_observation_times_.find(observed.first);
      if ((same_observation && (previous == last_actor_observation_times_.end() ||
        std::abs(observed.second - previous->second) > kTimeTolerance)) ||
        (previous != last_actor_observation_times_.end() &&
        observed.second + kTimeTolerance < previous->second))
      {
        return reject("track_observation_restamped_or_replayed");
      }
    }
  }

  auto started = new_epoch ? std::optional<double>{} : wait_started_sim_s_;
  auto limit = new_epoch ? std::optional<double>{} : wait_limit_s_;
  auto deadline = new_epoch ? std::optional<SteadyTime>{} : wait_deadline_steady_;
  if (c.action_id == "WAIT_YIELD") {
    const double declared_start = c.source_sim_time_s - c.wait_elapsed_s;
    if (!std::isfinite(declared_start)) {
      return reject("wait_ledger_overflow");
    }
    const bool continuing = !new_epoch && last_requested_action_ == "WAIT_YIELD" && started;
    started = continuing ? std::min(*started, declared_start) : declared_start;
    limit = continuing ? std::min(*limit, c.wait_duration_limit_s) : c.wait_duration_limit_s;
    const double elapsed = std::max(c.wait_elapsed_s, c.source_sim_time_s - *started);
    // The durations are independent watchdogs. Neither ROS nor steady stamps
    // are treated as a simulation timestamp. Transport consumes remaining TTL
    // and conservatively consumes remaining WAIT duration, never renews it.
    const double remaining = std::max(0.0, *limit - elapsed - age);
    const auto proposed_deadline = receipt + std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(remaining));
    deadline = continuing ? std::min(*deadline, proposed_deadline) : proposed_deadline;
  } else {
    started.reset(); limit.reset(); deadline.reset();
  }
  auto next = std::make_shared<Snapshot>();
  next->context = c;
  next->query_stamp_ns = stamp;
  next->receipt_ros_ns = ros_now_ns;
  next->receipt_query_age_s = age;
  next->receipt_steady = receipt;
  if (started) {
    next->wait_started_sim_s = *started;
    next->wait_limit_s = *limit;
    next->wait_deadline_steady = *deadline;
  }
  if (new_epoch) {
    seen_epochs_.insert(c.epoch_id);
  }
  epoch_id_ = c.epoch_id;
  have_sequence_ = true;
  last_packet_seq_ = c.packet_seq;
  last_pose_seq_ = c.pose_seq;
  last_observation_seq_ = c.observation_seq;
  last_query_stamp_ns_ = stamp;
  last_receipt_ros_ns_ = ros_now_ns;
  last_source_sim_s_ = c.source_sim_time_s;
  last_observation_sim_s_ = c.observation_sim_time_s;
  last_actor_observation_times_ = std::move(actor_observation_times);
  last_requested_action_ = c.action_id;
  wait_started_sim_s_ = started;
  wait_limit_s_ = limit;
  wait_deadline_steady_ = deadline;
  snapshot_ = std::move(next);
  last_rejection_.clear();
  return true;
}

bool InteractionCritic::fresh(
  const Snapshot & snapshot, int64_t ros_now_ns, SteadyTime now,
  double & query_age_s, std::string & reason) const
{
  if (ros_now_ns < snapshot.receipt_ros_ns || ros_now_ns < snapshot.query_stamp_ns ||
    now < snapshot.receipt_steady)
  {
    reason = "receipt_clock_reversed";
    return false;
  }
  const double steady_age = std::chrono::duration<double>(now - snapshot.receipt_steady).count();
  query_age_s = std::max(
    (ros_now_ns - snapshot.query_stamp_ns) * kNsToSeconds,
    snapshot.receipt_query_age_s + steady_age);
  const auto & c = snapshot.context;
  if (!std::isfinite(query_age_s) || query_age_s >= c.ttl_s) {
    reason = "expired_snapshot";
    return false;
  }
  if (c.source_sim_time_s - c.observation_sim_time_s + query_age_s > kMaxObservationAgeS) {
    reason = "stale_sensing";
    return false;
  }
  std::size_t offset = 0;
  for (std::size_t a = 0; a < c.actor_id.size(); ++a) {
    offset += c.sample_counts[a];
    if (c.observation_age_s[a] + query_age_s > kMaxObservationAgeS ||
      c.prediction_times_s[offset - 1] < query_age_s + kHorizonS)
    {
      reason = "stale_track_or_insufficient_horizon";
      return false;
    }
  }
  return true;
}

void InteractionCritic::invalidate(const std::string & reason, const Snapshot * expected)
{
  std::lock_guard<std::mutex> lock(context_mutex_);
  if (!expected || snapshot_.get() == expected) {
    snapshot_.reset();
    last_rejection_ = reason;
  }
}

void InteractionCritic::score(CriticData & data)
{
  if (data.fail_flag) {
    return;
  }
  std::shared_ptr<const Snapshot> snapshot;
  {
    std::lock_guard<std::mutex> lock(context_mutex_);
    snapshot = snapshot_;
  }
  const auto fail = [&](const std::string & reason) {
      invalidate(reason, snapshot.get()); data.fail_flag = true;
    };
  if (!enabled_ || !snapshot) {
    fail(enabled_ ? "no_context" : "mandatory_critic_disabled");
    return;
  }
  const int64_t ros_now = rosNowNs();
  const auto steady_now = steadyNow();
  double query_age = 0.0;
  std::string reason;
  if (!fresh(*snapshot, ros_now, steady_now, query_age, reason)) {
    fail(reason); return;
  }
  const auto & c = snapshot->context;
  const auto & pose = data.state.pose;
  int64_t pose_stamp = 0;
  // The bridge independently binds packet pose_seq to this query ROS stamp.
  // Nav2 may already have a newer pose at score time; it must still be fresh in
  // the same frame, never older than the context. No extra input is subscribed.
  if (pose.header.frame_id != frame_id_ || !stampNs(pose.header.stamp, pose_stamp) ||
    pose_stamp < snapshot->query_stamp_ns || pose_stamp > ros_now ||
    (ros_now - pose_stamp) * kNsToSeconds >= c.ttl_s)
  {
    fail("score_pose_frame_or_stamp"); return;
  }
  const auto & p = pose.pose.position;
  const auto & q = pose.pose.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
    !std::isfinite(norm) || std::abs(norm - 1.0) > 1.0e-3)
  {
    fail("score_pose_nonfinite_or_quaternion"); return;
  }
  const auto & tr = data.trajectories;
  const auto batch = tr.x.shape()[0];
  const auto steps = tr.x.shape()[1];
  if (batch == 0 || batch > kMaxBatch || steps == 0 || steps > kMaxSteps ||
    tr.y.shape() != tr.x.shape() || tr.yaws.shape() != tr.x.shape() ||
    data.state.vx.shape() != tr.x.shape() || data.state.vy.shape() != tr.x.shape() ||
    data.state.wz.shape() != tr.x.shape() || data.costs.size() != batch ||
    !std::isfinite(data.model_dt) || data.model_dt <= 0.0F ||
    std::abs(steps * static_cast<double>(data.model_dt) - kHorizonS) > 1.0e-5)
  {
    fail("candidate_shape_or_controller_horizon"); return;
  }
  // Validate before mutating costs, including a neutral empty scene.
  for (std::size_t b = 0; b < batch; ++b) {
    if (!std::isfinite(data.costs(b))) {
      fail("existing_cost_nonfinite"); return;
    }
    for (std::size_t t = 0; t < steps; ++t) {
      if (!std::isfinite(tr.x(b, t)) || !std::isfinite(tr.y(b, t)) ||
        !std::isfinite(tr.yaws(b, t)) || !std::isfinite(data.state.vx(b, t)) ||
        !std::isfinite(data.state.vy(b, t)) || !std::isfinite(data.state.wz(b, t)))
      {
        fail("candidate_nonfinite"); return;
      }
    }
  }
  std::string effective_action = c.action_id;
  if (c.action_id == "WAIT_YIELD" && (!c.waitable ||
    c.source_sim_time_s - snapshot->wait_started_sim_s + query_age >= snapshot->wait_limit_s ||
    steady_now >= snapshot->wait_deadline_steady))
  {
    // Neutral preference only, NOT a command to advance. Human/static/Unknown
    // costs and external hard guards remain in force. Requested WAIT persists
    // in the receipt ledger, even when the effective action is CRUISE.
    effective_action = "CRUISE";
  }

  double tangent_x = std::cos(std::atan2(
      2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)));
  double tangent_y = std::sin(std::atan2(
      2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)));
  if (effective_action == "AVOID_LEFT" || effective_action == "AVOID_RIGHT") {
    // Left is the positive cross product of the nearest global-path segment's
    // forward tangent and displacement from the robot. Degenerate/empty path
    // falls back to robot heading; actor coordinates are never a side basis.
    if (data.path.x.size() != data.path.y.size()) {
      fail("path_shape"); return;
    }
    double nearest = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < data.path.x.size(); ++i) {
      if (!std::isfinite(data.path.x(i)) || !std::isfinite(data.path.y(i))) {
        fail("path_nonfinite"); return;
      }
      if (i == 0) {continue;}
      const double x0 = data.path.x(i - 1), y0 = data.path.y(i - 1);
      const double dx = data.path.x(i) - x0, dy = data.path.y(i) - y0;
      const double length = std::hypot(dx, dy);
      if (length <= 1.0e-6) {continue;}
      const double projection = std::clamp(
        ((p.x - x0) * (dx / length) + (p.y - y0) * (dy / length)) / length, 0.0, 1.0);
      const double distance = std::hypot(p.x - x0 - projection * dx, p.y - y0 - projection * dy);
      if (!std::isfinite(distance)) {fail("path_arithmetic_overflow"); return;}
      if (distance < nearest) {
        nearest = distance; tangent_x = dx / length; tangent_y = dy / length;
      }
    }
  }

  std::array<std::array<TubeSample, kMaxSteps>, kMaxActors> tubes{};
  std::size_t offset = 0;
  for (std::size_t a = 0; a < c.actor_id.size(); ++a) {
    const std::size_t count = c.sample_counts[a];
    std::size_t j = 1;
    for (std::size_t t = 0; t < steps; ++t) {
      // Forecasts already start at the tracker query, not last_update. Adding
      // observation_age_s here would shift crossing humans a second time.
      const double time = query_age + (t + 1) * static_cast<double>(data.model_dt);
      while (j < count && c.prediction_times_s[offset + j] < time) {++j;}
      if (j == count) {fail("forecast_out_of_coverage"); return;}
      const auto lo = offset + j - 1, hi = offset + j;
      const double ratio = (time - c.prediction_times_s[lo]) /
        (c.prediction_times_s[hi] - c.prediction_times_s[lo]);
      const auto interpolate = [&](const auto & values) {
          return (1.0 - ratio) * values[lo] + ratio * values[hi];
        };
      auto & tube = tubes[a][t];
      tube.x = interpolate(c.trajectory_world_x_m);
      tube.y = interpolate(c.trajectory_world_y_m);
      tube.radius = kRobotRadiusM + c.body_radius_m[a] + interpolate(c.uncertainty_radius_m);
      if (!std::isfinite(tube.x) || !std::isfinite(tube.y) || !std::isfinite(tube.radius)) {
        fail("forecast_arithmetic_overflow"); return;
      }
    }
    offset += count;
  }
  std::vector<float> scored(batch);
  for (std::size_t b = 0; b < batch; ++b) {
    double extra = 0.0;
    for (std::size_t a = 0; a < c.actor_id.size(); ++a) {
      bool collision = false;
      double near = 0.0;
      for (std::size_t t = 0; t < steps; ++t) {
        const auto & tube = tubes[a][t];
        const double dx = static_cast<double>(tr.x(b, t)) - tube.x;
        const double dy = static_cast<double>(tr.y(b, t)) - tube.y;
        const double distance_squared = dx * dx + dy * dy;
        if (!std::isfinite(distance_squared)) {fail("distance_arithmetic_overflow"); return;}
        const double outer = tube.radius + near_distance_;
        if (distance_squared > outer * outer) {continue;}
        collision = collision || distance_squared <= tube.radius * tube.radius;
        const double clearance = std::sqrt(distance_squared) - tube.radius;
        near += near_weight_ * squaredPreference(1.0 - clearance / near_distance_);
      }
      // Confidence and mode weights are diagnostic ONLY. A zero-confidence
      // perceived track still has its full tube and collision penalty.
      extra += (collision ? collision_penalty_ : 0.0) + near / steps;
    }
    double preference = 0.0;
    for (std::size_t t = 0; t < steps; ++t) {
      if (effective_action == "AVOID_LEFT" || effective_action == "AVOID_RIGHT") {
        const double lateral = tangent_x * (tr.y(b, t) - p.y) - tangent_y * (tr.x(b, t) - p.x);
        const double wrong_side = effective_action == "AVOID_LEFT" ? -lateral : lateral;
        preference += squaredPreference(wrong_side / 0.50);
      } else if (effective_action == "SLOWDOWN") {
        preference += squaredPreference(
          (std::abs(data.state.vx(b, t)) - slowdown_speed_) / (0.28 - slowdown_speed_));
      } else if (effective_action == "WAIT_YIELD") {
        preference += squaredPreference(
          std::hypot(data.state.vx(b, t), data.state.vy(b, t)) / 0.28 +
          std::abs(data.state.wz(b, t)) / 0.70);
      }
    }
    extra += preference_weight_ * preference / steps;
    const double total = static_cast<double>(data.costs(b)) + extra;
    if (!std::isfinite(extra) || extra < 0.0 || !std::isfinite(total) ||
      total > std::numeric_limits<float>::max())
    {
      fail("cost_arithmetic_overflow"); return;
    }
    scored[b] = static_cast<float>(total);
  }
  // Do not commit an expired result or a snapshot superseded/invalidated while
  // computing. Costs are changed only after the complete scoring transaction.
  std::lock_guard<std::mutex> lock(context_mutex_);
  if (snapshot_.get() != snapshot.get()) {
    data.fail_flag = true;
    return;
  }
  double end_age = 0.0;
  if (!fresh(*snapshot, rosNowNs(), steadyNow(), end_age, reason)) {
    snapshot_.reset(); last_rejection_ = reason; data.fail_flag = true;
    return;
  }
  for (std::size_t b = 0; b < batch; ++b) {
    // Even signed zero remains bit-for-bit unchanged for CRUISE + empty tracks.
    if (c.actor_id.empty() && effective_action == "CRUISE") {continue;}
    data.costs(b) = scored[b];
  }
}

}  // namespace mppi::critics

PLUGINLIB_EXPORT_CLASS(mppi::critics::InteractionCritic, mppi::critics::CriticFunction)
