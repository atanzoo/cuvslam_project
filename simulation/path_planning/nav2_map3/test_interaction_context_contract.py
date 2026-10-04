"""Focused tests for the versioned simulation interaction-context contract."""

from __future__ import annotations

import copy
import unittest
from typing import Any, Dict, List, Optional, Union

from interaction_context_contract import (
    ACTION_IDS,
    CONTROLLER_HORIZON_S,
    INTERACTION_CONTEXT_SCHEMA,
    MAX_ACTORS,
    MAX_PREDICTION_SAMPLES,
    ContextValidationError,
    InteractionContextFreshnessGuard,
    build_interaction_context_packet,
    parse_interaction_context,
    resolve_action_preference,
    serialize_interaction_context,
    validate_interaction_context,
)


class SyntheticTrack:
    """Perceived tracker output used without any simulated actor truth."""

    def __init__(self, track_id: int, confidence: int = 6, last_update: float = 10.0):
        self.track_id = track_id
        self.confidence = confidence
        self.last_update = last_update


class TruthTripwire:
    def __getattribute__(self, name: str) -> object:
        raise AssertionError(f"serializer touched forbidden truth field {name!r}")


class RestrictedPerceivedTrack:
    """Fail if serialization reads anything beyond tracker-owned fields."""

    __slots__ = ("track_id", "confidence", "last_update")
    _allowed = frozenset({"track_id", "confidence", "last_update", "__class__"})

    def __init__(self, track_id: int, confidence: int, last_update: float):
        object.__setattr__(self, "track_id", track_id)
        object.__setattr__(self, "confidence", confidence)
        object.__setattr__(self, "last_update", last_update)

    def __getattribute__(self, name: str) -> object:
        if name not in object.__getattribute__(self, "_allowed"):
            raise AssertionError(f"unexpected perceived-track access: {name!r}")
        return object.__getattribute__(self, name)


def _actor(
    *,
    actor_id: str = "track-7",
    confidence: float = 0.75,
    body_radius_m: float = 0.35,
    observation_age_s: float = 0.0,
    end_time_s: float = 3.5,
) -> Dict[str, Any]:
    count = int(round(end_time_s / 0.5)) + 1
    times = [min(index * 0.5, end_time_s) for index in range(count)]
    if times[-1] != end_time_s:
        times.append(end_time_s)
    return {
        "actor_id": actor_id,
        "confidence": confidence,
        "body_radius_m": body_radius_m,
        "observation_age_s": observation_age_s,
        "model_weights": {"CV": 0.50, "CA": 0.30, "CTRV": 0.20},
        "prediction_times_s": times,
        "trajectory_world_xy_m": [[1.0 + index * 0.1, -0.25] for index in range(len(times))],
        "uncertainty_radius_m": [0.10 for _ in times],
    }


def _context(**overrides: Any) -> Dict[str, Any]:
    packet: Dict[str, Any] = {
        "schema_version": 1,
        "epoch_id": "episode-1",
        "scene_id": "map3-static-dynamic-v1",
        "frame_id": "map",
        "packet_seq": 1,
        "pose_seq": 41,
        "observation_seq": 1,
        "source_sim_time_s": 10.0,
        "observation_sim_time_s": 10.0,
        "ttl_s": 0.30,
        "sensing_valid": True,
        "controller_horizon_s": CONTROLLER_HORIZON_S,
        "prediction_semantics": "fused_mean_scalar_uncertainty_v1",
        "action_id": "CRUISE",
        "waitable": False,
        "wait_elapsed_s": 0.0,
        "wait_duration_limit_s": 1.8,
        "actors": [_actor()],
    }
    packet.update(overrides)
    if "actors" not in overrides:
        packet["actors"] = [_actor(observation_age_s=(
            packet["source_sim_time_s"] - packet["observation_sim_time_s"]
        ))]
    return packet


def _prediction(
    track_id: int = 7,
    *,
    confidence: int = 6,
    last_update: float = 10.0,
    body_radius_m: float = 0.35,
) -> Dict[str, Any]:
    return {
        "track": SyntheticTrack(track_id, confidence, last_update),
        "body_radius_m": body_radius_m,
        "trajectory_world": [[1.0 + index * 0.1, -0.25] for index in range(8)],
        "trajectory_uncertainty_world": [0.10] * 8,
        "model_weights": {"CV": 0.50, "CA": 0.30, "CTRV": 0.20},
        "prediction_times": [index * 0.5 for index in range(8)],
        # The serializer must not inspect this or any simulation truth state.
        "truth": TruthTripwire(),
    }


def _build_packet(
    *,
    action_id: Union[str, int] = "CRUISE",
    predictions: Optional[List[Dict[str, Any]]] = None,
    **overrides: Any
) -> Dict[str, Any]:
    args: Dict[str, Any] = {
        "epoch_id": "episode-1",
        "scene_id": "map3-static-dynamic-v1",
        "frame_id": "map",
        "packet_seq": 1,
        "pose_seq": 41,
        "observation_seq": 1,
        "source_sim_time_s": 10.0,
        "observation_sim_time_s": 10.0,
        "ttl_s": 0.30,
        "sensing_valid": True,
        "action_id": action_id,
        "waitable": action_id in ("WAIT_YIELD", 4),
        "wait_elapsed_s": 0.0,
        "wait_duration_limit_s": 1.8,
        "tracker_predictions": [] if predictions is None else predictions,
        "body_radius_by_track_id": {},
    }
    args.update(overrides)
    return build_interaction_context_packet(**args)  # type: ignore[arg-type]


def _guard() -> InteractionContextFreshnessGuard:
    guard = InteractionContextFreshnessGuard()
    guard.reset_epoch("episode-1", disarmed=True)
    return guard


class InteractionContextContractTests(unittest.TestCase):
    def test_schema_is_inspectable_and_closes_unknown_fields(self) -> None:
        self.assertEqual(INTERACTION_CONTEXT_SCHEMA["properties"]["schema_version"]["const"], 1)
        self.assertFalse(INTERACTION_CONTEXT_SCHEMA["additionalProperties"])
        self.assertIn("actors", INTERACTION_CONTEXT_SCHEMA["required"])

    def test_all_five_action_ids_serialize_and_validate(self) -> None:
        for index, name in enumerate(ACTION_IDS):
            with self.subTest(action=name):
                packet = _build_packet(action_id=index)
                self.assertEqual(packet["action_id"], name)
                self.assertEqual(validate_interaction_context(packet), packet)

    def test_fresh_zero_track_packet_round_trips(self) -> None:
        packet = _build_packet(predictions=[])
        self.assertEqual(packet["actors"], [])
        encoded = serialize_interaction_context(packet)
        decoded = parse_interaction_context(encoded, now_sim_time_s=10.1)
        self.assertEqual(decoded["actors"], [])
        self.assertTrue(decoded["sensing_valid"])

    def test_tracker_prediction_fields_are_serialized_without_truth_access(self) -> None:
        restricted = _prediction()
        restricted["track"] = RestrictedPerceivedTrack(9, 4, 9.90)
        packet = _build_packet(
            predictions=[restricted],
            source_sim_time_s=10.0,
            observation_sim_time_s=10.0,
        )
        actor = packet["actors"][0]
        self.assertEqual(actor["actor_id"], "9")
        self.assertEqual(actor["confidence"], 0.5)
        self.assertAlmostEqual(actor["observation_age_s"], 0.1)
        self.assertEqual(len(actor["prediction_times_s"]), 8)

    def test_duplicate_actor_ids_are_rejected(self) -> None:
        with self.assertRaisesRegex(ContextValidationError, "duplicate actor_id"):
            _build_packet(predictions=[_prediction(7), _prediction(7)])

    def test_scene_and_frame_identity_are_checked(self) -> None:
        packet = _context()
        with self.assertRaisesRegex(ContextValidationError, "scene_id mismatch"):
            validate_interaction_context(packet, expected_scene_id="other-scene")
        with self.assertRaisesRegex(ContextValidationError, "frame_id mismatch"):
            validate_interaction_context(packet, expected_frame_id="odom")
        for field, value in (("scene_id", ""), ("frame_id", "map frame")):
            with self.subTest(field=field), self.assertRaises(ContextValidationError):
                validate_interaction_context(_context(**{field: value}))

    def test_missing_or_invalid_sensing_marker_is_not_empty_tracks(self) -> None:
        with self.assertRaisesRegex(ContextValidationError, "sensing_valid must be true"):
            validate_interaction_context(_context(sensing_valid=False, actors=[]))
        missing = _context(actors=[])
        del missing["sensing_valid"]
        with self.assertRaisesRegex(ContextValidationError, "missing required field.*sensing_valid"):
            validate_interaction_context(missing)

    def test_stale_global_sensing_and_track_age_are_rejected(self) -> None:
        with self.assertRaisesRegex(ContextValidationError, "observation age"):
            validate_interaction_context(_context(observation_sim_time_s=9.49))
        stale_actor = _actor(observation_age_s=0.51)
        with self.assertRaisesRegex(ContextValidationError, "observation_age_s"):
            validate_interaction_context(_context(actors=[stale_actor]))

    def test_ttl_is_bounded_and_expiry_is_checked_in_sim_time(self) -> None:
        with self.assertRaisesRegex(ContextValidationError, "ttl_s.*0.30"):
            validate_interaction_context(_context(ttl_s=0.301))
        packet = _context()
        with self.assertRaisesRegex(ContextValidationError, "expired"):
            validate_interaction_context(packet, now_sim_time_s=10.30)
        with self.assertRaisesRegex(ContextValidationError, "future"):
            validate_interaction_context(packet, now_sim_time_s=9.99)

    def test_nonfinite_context_actor_and_tracker_values_are_rejected(self) -> None:
        with self.assertRaisesRegex(ContextValidationError, "source_sim_time_s.*finite"):
            validate_interaction_context(_context(source_sim_time_s=float("nan")))
        actor = _actor()
        actor["trajectory_world_xy_m"][0][0] = float("inf")
        with self.assertRaisesRegex(ContextValidationError, "finite"):
            validate_interaction_context(_context(actors=[actor]))
        with self.assertRaisesRegex(ContextValidationError, "finite"):
            _build_packet(predictions=[_prediction(confidence=6, last_update=float("nan"))])
        with self.assertRaises(ContextValidationError):
            serialize_interaction_context(_context(source_sim_time_s=float("inf")))

    def test_observation_and_packet_source_times_cannot_be_future(self) -> None:
        with self.assertRaisesRegex(ContextValidationError, "observation_sim_time_s.*future"):
            validate_interaction_context(_context(observation_sim_time_s=10.01))
        with self.assertRaisesRegex(ContextValidationError, "source_sim_time_s.*future"):
            validate_interaction_context(_context(), now_sim_time_s=9.99)

    def test_replay_guard_rejects_replayed_sequences_and_timestamps(self) -> None:
        guard = _guard()
        first = _context()
        guard.accept(first, now_sim_time_s=10.05, expected_pose_seq=41)

        replayed_packet = copy.deepcopy(first)
        replayed_packet["packet_seq"] = 2
        replayed_packet["pose_seq"] = 42
        replayed_packet["source_sim_time_s"] = 9.99
        replayed_packet["observation_sim_time_s"] = 9.99
        with self.assertRaisesRegex(ContextValidationError, "source_sim_time_s.*monotonic"):
            guard.accept(replayed_packet, now_sim_time_s=10.05, expected_pose_seq=42)

        replayed_packet["source_sim_time_s"] = 10.06
        replayed_packet["observation_sim_time_s"] = 10.06
        replayed_packet["observation_seq"] = 2
        replayed_packet["packet_seq"] = 1
        with self.assertRaisesRegex(ContextValidationError, "packet_seq.*monotonic"):
            guard.accept(replayed_packet, now_sim_time_s=10.07, expected_pose_seq=42)

    def test_pose_sequence_must_match_and_increase(self) -> None:
        guard = _guard()
        with self.assertRaisesRegex(ContextValidationError, "pose_seq mismatch"):
            guard.accept(_context(), now_sim_time_s=10.01, expected_pose_seq=42)
        guard.accept(_context(), now_sim_time_s=10.01, expected_pose_seq=41)
        next_packet = _context(packet_seq=2, pose_seq=41, source_sim_time_s=10.02)
        with self.assertRaisesRegex(ContextValidationError, "pose_seq.*monotonic"):
            guard.accept(next_packet, now_sim_time_s=10.03, expected_pose_seq=41)

    def test_observation_timestamp_may_repeat_but_may_not_go_backwards(self) -> None:
        guard = _guard()
        guard.accept(_context(), now_sim_time_s=10.01, expected_pose_seq=41)
        repeated = _context(packet_seq=2, pose_seq=42, source_sim_time_s=10.02)
        guard.accept(repeated, now_sim_time_s=10.03, expected_pose_seq=42)
        backwards = _context(
            packet_seq=3,
            pose_seq=43,
            source_sim_time_s=10.04,
            observation_sim_time_s=9.99,
        )
        with self.assertRaisesRegex(ContextValidationError, "observation_sim_time_s.*replay"):
            guard.accept(backwards, now_sim_time_s=10.05, expected_pose_seq=43)

    def test_prediction_horizon_must_cover_controller_plus_age(self) -> None:
        too_short = _actor(end_time_s=3.0)
        with self.assertRaisesRegex(ContextValidationError, "prediction horizon.*3.5"):
            validate_interaction_context(_context(actors=[too_short]))
        aged = _actor(observation_age_s=0.4, end_time_s=3.4)
        with self.assertRaisesRegex(ContextValidationError, "prediction horizon.*3.5"):
            validate_interaction_context(_context(actors=[aged]))

    def test_prediction_times_are_ordered_nonnegative_and_array_lengths_match(self) -> None:
        actor = _actor()
        actor["prediction_times_s"] = [0.0, 0.5, 0.5, 3.5]
        actor["trajectory_world_xy_m"] = [[0.0, 0.0]] * 4
        actor["uncertainty_radius_m"] = [0.1] * 4
        with self.assertRaisesRegex(ContextValidationError, "strictly increasing"):
            validate_interaction_context(_context(actors=[actor]))

        actor = _actor()
        actor["prediction_times_s"] = [-0.1, 3.5]
        actor["trajectory_world_xy_m"] = [[0.0, 0.0]] * 2
        actor["uncertainty_radius_m"] = [0.1] * 2
        with self.assertRaisesRegex(ContextValidationError, "nonnegative"):
            validate_interaction_context(_context(actors=[actor]))

        actor = _actor()
        actor["uncertainty_radius_m"] = [0.1]
        with self.assertRaisesRegex(ContextValidationError, "matching array lengths"):
            validate_interaction_context(_context(actors=[actor]))

    def test_actor_and_prediction_arrays_are_bounded(self) -> None:
        with self.assertRaisesRegex(ContextValidationError, f"at most {MAX_ACTORS} actors"):
            validate_interaction_context(_context(actors=[_actor()] * (MAX_ACTORS + 1)))

        actor = _actor()
        times = [float(index) for index in range(MAX_PREDICTION_SAMPLES + 1)]
        actor["prediction_times_s"] = times
        actor["trajectory_world_xy_m"] = [[0.0, 0.0] for _ in times]
        actor["uncertainty_radius_m"] = [0.1 for _ in times]
        with self.assertRaisesRegex(ContextValidationError, "at most .* prediction samples"):
            validate_interaction_context(_context(actors=[actor]))

    def test_body_and_uncertainty_radii_are_bounded(self) -> None:
        for radius in (0.0, -0.1, 2.01, float("nan")):
            with self.subTest(radius=radius), self.assertRaisesRegex(
                ContextValidationError, "body_radius_m"
            ):
                validate_interaction_context(_context(actors=[_actor(body_radius_m=radius)]))
        actor = _actor()
        actor["uncertainty_radius_m"][0] = 10.01
        with self.assertRaisesRegex(ContextValidationError, "uncertainty radius"):
            validate_interaction_context(_context(actors=[actor]))

    def test_confidence_and_weights_are_validated_not_silently_normalized(self) -> None:
        for confidence in (-0.01, 1.01, float("nan")):
            with self.subTest(confidence=confidence), self.assertRaisesRegex(
                ContextValidationError, "confidence"
            ):
                validate_interaction_context(_context(actors=[_actor(confidence=confidence)]))
        actor = _actor()
        actor["model_weights"]["CV"] = 0.8
        with self.assertRaisesRegex(ContextValidationError, "sum to 1"):
            validate_interaction_context(_context(actors=[actor]))

    def test_unavailable_or_exhausted_wait_is_valid_with_neutral_fallback(self) -> None:
        unavailable = _context(action_id="WAIT_YIELD", waitable=False)
        self.assertEqual(validate_interaction_context(unavailable)["action_id"], "WAIT_YIELD")
        decision = resolve_action_preference(unavailable)
        self.assertFalse(decision.requires_safe_stop)
        self.assertEqual(decision.requested_action_id, "WAIT_YIELD")
        self.assertEqual(decision.effective_action_id, "CRUISE")
        self.assertEqual(decision.reason, "wait_not_waitable")
        guard = _guard()
        for index, elapsed in enumerate((1.8, 2.0, 2.4)):
            # Consecutive requested WAIT remains exhausted after CRUISE fallback;
            # resolving it must preserve the producer's ongoing wait budget.
            stamp = 10.0 + index * 0.1
            exhausted = _context(
                action_id="WAIT_YIELD", waitable=True, wait_elapsed_s=elapsed,
                source_sim_time_s=stamp, observation_sim_time_s=stamp,
                packet_seq=index + 1, pose_seq=41 + index, observation_seq=index + 1,
            )
            guard.accept(exhausted, now_sim_time_s=stamp, expected_pose_seq=41 + index)
            self.assertTrue(validate_interaction_context(exhausted)["sensing_valid"])
            before = copy.deepcopy(exhausted)
            decision = resolve_action_preference(exhausted)
            self.assertFalse(decision.requires_safe_stop)
            self.assertEqual(decision.requested_action_id, "WAIT_YIELD")
            self.assertEqual(decision.effective_action_id, "CRUISE")
            self.assertEqual(decision.reason, "wait_budget_exhausted")
            self.assertEqual(exhausted, before)

    def test_wait_eligibility_expires_at_use_time_and_bound_is_validated(self) -> None:
        waiting = _context(action_id="WAIT_YIELD", waitable=True, wait_elapsed_s=1.7)
        self.assertEqual(resolve_action_preference(waiting).effective_action_id, "WAIT_YIELD")
        expired = resolve_action_preference(waiting, now_sim_time_s=10.15)
        self.assertFalse(expired.requires_safe_stop)
        self.assertEqual(expired.effective_action_id, "CRUISE")
        self.assertEqual(expired.reason, "wait_budget_exhausted")
        with self.assertRaisesRegex(ContextValidationError, "wait_duration_limit_s.*1.8"):
            validate_interaction_context(
                _context(action_id="WAIT_YIELD", waitable=True, wait_duration_limit_s=1.81)
            )

    def test_rejected_packet_does_not_consume_sequences_or_timestamps(self) -> None:
        guard = _guard()
        guard.accept(_context(), now_sim_time_s=10.01, expected_pose_seq=41)
        before = guard.snapshot()
        bad = _context(packet_seq=999, pose_seq=999, source_sim_time_s=10.1,
                       observation_sim_time_s=10.1, observation_seq=999, sensing_valid=False)
        with self.assertRaises(ContextValidationError):
            guard.accept(bad, now_sim_time_s=10.11, expected_pose_seq=999)
        self.assertEqual(guard.snapshot(), before)
        good = _context(packet_seq=2, pose_seq=42, source_sim_time_s=10.1,
                        observation_sim_time_s=10.1, observation_seq=2)
        guard.accept(good, now_sim_time_s=10.11, expected_pose_seq=42)
        before = guard.snapshot()
        stale = _context(packet_seq=999, pose_seq=999, source_sim_time_s=10.5,
                         observation_sim_time_s=10.5, observation_seq=999)
        with self.assertRaisesRegex(ContextValidationError, "expired"):
            guard.accept(stale, now_sim_time_s=10.81, expected_pose_seq=999)
        self.assertEqual(guard.snapshot(), before)
        guard.accept(_context(packet_seq=3, pose_seq=43, source_sim_time_s=10.2,
                              observation_sim_time_s=10.2, observation_seq=3),
                     now_sim_time_s=10.21, expected_pose_seq=43)

    def test_observation_cannot_be_restamped_as_fresh(self) -> None:
        guard = _guard()
        guard.accept(_context(), now_sim_time_s=10.01, expected_pose_seq=41)
        before = guard.snapshot()
        restamped = _context(packet_seq=2, pose_seq=42, source_sim_time_s=10.1,
                             observation_sim_time_s=10.1)  # Same observation_seq.
        with self.assertRaisesRegex(ContextValidationError, "restamped"):
            guard.accept(restamped, now_sim_time_s=10.11, expected_pose_seq=42)
        self.assertEqual(guard.snapshot(), before)
        valid = _context(packet_seq=2, pose_seq=42, source_sim_time_s=10.1)
        guard.accept(valid, now_sim_time_s=10.11, expected_pose_seq=42)

    def test_actor_observation_cannot_be_restamped_without_new_sensing(self) -> None:
        guard = _guard()
        guard.accept(_context(actors=[_actor(observation_age_s=0.1)]),
                     now_sim_time_s=10.01, expected_pose_seq=41)
        new_query = _context(packet_seq=2, pose_seq=42, source_sim_time_s=10.1,
                             actors=[_actor(observation_age_s=0.1)])
        with self.assertRaisesRegex(ContextValidationError, "actor observation age.*restamped"):
            guard.accept(new_query, now_sim_time_s=10.11, expected_pose_seq=42)

    def test_new_observation_sequence_requires_new_sensor_timestamp(self) -> None:
        guard = _guard()
        guard.accept(_context(), now_sim_time_s=10.01, expected_pose_seq=41)
        no_scan = _context(packet_seq=2, pose_seq=42, source_sim_time_s=10.1,
                           observation_seq=2)
        with self.assertRaisesRegex(ContextValidationError, "requires a newer"):
            guard.accept(no_scan, now_sim_time_s=10.11, expected_pose_seq=42)

    def test_epoch_reset_requires_disarmed_and_never_reuses_old_epoch(self) -> None:
        guard = _guard()
        guard.accept(_context(), now_sim_time_s=10.01, expected_pose_seq=41)
        before = guard.snapshot()
        with self.assertRaisesRegex(ContextValidationError, "only while disarmed"):
            guard.reset_epoch("episode-2", disarmed=False)
        self.assertEqual(guard.snapshot(), before)
        with self.assertRaisesRegex(ContextValidationError, "cannot reuse"):
            guard.reset_epoch("episode-1", disarmed=True)
        self.assertEqual(guard.snapshot(), before)
        guard.reset_epoch("episode-2", disarmed=True)
        with self.assertRaisesRegex(ContextValidationError, "epoch_id mismatch"):
            guard.accept(_context(), now_sim_time_s=10.01, expected_pose_seq=41)

    def test_episode_clock_resets_but_pose_sequence_keeps_increasing(self) -> None:
        guard = _guard()
        guard.accept(_context(), now_sim_time_s=10.01, expected_pose_seq=41)
        guard.reset_epoch("episode-2", disarmed=True)
        initial = _context(epoch_id="episode-2", packet_seq=0, pose_seq=41,
                           observation_seq=0, source_sim_time_s=0.0,
                           observation_sim_time_s=0.0, actors=[])
        with self.assertRaisesRegex(ContextValidationError, "pose_seq.*monotonic"):
            guard.accept(initial, now_sim_time_s=0.0, expected_pose_seq=41)
        initial["pose_seq"] = 42
        accepted = guard.accept(initial, now_sim_time_s=0.0, expected_pose_seq=42)
        self.assertEqual(accepted["source_sim_time_s"], 0.0)
        self.assertEqual(accepted["packet_seq"], 0)

    def test_stationary_zero_time_needs_no_fabricated_clock_increment(self) -> None:
        guard = _guard()
        initial = _context(packet_seq=0, pose_seq=0, observation_seq=0,
                           source_sim_time_s=0.0, observation_sim_time_s=0.0, actors=[])
        guard.accept(initial, now_sim_time_s=0.0, expected_pose_seq=0)
        initial["packet_seq"] = 1
        initial["pose_seq"] = 1
        guard.accept(initial, now_sim_time_s=0.0, expected_pose_seq=1)
        self.assertEqual(guard.snapshot()["source_sim_time_s"], 0.0)

    def test_guard_cannot_accept_until_disarmed_epoch_initialization(self) -> None:
        guard = InteractionContextFreshnessGuard()
        with self.assertRaisesRegex(ContextValidationError, "requires reset_epoch"):
            guard.accept(_context(), now_sim_time_s=10.01, expected_pose_seq=41)

    def test_receipt_time_does_not_replace_source_simulation_clock(self) -> None:
        packet = _context(jetson_ros_receipt_time_s=10.0)
        with self.assertRaisesRegex(ContextValidationError, "unexpected field"):
            validate_interaction_context(packet)

    def test_sensor_and_track_freshness_are_checked_at_current_sim_time(self) -> None:
        packet = _context(observation_sim_time_s=9.6)
        with self.assertRaisesRegex(ContextValidationError, "observation age"):
            validate_interaction_context(packet, now_sim_time_s=10.2)
        packet = _context(actors=[_actor(observation_age_s=0.4)])
        with self.assertRaisesRegex(ContextValidationError, "observation_age_s.*stale"):
            validate_interaction_context(packet, now_sim_time_s=10.2)

    def test_unknown_action_and_unknown_fields_are_rejected(self) -> None:
        with self.assertRaisesRegex(ContextValidationError, "unknown action_id"):
            validate_interaction_context(_context(action_id="AVOID"))
        packet = _context(unexpected=True)
        with self.assertRaisesRegex(ContextValidationError, "unexpected field"):
            validate_interaction_context(packet)

    def test_json_duplicate_keys_and_nonstandard_nan_are_rejected(self) -> None:
        duplicate = '{"schema_version":1,"schema_version":1}'
        with self.assertRaisesRegex(ContextValidationError, "duplicate JSON object key"):
            parse_interaction_context(duplicate)
        with self.assertRaisesRegex(ContextValidationError, "non-finite JSON number"):
            parse_interaction_context('{"value":NaN}')


if __name__ == "__main__":
    unittest.main()
