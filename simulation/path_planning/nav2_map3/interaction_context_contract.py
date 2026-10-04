"""Version-1 JSON contract for /simulation/interaction_context, Python 3.8+.

All timestamps are seconds in one *source simulation* epoch. Source time is
the tracker query/packet time; observation time is the latest actual sensing
update. Prediction offsets start at that query time, not at a fabricated new
measurement time. Actor observation age comes from the perceived track's
``last_update``. No ROS receipt stamp belongs in this packet. A receiver must
establish source-clock alignment separately before supplying ``now_sim_time_s``.

``build_interaction_context_packet`` reads only the existing prediction fields
``track``, ``trajectory_world``, ``trajectory_uncertainty_world``,
``model_weights`` and ``prediction_times``, plus perceived body-radius metadata.
It copies the fused mean and scalar uncertainty tube without padding, shifting,
normalizing weights, or claiming per-mode covariance. Tracker confidence levels
0..8 are explicitly represented as 0..1 in the wire packet.

Use ``validate_interaction_context`` for intrinsic/clock checks, JSON
``serialize_interaction_context`` / ``parse_interaction_context`` for transport,
and ``InteractionContextFreshnessGuard.accept`` for transactional replay checks.
Runtime acceptance requires the guard and a matching independently received
pose sequence. Invalid sensing raises ``ContextValidationError``; the bridge
must disarm and invalidate its usable context. The helper never retains an old
usable packet. An unavailable/exhausted WAIT_YIELD remains a valid preference;
``resolve_action_preference`` removes the waiting preference via neutral CRUISE
fallback without labelling the sensing malformed. CRUISE is not a velocity
command; human/static costs and hard guards still apply. These helpers implement
no velocity or clearance policy.
"""

from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Mapping, Optional, Tuple, Union


SCHEMA_VERSION = 1
ACTION_IDS = ("CRUISE", "AVOID_LEFT", "AVOID_RIGHT", "SLOWDOWN", "WAIT_YIELD")
MODEL_NAMES = ("CV", "CA", "CTRV")
PREDICTION_SEMANTICS = "fused_mean_scalar_uncertainty_v1"
CONTROLLER_HORIZON_S = 3.0
MAX_OBSERVATION_AGE_S = 0.5
MIN_PREDICTION_COVERAGE_S = CONTROLLER_HORIZON_S + MAX_OBSERVATION_AGE_S
MAX_TTL_S = 0.30
MAX_WAIT_DURATION_S = 1.8  # Existing decision configuration's upper bound.
MAX_ACTORS = 8
MAX_PREDICTION_SAMPLES = 64
MAX_BODY_RADIUS_M = 2.0
MAX_UNCERTAINTY_RADIUS_M = 10.0
MAX_JSON_BYTES = 512 * 1024
MAX_SEQUENCE = 2 ** 53 - 1  # Exact integer representation in JSON consumers.
TRACKER_CONFIDENCE_LEVELS = 8
_IDENTIFIER_PATTERN = r"^[A-Za-z0-9][A-Za-z0-9_.:/-]*$"
_TIME_TOLERANCE_S = 1e-9


def _array_schema(items: Dict[str, Any]) -> Dict[str, Any]:
    return {"type": "array", "minItems": 2,
            "maxItems": MAX_PREDICTION_SAMPLES, "items": items}


_ACTOR_PROPERTIES = {
    "actor_id": {"type": "string", "minLength": 1, "maxLength": 128,
                 "pattern": _IDENTIFIER_PATTERN},
    "confidence": {"type": "number", "minimum": 0, "maximum": 1},
    "body_radius_m": {"type": "number", "exclusiveMinimum": 0,
                      "maximum": MAX_BODY_RADIUS_M},
    "observation_age_s": {"type": "number", "minimum": 0,
                          "maximum": MAX_OBSERVATION_AGE_S},
    "model_weights": {
        "type": "object", "additionalProperties": False,
        "required": list(MODEL_NAMES),
        "properties": {name: {"type": "number", "minimum": 0, "maximum": 1}
                       for name in MODEL_NAMES},
    },
    "prediction_times_s": _array_schema({"type": "number", "minimum": 0}),
    "trajectory_world_xy_m": _array_schema({
        "type": "array", "minItems": 2, "maxItems": 2,
        "items": {"type": "number"},
    }),
    "uncertainty_radius_m": _array_schema({
        "type": "number", "minimum": 0, "maximum": MAX_UNCERTAINTY_RADIUS_M,
    }),
}
_PACKET_PROPERTIES = {
    "schema_version": {"type": "integer", "const": SCHEMA_VERSION},
    "epoch_id": {"type": "string", "minLength": 1, "maxLength": 128,
                 "pattern": _IDENTIFIER_PATTERN},
    "scene_id": {"type": "string", "minLength": 1, "maxLength": 128,
                 "pattern": _IDENTIFIER_PATTERN},
    "frame_id": {"type": "string", "minLength": 1, "maxLength": 128,
                 "pattern": _IDENTIFIER_PATTERN},
    "packet_seq": {"type": "integer", "minimum": 0, "maximum": MAX_SEQUENCE},
    "pose_seq": {"type": "integer", "minimum": 0, "maximum": MAX_SEQUENCE},
    "observation_seq": {"type": "integer", "minimum": 0, "maximum": MAX_SEQUENCE},
    "source_sim_time_s": {"type": "number", "minimum": 0},
    "observation_sim_time_s": {"type": "number", "minimum": 0},
    "ttl_s": {"type": "number", "exclusiveMinimum": 0, "maximum": MAX_TTL_S},
    "sensing_valid": {"type": "boolean", "const": True},
    "controller_horizon_s": {"type": "number", "const": CONTROLLER_HORIZON_S},
    "prediction_semantics": {"type": "string", "const": PREDICTION_SEMANTICS},
    "action_id": {"type": "string", "enum": list(ACTION_IDS)},
    "waitable": {"type": "boolean"},
    "wait_elapsed_s": {"type": "number", "minimum": 0},
    "wait_duration_limit_s": {"type": "number", "exclusiveMinimum": 0,
                              "maximum": MAX_WAIT_DURATION_S},
    "actors": {"type": "array", "maxItems": MAX_ACTORS, "items": {
        "type": "object", "additionalProperties": False,
        "required": list(_ACTOR_PROPERTIES), "properties": _ACTOR_PROPERTIES,
    }},
}
# JSON Schema describes the wire shape. The functions below also enforce
# finite values, normalized weights, matching arrays, horizon and time/state
# relationships, which cannot all be expressed by this shape schema.
INTERACTION_CONTEXT_SCHEMA = {
    "$schema": "https://json-schema.org/draft/2020-12/schema",
    "title": "Simulation InteractionContext v1",
    "type": "object", "additionalProperties": False,
    "required": list(_PACKET_PROPERTIES), "properties": _PACKET_PROPERTIES,
}


class ContextValidationError(ValueError):
    """A reasoned rejection; runtime callers must disarm on invalid sensing."""


def _fail(reason: str) -> None:
    raise ContextValidationError(reason)


def _object(value: Any, fields: Iterable[str], path: str) -> Dict[str, Any]:
    if not isinstance(value, dict):
        _fail("{} must be a JSON object".format(path))
    expected = set(fields)
    missing = expected.difference(value)
    extra = set(value).difference(expected)
    if missing:
        _fail("{} missing required field(s): {}".format(path, sorted(missing)))
    if extra:
        _fail("{} unexpected field(s): {}".format(path, sorted(extra, key=str)))
    return value


def _number(value: Any, path: str, minimum: Optional[float] = None,
            maximum: Optional[float] = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        _fail("{} must be a finite number".format(path))
    try:
        result = float(value)
    except (ValueError, OverflowError):
        _fail("{} must be a finite number".format(path))
    if not math.isfinite(result):
        _fail("{} must be a finite number".format(path))
    if minimum is not None and result < minimum:
        _fail("{} must be >= {} (nonnegative when minimum is 0)".format(path, minimum))
    if maximum is not None and result > maximum:
        _fail("{} must be <= {}".format(path, maximum))
    return result


def _sequence(value: Any, path: str) -> int:
    if type(value) is not int or not 0 <= value <= MAX_SEQUENCE:
        _fail("{} must be an integer in [0, {}]".format(path, MAX_SEQUENCE))
    return value


def _identifier(value: Any, path: str) -> str:
    if (not isinstance(value, str) or not 1 <= len(value) <= 128
            or re.fullmatch(_IDENTIFIER_PATTERN, value) is None):
        _fail("{} must be a nonempty identifier of at most 128 characters".format(path))
    return value


def _array(value: Any, path: str) -> List[Any]:
    if not isinstance(value, list):
        _fail("{} must be a JSON array".format(path))
    return value


def validate_interaction_context(
    packet: Any, *, now_sim_time_s: Optional[float] = None,
    expected_scene_id: Optional[str] = None, expected_frame_id: Optional[str] = None,
    expected_pose_seq: Optional[int] = None, expected_epoch_id: Optional[str] = None
) -> Dict[str, Any]:
    """Return a detached canonical packet or raise with the failed invariant.

    Omitting ``now_sim_time_s`` validates at the source packet timestamp only;
    it does not establish live freshness, replay resistance or pose alignment.
    Runtime callers use the stateful guard with an independently known pose
    sequence and a verified reading of the same simulation clock.
    """
    raw = _object(packet, _PACKET_PROPERTIES, "context")
    if type(raw["schema_version"]) is not int or raw["schema_version"] != SCHEMA_VERSION:
        _fail("schema_version must be integer 1")
    result = {name: _identifier(raw[name], name)
              for name in ("epoch_id", "scene_id", "frame_id")}
    for name, expected in (("epoch_id", expected_epoch_id),
                           ("scene_id", expected_scene_id), ("frame_id", expected_frame_id)):
        if expected is not None and result[name] != _identifier(expected, "expected_" + name):
            _fail("{} mismatch: expected {!r}, received {!r}".format(name, expected, result[name]))
    for name in ("packet_seq", "pose_seq", "observation_seq"):
        result[name] = _sequence(raw[name], name)
    if expected_pose_seq is not None and result["pose_seq"] != _sequence(expected_pose_seq, "expected_pose_seq"):
        _fail("pose_seq mismatch: expected {}, received {}".format(expected_pose_seq, result["pose_seq"]))
    source = _number(raw["source_sim_time_s"], "source_sim_time_s", 0.0)
    observation = _number(raw["observation_sim_time_s"], "observation_sim_time_s", 0.0)
    if observation > source:
        _fail("observation_sim_time_s is in the future relative to source_sim_time_s")
    now = source if now_sim_time_s is None else _number(now_sim_time_s, "now_sim_time_s", 0.0)
    if now < source:
        _fail("source_sim_time_s is in the future relative to now_sim_time_s")
    ttl = _number(raw["ttl_s"], "ttl_s")
    if not 0.0 < ttl <= MAX_TTL_S:
        _fail("ttl_s must be > 0 and <= 0.30 s")
    if now >= source + ttl:
        _fail("context expired at source_sim_time_s + ttl_s")
    if raw["sensing_valid"] is not True:
        _fail("sensing_valid must be true; missing sensing is not an empty track list")
    if now - observation > MAX_OBSERVATION_AGE_S + _TIME_TOLERANCE_S:
        _fail("observation age exceeds {:.1f} s".format(MAX_OBSERVATION_AGE_S))
    horizon = _number(raw["controller_horizon_s"], "controller_horizon_s")
    if horizon != CONTROLLER_HORIZON_S:
        _fail("controller_horizon_s must be 3.0 s for version 1")
    if raw["prediction_semantics"] != PREDICTION_SEMANTICS:
        _fail("prediction_semantics must identify the fused mean and scalar uncertainty")
    action = raw["action_id"]
    if not isinstance(action, str) or action not in ACTION_IDS:
        _fail("unknown action_id: {!r}".format(action))
    if type(raw["waitable"]) is not bool:
        _fail("waitable must be a boolean")
    wait_elapsed = _number(raw["wait_elapsed_s"], "wait_elapsed_s", 0.0)
    wait_limit = _number(raw["wait_duration_limit_s"], "wait_duration_limit_s")
    if not 0.0 < wait_limit <= MAX_WAIT_DURATION_S:
        _fail("wait_duration_limit_s must be > 0 and <= 1.8 s")
    # WAIT eligibility is an execution decision, not a sensing validity test.
    # Exhausted elapsed time may equal/exceed the limit and is kept verbatim.
    actors = _array(raw["actors"], "actors")
    if len(actors) > MAX_ACTORS:
        _fail("context supports at most {} actors".format(MAX_ACTORS))
    validated_actors = []
    actor_ids = set()
    for index, actor in enumerate(actors):
        path = "actors[{}]".format(index)
        item = _object(actor, _ACTOR_PROPERTIES, path)
        actor_id = _identifier(item["actor_id"], path + ".actor_id")
        if actor_id in actor_ids:
            _fail("duplicate actor_id {!r}".format(actor_id))
        actor_ids.add(actor_id)
        confidence = _number(item["confidence"], path + ".confidence", 0.0, 1.0)
        radius = _number(item["body_radius_m"], path + ".body_radius_m", 0.0, MAX_BODY_RADIUS_M)
        if radius == 0.0:
            _fail(path + ".body_radius_m must be positive")
        actor_age = _number(item["observation_age_s"], path + ".observation_age_s",
                            0.0, MAX_OBSERVATION_AGE_S)
        if actor_age + _TIME_TOLERANCE_S < source - observation:
            _fail(path + ".observation_age_s implies a track observation newer than sensing")
        if actor_age + now - source > MAX_OBSERVATION_AGE_S + _TIME_TOLERANCE_S:
            _fail(path + ".observation_age_s is stale at now_sim_time_s")
        weights_raw = _object(item["model_weights"], MODEL_NAMES, path + ".model_weights")
        weights = {name: _number(weights_raw[name], path + ".model_weights." + name, 0.0, 1.0)
                   for name in MODEL_NAMES}
        if not math.isclose(sum(weights.values()), 1.0, rel_tol=0.0, abs_tol=1e-6):
            _fail(path + ".model_weights must sum to 1; weights are not silently normalized")
        times_raw = _array(item["prediction_times_s"], path + ".prediction_times_s")
        if not 2 <= len(times_raw) <= MAX_PREDICTION_SAMPLES:
            _fail("{} must have 2 to at most {} prediction samples".format(path, MAX_PREDICTION_SAMPLES))
        times = [_number(value, path + ".prediction_times_s[{}]".format(i), 0.0)
                 for i, value in enumerate(times_raw)]
        if any(right <= left for left, right in zip(times, times[1:])):
            _fail(path + ".prediction_times_s must be strictly increasing")
        if times[0] != 0.0:
            _fail(path + ".prediction_times_s must begin at query offset 0; no padding is permitted")
        if times[-1] < MIN_PREDICTION_COVERAGE_S:
            _fail("{} prediction horizon must cover at least 3.5 s (3.0 s controller + bounded observation age)".format(path))
        points = _array(item["trajectory_world_xy_m"], path + ".trajectory_world_xy_m")
        radii = _array(item["uncertainty_radius_m"], path + ".uncertainty_radius_m")
        if len(points) != len(times) or len(radii) != len(times):
            _fail(path + " requires matching array lengths for times, world XY and uncertainty radius")
        xy = []
        for i, point in enumerate(points):
            pair = _array(point, path + ".trajectory_world_xy_m[{}]".format(i))
            if len(pair) != 2:
                _fail(path + ".trajectory_world_xy_m samples must have exactly two coordinates")
            xy.append([_number(value, path + ".trajectory_world_xy_m[{}][{}]".format(i, j))
                       for j, value in enumerate(pair)])
        uncertainty = [_number(value, path + ".uncertainty radius[{}]".format(i),
                               0.0, MAX_UNCERTAINTY_RADIUS_M)
                       for i, value in enumerate(radii)]
        validated_actors.append({
            "actor_id": actor_id, "confidence": confidence, "body_radius_m": radius,
            "observation_age_s": actor_age, "model_weights": weights,
            "prediction_times_s": times, "trajectory_world_xy_m": xy,
            "uncertainty_radius_m": uncertainty,
        })
    result.update({
        "schema_version": SCHEMA_VERSION, "source_sim_time_s": source,
        "observation_sim_time_s": observation, "ttl_s": ttl, "sensing_valid": True,
        "controller_horizon_s": horizon, "prediction_semantics": PREDICTION_SEMANTICS,
        "action_id": action, "waitable": raw["waitable"], "wait_elapsed_s": wait_elapsed,
        "wait_duration_limit_s": wait_limit, "actors": validated_actors,
    })
    return result


def _plain_scalar(value: Any) -> Any:
    if isinstance(value, (str, bool, int, float)):
        return value
    item = getattr(value, "item", None)
    return item() if callable(item) else value


def _plain_array(value: Any, path: str) -> List[Any]:
    """Copy lists/tuples or NumPy-compatible ``tolist`` outputs, no import."""
    if not isinstance(value, (list, tuple)):
        tolist = getattr(value, "tolist", None)
        if not callable(tolist):
            _fail(path + " must be a prediction array")
        value = tolist()
    if not isinstance(value, (list, tuple)):
        _fail(path + " must be a prediction array")
    return [_plain_array(item, path) if isinstance(item, (list, tuple))
            else _plain_scalar(item) for item in value]


def _track_field(track: Any, name: str) -> Any:
    try:
        return track[name] if isinstance(track, Mapping) else getattr(track, name)
    except (KeyError, AttributeError):
        _fail("perceived track missing required field {!r}".format(name))


def build_interaction_context_packet(
    *, epoch_id: str, scene_id: str, frame_id: str, packet_seq: int, pose_seq: int,
    observation_seq: int, source_sim_time_s: float, observation_sim_time_s: float,
    ttl_s: float, sensing_valid: bool, action_id: Union[str, int], waitable: bool,
    wait_elapsed_s: float, wait_duration_limit_s: float,
    tracker_predictions: Iterable[Mapping[str, Any]],
    body_radius_by_track_id: Optional[Mapping[Union[str, int], float]] = None
) -> Dict[str, Any]:
    """Build v1 from perceived tracker prediction dictionaries only.

    The caller preserves the sensor's epoch/observation sequence/timestamp;
    querying predictions does not create a new observation. Body radius must
    be supplied as ``prediction['body_radius_m']`` or in the explicit metadata
    map, from sensing/tracker geometry rather than simulated actor truth.
    Integer PPO action indices 0..4 map to the existing symbolic wire IDs.
    """
    source = _number(source_sim_time_s, "source_sim_time_s", 0.0)
    if type(action_id) is int:
        if not 0 <= action_id < len(ACTION_IDS):
            _fail("unknown action_id: {!r}".format(action_id))
        action_id = ACTION_IDS[action_id]
    if isinstance(tracker_predictions, (str, bytes, Mapping)):
        _fail("tracker_predictions must be an iterable of perceived prediction dictionaries")
    predictions = []
    # Bounded iteration also rejects a generator before consuming unbounded work.
    for prediction in tracker_predictions:
        if len(predictions) >= MAX_ACTORS:
            _fail("context supports at most {} actors".format(MAX_ACTORS))
        if not isinstance(prediction, Mapping):
            _fail("tracker_predictions entries must be prediction dictionaries")
        predictions.append(prediction)
    actors = []
    for prediction in predictions:
        required = ("track", "trajectory_world", "trajectory_uncertainty_world",
                    "model_weights", "prediction_times")
        for field in required:
            if field not in prediction:
                _fail("tracker prediction missing required field {!r}".format(field))
        track = prediction["track"]
        track_id = _plain_scalar(_track_field(track, "track_id"))
        if isinstance(track_id, bool) or not isinstance(track_id, (str, int)):
            _fail("perceived track_id must be an integer or identifier")
        if isinstance(track_id, int) and track_id < 0:
            _fail("perceived track_id must be nonnegative")
        confidence_level = _plain_scalar(_track_field(track, "confidence"))
        if type(confidence_level) is not int or not 0 <= confidence_level <= TRACKER_CONFIDENCE_LEVELS:
            _fail("perceived track confidence must be an integer level in [0, 8]")
        last_observation = _number(_plain_scalar(_track_field(track, "last_update")),
                                   "perceived track last_update", 0.0)
        if "body_radius_m" in prediction:
            body_radius = prediction["body_radius_m"]
        elif body_radius_by_track_id is not None and track_id in body_radius_by_track_id:
            body_radius = body_radius_by_track_id[track_id]
        else:
            _fail("perceived track {!r} missing body_radius_m metadata".format(track_id))
        weights = prediction["model_weights"]
        if not isinstance(weights, Mapping):
            _fail("tracker model_weights must be a mapping")
        actors.append({
            "actor_id": str(track_id),
            "confidence": confidence_level / float(TRACKER_CONFIDENCE_LEVELS),
            "body_radius_m": _plain_scalar(body_radius),
            "observation_age_s": source - last_observation,
            "model_weights": {name: _plain_scalar(value) for name, value in weights.items()},
            "prediction_times_s": _plain_array(prediction["prediction_times"], "prediction_times"),
            "trajectory_world_xy_m": _plain_array(prediction["trajectory_world"], "trajectory_world"),
            "uncertainty_radius_m": _plain_array(prediction["trajectory_uncertainty_world"],
                                                 "trajectory_uncertainty_world"),
        })
    return validate_interaction_context({
        "schema_version": SCHEMA_VERSION, "epoch_id": epoch_id, "scene_id": scene_id,
        "frame_id": frame_id, "packet_seq": packet_seq, "pose_seq": pose_seq,
        "observation_seq": observation_seq, "source_sim_time_s": source,
        "observation_sim_time_s": observation_sim_time_s, "ttl_s": ttl_s,
        "sensing_valid": sensing_valid, "controller_horizon_s": CONTROLLER_HORIZON_S,
        "prediction_semantics": PREDICTION_SEMANTICS, "action_id": action_id,
        "waitable": waitable, "wait_elapsed_s": wait_elapsed_s,
        "wait_duration_limit_s": wait_duration_limit_s, "actors": actors,
    })


def serialize_interaction_context(packet: Any, **validation_options: Any) -> str:
    """Validate and encode deterministic strict JSON; never emit NaN/Infinity."""
    checked = validate_interaction_context(packet, **validation_options)
    encoded = json.dumps(checked, allow_nan=False, sort_keys=True, separators=(",", ":"))
    if len(encoded.encode("utf-8")) > MAX_JSON_BYTES:
        _fail("serialized context exceeds the JSON byte bound")
    return encoded


def _unique_json_object(pairs: List[Tuple[str, Any]]) -> Dict[str, Any]:
    result = {}
    for key, value in pairs:
        if key in result:
            _fail("duplicate JSON object key {!r}".format(key))
        result[key] = value
    return result


def _reject_json_constant(value: str) -> None:
    _fail("non-finite JSON number {!r}".format(value))


def parse_interaction_context(payload: Union[str, bytes], **validation_options: Any) -> Dict[str, Any]:
    """Decode strict JSON, reject duplicates, then validate the v1 packet."""
    if not isinstance(payload, (str, bytes)):
        _fail("context JSON must be text or bytes")
    size = len(payload.encode("utf-8")) if isinstance(payload, str) else len(payload)
    if size > MAX_JSON_BYTES:
        _fail("context JSON exceeds the byte bound")
    try:
        packet = json.loads(payload, object_pairs_hook=_unique_json_object,
                            parse_constant=_reject_json_constant)
    except (ValueError, UnicodeError, RecursionError) as error:
        if isinstance(error, ContextValidationError):
            raise
        _fail("invalid context JSON: {}".format(error))
    return validate_interaction_context(packet, **validation_options)


@dataclass(frozen=True)
class PreferenceResolution:
    requested_action_id: str
    effective_action_id: Optional[str]
    requires_safe_stop: bool
    reason: str


def resolve_action_preference(packet: Any, *, now_sim_time_s: Optional[float] = None) -> PreferenceResolution:
    """Resolve WAIT eligibility independently from sensing validity.

    An unavailable/exhausted WAIT has effective CRUISE, meaning remove the wait
    preference while retaining all human/static costs and hard guards. CRUISE
    is not a command to move forward. Re-evaluate at each use: elapsed wait
    continues from the source stamp. This function preserves the requested
    action, reason and packet's elapsed/budget fields; a fallback never resets
    that budget. The producer must maintain elapsed across consecutive requested
    WAIT packets, including after their effective action becomes CRUISE.
    """
    checked = validate_interaction_context(packet, now_sim_time_s=now_sim_time_s)
    action = checked["action_id"]
    if action == "WAIT_YIELD":
        if not checked["waitable"]:
            return PreferenceResolution(action, "CRUISE", False, "wait_not_waitable")
        now = checked["source_sim_time_s"] if now_sim_time_s is None else now_sim_time_s
        elapsed = checked["wait_elapsed_s"] + now - checked["source_sim_time_s"]
        if elapsed >= checked["wait_duration_limit_s"]:
            return PreferenceResolution(action, "CRUISE", False, "wait_budget_exhausted")
    return PreferenceResolution(action, action, False, "preference_valid")


class InteractionContextFreshnessGuard:
    """Transactional packet/pose/sensor replay guard, persistent across episodes.

    Initialize/reset via ``reset_epoch(new_id, disarmed=True)`` only. The caller
    must attest the bridge is disarmed and its cached context invalidated; this
    ROS-free helper cannot inspect runtime arming. Never automatically reset
    from a received packet or a backwards clock. Epoch IDs cannot be reused.
    Packet/observation sequences and simulation time may restart in a new
    epoch; the accepted pose-sequence high watermark persists across epochs.

    Packet and pose sequences strictly increase. Source time is nondecreasing
    (stationary startup at elapsed=0 is legitimate). A repeated observation
    sequence must keep its original timestamp, actor set and actor measurement
    timestamps. A new observation sequence requires a newer actual observation
    timestamp. No code here creates a sensor update or increments a clock.
    Rejections change no state and callers must disarm, not reuse prior context.
    """

    def __init__(self, *, expected_scene_id: Optional[str] = None,
                 expected_frame_id: Optional[str] = None) -> None:
        self.expected_scene_id = expected_scene_id
        self.expected_frame_id = expected_frame_id
        self._epoch_id = None  # type: Optional[str]
        self._seen_epochs = set()  # type: set
        self._last_pose_seq = None  # type: Optional[int]
        self._last_packet_seq = None  # type: Optional[int]
        self._last_source_time = None  # type: Optional[float]
        self._last_now_time = None  # type: Optional[float]
        self._last_observation_seq = None  # type: Optional[int]
        self._last_observation_time = None  # type: Optional[float]
        self._actor_observation_times = {}  # type: Dict[str, float]

    def snapshot(self) -> Dict[str, Any]:
        """Detached diagnostic state, containing no usable cached context."""
        return {
            "epoch_id": self._epoch_id, "pose_seq": self._last_pose_seq,
            "packet_seq": self._last_packet_seq, "source_sim_time_s": self._last_source_time,
            "now_sim_time_s": self._last_now_time, "observation_seq": self._last_observation_seq,
            "observation_sim_time_s": self._last_observation_time,
            "actor_observation_times": dict(self._actor_observation_times),
        }

    def reset_epoch(self, epoch_id: str, *, disarmed: bool) -> None:
        """Start a never-used epoch while disarmed; retain pose high watermark."""
        if disarmed is not True:
            _fail("reset_epoch is permitted only while disarmed")
        new_epoch = _identifier(epoch_id, "epoch_id")
        if new_epoch in self._seen_epochs:
            _fail("reset_epoch cannot reuse an epoch_id or old context")
        self._seen_epochs.add(new_epoch)
        self._epoch_id = new_epoch
        self._last_packet_seq = None
        self._last_source_time = None
        self._last_now_time = None
        self._last_observation_seq = None
        self._last_observation_time = None
        self._actor_observation_times = {}

    def accept(self, packet: Any, *, now_sim_time_s: float,
               expected_pose_seq: int) -> Dict[str, Any]:
        """Validate all invariants, then update state once and return the copy."""
        if self._epoch_id is None:
            _fail("guard requires reset_epoch while disarmed before accepting context")
        checked = validate_interaction_context(
            packet, now_sim_time_s=now_sim_time_s, expected_epoch_id=self._epoch_id,
            expected_scene_id=self.expected_scene_id, expected_frame_id=self.expected_frame_id,
            expected_pose_seq=expected_pose_seq,
        )
        for name, previous in (("packet_seq", self._last_packet_seq),
                               ("pose_seq", self._last_pose_seq)):
            if previous is not None and checked[name] <= previous:
                _fail("{} must be strictly monotonic; replay rejected".format(name))
        source = checked["source_sim_time_s"]
        observation = checked["observation_sim_time_s"]
        obs_seq = checked["observation_seq"]
        if self._last_source_time is not None and source < self._last_source_time:
            _fail("source_sim_time_s must be monotonic within an epoch; replay rejected")
        if self._last_now_time is not None and now_sim_time_s < self._last_now_time:
            _fail("now_sim_time_s must be monotonic within an epoch; reset only disarmed")
        actor_times = {actor["actor_id"]: source - actor["observation_age_s"]
                       for actor in checked["actors"]}
        if self._last_observation_seq is not None:
            if obs_seq < self._last_observation_seq:
                _fail("observation_seq replay rejected")
            if observation < self._last_observation_time:
                _fail("observation_sim_time_s replay rejected")
            if obs_seq == self._last_observation_seq:
                if observation != self._last_observation_time:
                    _fail("observation_sim_time_s cannot be restamped for the same observation_seq")
                if set(actor_times) != set(self._actor_observation_times):
                    _fail("actor set cannot change without a new sensing observation_seq")
                for actor_id, stamp in actor_times.items():
                    if not math.isclose(stamp, self._actor_observation_times[actor_id],
                                        rel_tol=0.0, abs_tol=_TIME_TOLERANCE_S):
                        _fail("actor observation age cannot be restamped without new sensing")
            elif observation <= self._last_observation_time:
                _fail("a new observation_seq requires a newer observation_sim_time_s")
            for actor_id, stamp in actor_times.items():
                if (actor_id in self._actor_observation_times
                        and stamp + _TIME_TOLERANCE_S < self._actor_observation_times[actor_id]):
                    _fail("actor observation timestamp replay rejected")
        # Transaction commit: no state is touched by any of the checks above.
        self._last_packet_seq = checked["packet_seq"]
        self._last_pose_seq = checked["pose_seq"]
        self._last_source_time = source
        self._last_now_time = float(now_sim_time_s)
        self._last_observation_seq = obs_seq
        self._last_observation_time = observation
        self._actor_observation_times = actor_times
        return checked
