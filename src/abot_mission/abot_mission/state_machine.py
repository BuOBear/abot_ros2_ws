"""Transport-free mission policy.  A goal token identifies one Nav2 request."""

from dataclasses import dataclass
import math


SUCCEEDED = 4  # action_msgs/msg/GoalStatus.STATUS_SUCCEEDED
CANCELED = 5
ABORTED = 6


@dataclass(frozen=True)
class Observation:
    topic: str
    class_id: str
    timeout: float
    max_age: float
    min_score: float


@dataclass(frozen=True)
class Step:
    name: str
    x: float
    y: float
    yaw_rad: float
    timeout: float
    observe: Observation | None = None


@dataclass(frozen=True)
class Route:
    map_id: str
    frame_id: str
    steps: tuple[Step, ...]


def _number(value, field, *, minimum=None, maximum=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{field} must be a finite number')
    value = float(value)
    if not math.isfinite(value) or (minimum is not None and value < minimum) or (
            maximum is not None and value > maximum):
        raise ValueError(f'{field} is outside its valid range')
    return value


def parse_route(data):
    """Require explicit map identity and finite map poses; no fallback coordinates."""
    if not isinstance(data, dict):
        raise ValueError('route must be a mapping')
    map_id = data.get('map_id')
    frame_id = data.get('frame_id')
    angle_unit = data.get('angle_unit')
    if not isinstance(map_id, str) or not map_id.strip():
        raise ValueError('map_id must identify the validated map')
    if frame_id != 'map':
        raise ValueError('frame_id must be map')
    if angle_unit not in ('degrees', 'radians'):
        raise ValueError('angle_unit must be degrees or radians')
    raw_steps = data.get('steps')
    if not isinstance(raw_steps, list) or not raw_steps:
        raise ValueError('steps must be a nonempty list')
    steps = []
    names = set()
    for index, raw in enumerate(raw_steps):
        if not isinstance(raw, dict) or raw.get('type') != 'navigate':
            raise ValueError(f'steps[{index}] must be a navigate mapping')
        name = raw.get('name')
        pose = raw.get('pose')
        if not isinstance(name, str) or not name.strip() or name in names:
            raise ValueError(f'steps[{index}].name must be unique and nonempty')
        names.add(name)
        if not isinstance(pose, dict):
            raise ValueError(f'steps[{index}].pose is required')
        x = _number(pose.get('x'), 'x')
        y = _number(pose.get('y'), 'y')
        yaw = _number(pose.get('yaw'), 'yaw')
        timeout = _number(raw.get('timeout_sec'), 'timeout_sec', minimum=0.001)
        observation = None
        if 'observe' in raw:
            obs = raw['observe']
            if not isinstance(obs, dict):
                raise ValueError('observe must be a mapping')
            topic = obs.get('topic')
            class_id = obs.get('class_id')
            if (not isinstance(topic, str) or not topic.startswith('/perception/') or
                    not isinstance(class_id, str) or not class_id.strip()):
                raise ValueError('observe requires an absolute perception topic and class_id')
            observation = Observation(
                topic, class_id,
                _number(obs.get('timeout_sec'), 'observe.timeout_sec', minimum=0.001),
                _number(obs.get('max_age_sec'), 'observe.max_age_sec', minimum=0.001),
                _number(obs.get('min_score', 0.0), 'observe.min_score', minimum=0.0,
                        maximum=1.0))
        steps.append(Step(name, x, y, math.radians(yaw) if angle_unit == 'degrees' else yaw,
                          timeout, observation))
    return Route(map_id.strip(), frame_id, tuple(steps))


class Mission:
    """All terminal navigation transitions require the current goal's result."""

    def __init__(self, route):
        self.route = route
        self.phase = 'idle'
        self.index = 0
        self.token = 0
        self.active_token = None
        self.goal_accepted = False
        self.cancel_requested = False
        self.cancel_reason = ''
        self.deadline = None
        self.observation_started_ros = None
        self.detail = ''

    @property
    def step(self):
        return self.route.steps[self.index]

    def start(self):
        if self.phase not in ('idle', 'completed', 'failed', 'canceled') or self.active_token is not None:
            return False
        self.phase = 'preparing'
        self.index = 0
        self.detail = ''
        self.cancel_requested = False
        self.cancel_reason = ''
        self.deadline = None
        return True

    def dispatch(self, now):
        if self.phase != 'preparing' or self.active_token is not None:
            return None
        self.token += 1
        self.active_token = self.token
        self.goal_accepted = False
        self.deadline = now + self.step.timeout
        self.phase = 'awaiting_goal'
        return self.token

    def goal_response(self, token, accepted):
        if token != self.active_token or self.phase not in ('awaiting_goal', 'canceling'):
            return False
        if not accepted:
            self.active_token = None
            self.phase = 'canceled' if self.cancel_requested else 'failed'
            self.detail = 'goal_rejected'
            return True
        self.goal_accepted = True
        if not self.cancel_requested:
            self.phase = 'navigating'
        return True

    def request_cancel(self, reason='operator'):
        if self.phase in ('preparing', 'observing'):
            self.phase = 'canceled' if reason == 'operator' else 'failed'
            self.detail = reason
            return True
        if self.phase in ('awaiting_goal', 'navigating', 'canceling'):
            self.cancel_requested = True
            self.cancel_reason = reason
            self.phase = 'canceling'
            return True
        return False

    def tick(self, now):
        if self.deadline is not None and now > self.deadline:
            if self.phase in ('awaiting_goal', 'navigating'):
                self.request_cancel('navigation_timeout')
                return 'cancel_goal'
            if self.phase == 'observing':
                self.phase = 'failed'
                self.detail = 'observation_timeout'
        return None

    def goal_result(self, token, status, now_ros):
        if token != self.active_token or not self.goal_accepted:
            return False
        self.active_token = None
        self.goal_accepted = False
        self.deadline = None
        if self.cancel_requested:
            self.phase = 'canceled' if self.cancel_reason == 'operator' else 'failed'
            self.detail = f'{self.cancel_reason}:goal_status_{status}'
        elif status != SUCCEEDED:
            self.phase = 'failed'
            self.detail = f'goal_status_{status}'
        elif self.step.observe is not None:
            self.phase = 'observing'
            self.observation_started_ros = now_ros
            self.deadline = None  # Set by start_observation with monotonic time.
        else:
            self._advance()
        return True

    def start_observation(self, now):
        if self.phase == 'observing' and self.deadline is None:
            self.deadline = now + self.step.observe.timeout

    def observation(self, topic, class_id, score, stamp_ros, now_ros, now):
        if self.phase != 'observing' or self.deadline is None:
            return False
        obs = self.step.observe
        if (topic != obs.topic or class_id != obs.class_id or
                not math.isfinite(score) or score < obs.min_score or
                not math.isfinite(stamp_ros) or
                stamp_ros < self.observation_started_ros or
                not 0 <= now_ros - stamp_ros <= obs.max_age or now > self.deadline):
            return False
        self._advance()
        return True

    def _advance(self):
        self.deadline = None
        self.cancel_requested = False
        self.cancel_reason = ''
        self.index += 1
        self.phase = 'completed' if self.index >= len(self.route.steps) else 'preparing'
