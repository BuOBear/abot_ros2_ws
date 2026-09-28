import math

import pytest

from abot_mission.state_machine import Mission, parse_route, SUCCEEDED, CANCELED, ABORTED


def route(observe=False):
    steps = [
        {'type': 'navigate', 'name': 'first', 'pose': {'x': 1.0, 'y': -2.0, 'yaw': 90},
         'timeout_sec': 3.0},
        {'type': 'navigate', 'name': 'second', 'pose': {'x': 2.0, 'y': 1.0, 'yaw': 0},
         'timeout_sec': 3.0},
    ]
    if observe:
        steps[0]['observe'] = {'topic': '/perception/templates', 'class_id': 'template:10',
                               'timeout_sec': 2.0, 'max_age_sec': 0.5, 'min_score': 0.7}
    return parse_route({'map_id': 'validated_map_A', 'frame_id': 'map',
                        'angle_unit': 'degrees', 'steps': steps})


def test_route_rejects_unverified_or_malformed_poses():
    for change in ({'map_id': ''}, {'frame_id': 'odom'}, {'angle_unit': 'implicit'},
                   {'steps': []}):
        data = {'map_id': 'A', 'frame_id': 'map', 'angle_unit': 'degrees',
                'steps': [{'type': 'navigate', 'name': 'one',
                           'pose': {'x': 1, 'y': 0, 'yaw': 0}, 'timeout_sec': 5}]}
        data.update(change)
        with pytest.raises(ValueError):
            parse_route(data)
    parsed = route()
    assert math.isclose(parsed.steps[0].yaw_rad, math.pi / 2)


def test_only_bound_succeeded_goal_advances():
    mission = Mission(route())
    assert mission.start()
    token = mission.dispatch(0.0)
    assert mission.goal_response(token, True)
    assert not mission.goal_result(token + 1, SUCCEEDED, 1.0)
    assert mission.index == 0
    assert mission.goal_result(token, SUCCEEDED, 1.0)
    assert mission.index == 1 and mission.phase == 'preparing'
    second = mission.dispatch(2.0)
    assert second != token
    assert not mission.goal_result(token, SUCCEEDED, 3.0)
    assert mission.goal_response(second, True)
    assert mission.goal_result(second, ABORTED, 3.0)
    assert mission.index == 1 and mission.phase == 'failed'


def test_rejection_timeout_and_cancel_result():
    mission = Mission(route())
    mission.start()
    token = mission.dispatch(0.0)
    mission.goal_response(token, False)
    assert mission.phase == 'failed' and mission.detail == 'goal_rejected'
    assert mission.start()
    token = mission.dispatch(10.0)
    mission.goal_response(token, True)
    assert mission.tick(13.01) == 'cancel_goal'
    assert mission.phase == 'canceling' and not mission.start()
    assert mission.index == 0 and mission.active_token == token
    mission.goal_result(token, CANCELED, 14.0)
    assert mission.phase == 'failed' and 'navigation_timeout' in mission.detail
    assert mission.start()
    token = mission.dispatch(20.0)
    mission.goal_response(token, True)
    mission.request_cancel('operator')
    mission.goal_result(token, SUCCEEDED, 21.0)
    assert mission.phase == 'canceled' and mission.detail == 'operator:goal_status_4'
    assert mission.index == 0


def test_observation_requires_post_arrival_fresh_matching_sample():
    mission = Mission(route(observe=True))
    mission.start()
    token = mission.dispatch(0.0)
    mission.goal_response(token, True)
    mission.goal_result(token, SUCCEEDED, 100.0)
    mission.start_observation(5.0)
    assert mission.phase == 'observing'
    assert not mission.observation('/perception/templates', 'template:10', 0.8, 99.9, 100.1, 5.1)
    assert not mission.observation('/perception/colors', 'template:10', 0.8, 100.0, 100.1, 5.1)
    assert not mission.observation('/perception/templates', 'template:11', 0.8, 100.0, 100.1, 5.1)
    assert not mission.observation('/perception/templates', 'template:10', 0.8, 100.0, 100.7, 5.1)
    assert mission.observation('/perception/templates', 'template:10', 0.8, 100.1, 100.2, 5.2)
    assert mission.phase == 'preparing' and mission.index == 1
