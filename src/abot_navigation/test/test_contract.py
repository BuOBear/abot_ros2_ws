from pathlib import Path
import importlib.util

from launch import LaunchContext
import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]


def config(name):
    return yaml.safe_load((ROOT / 'config' / name).read_text())


def test_ekf_owns_local_tf_and_fuses_lateral_velocity():
    ekf = config('ekf.yaml')['ekf_filter_node']['ros__parameters']
    assert (ekf['world_frame'], ekf['odom_frame'], ekf['base_link_frame']) == (
        'odom', 'odom', 'base_footprint')
    assert ekf['publish_tf'] is True
    assert ekf['odom0'] == '/wheel_odom'
    assert ekf['odom0_config'][7] is True
    assert len(ekf['odom0_config']) == len(ekf['imu0_config']) == 15
    assert config('imu_filter.yaml')['imu_filter']['ros__parameters']['publish_tf'] is False


@pytest.mark.parametrize('simulated', [True, False])
def test_nav2_rewrite_sets_clock_for_servers_and_nested_costmaps(simulated):
    from nav2_common.launch import RewrittenYaml
    rewritten = RewrittenYaml(
        source_file=str(ROOT / 'config' / 'nav2.yaml'),
        param_rewrites={'use_sim_time': str(simulated).lower()}, convert_types=True)
    path = Path(rewritten.perform(LaunchContext()))
    try:
        data = yaml.safe_load(path.read_text())
    finally:
        path.unlink()
    clocks = []

    def visit(value):
        if isinstance(value, dict):
            for key, child in value.items():
                if key == 'ros__parameters':
                    assert child['use_sim_time'] is simulated
                    clocks.append(child['use_sim_time'])
                else:
                    visit(child)

    visit(data)
    assert len(clocks) == 11


def test_nav2_omni_and_filtered_scan_contract():
    nav = config('nav2.yaml')
    amcl = nav['amcl']['ros__parameters']
    assert amcl['robot_model_type'] == 'nav2_amcl::OmniMotionModel'
    assert amcl['scan_topic'] == '/scan_filtered'
    dwb = nav['controller_server']['ros__parameters']['FollowPath']
    assert dwb['plugin'] == 'dwb_core::DWBLocalPlanner'
    assert dwb['min_vel_y'] < 0 < dwb['max_vel_y']
    assert dwb['vy_samples'] >= 3 and dwb['acc_lim_y'] > 0 and dwb['decel_lim_y'] < 0
    assert nav['planner_server']['ros__parameters']['GridBased']['plugin'] == (
        'nav2_navfn_planner/NavfnPlanner')
    for costmap in ('local_costmap', 'global_costmap'):
        params = nav[costmap][costmap]['ros__parameters']
        assert params['robot_base_frame'] == 'base_footprint'
        assert params['obstacle_layer']['scan']['topic'] == '/scan_filtered'
    local = nav['local_costmap']['local_costmap']['ros__parameters']
    assert type(local['width']) is int and type(local['height']) is int


def test_only_one_velocity_smoother_and_final_cmd_vel_owner():
    velocity = config('velocity.yaml')
    mux = velocity['twist_mux']['ros__parameters']['topics']
    assert (mux['hold']['priority'] > mux['teleop']['priority'] >
            mux['tracking']['priority'] > mux['navigation']['priority'])
    assert all(source['timeout'] > 0 for source in mux.values())
    assert velocity['velocity_smoother']['ros__parameters']['max_velocity'][1] > 0
    monitor = velocity['collision_monitor']['ros__parameters']
    assert monitor['cmd_vel_in_topic'] == '/cmd_vel/smoothed'
    assert monitor['cmd_vel_out_topic'] == '/cmd_vel/collision_checked'
    assert monitor['scan']['topic'] == '/scan_filtered'
    gate = velocity['velocity_gate']['ros__parameters']
    assert 0 < gate['input_timeout'] < monitor['source_timeout']
    assert gate['scan_timeout'] <= monitor['source_timeout']
    nav_launch = (ROOT / 'launch' / 'navigation.launch.py').read_text()
    vel_launch = (ROOT / 'launch' / 'velocity.launch.py').read_text()
    assert "package='nav2_velocity_smoother'" not in nav_launch
    assert vel_launch.count("package='nav2_velocity_smoother'") == 1
    assert vel_launch.count("executable='velocity_gate'") == 1
    assert "('cmd_vel', '/cmd_vel/nav')" in nav_launch


def test_navigation_launch_selects_one_map_owner(tmp_path):
    path = ROOT / 'launch' / 'navigation.launch.py'
    spec = importlib.util.spec_from_file_location('abot_navigation_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.get_package_share_directory = lambda _: str(ROOT)
    context = LaunchContext()
    context.launch_configurations.update(
        {'mode': 'mapping', 'map': '', 'use_sim_time': 'false'})
    mapping = module.launch_setup(context)
    mapping_executables = [node.node_executable for node in mapping]
    assert 'sync_slam_toolbox_node' in mapping_executables
    assert 'amcl' not in mapping_executables
    assert 'map_server' not in mapping_executables
    assert 'velocity_smoother' not in mapping_executables

    map_path = tmp_path / 'map.yaml'
    map_path.write_text('image: map.pgm\n')
    context.launch_configurations.update(
        {'mode': 'localization', 'map': str(map_path)})
    localization = module.launch_setup(context)
    localization_executables = [node.node_executable for node in localization]
    assert 'amcl' in localization_executables
    assert 'map_server' in localization_executables
    assert 'sync_slam_toolbox_node' not in localization_executables
    assert 'map_saver_server' not in localization_executables
    for node in localization:
        if node.node_executable in ('controller_server', 'behavior_server'):
            remaps = [(
                ''.join(part.perform(context) for part in source),
                ''.join(part.perform(context) for part in destination))
                for source, destination in node._Node__remappings]
            assert ('cmd_vel', '/cmd_vel/nav') in remaps

    context.launch_configurations['map'] = '/missing/map.yaml'
    with pytest.raises(RuntimeError, match='existing map'):
        module.launch_setup(context)
    context.launch_configurations['mode'] = 'invalid'
    with pytest.raises(RuntimeError, match='mode must'):
        module.launch_setup(context)
