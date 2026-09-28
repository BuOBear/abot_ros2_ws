import os
import select
import threading
import pytest
import serial
import yaml
from jy_real_robot.servo import ACTUATORS, encode, load_profile, ServoTransport


@pytest.mark.parametrize('channel,angle,expected', [
    ('A', 0, b'$A000#'), ('A', 1, b'$A001#'), ('F', 9, b'$F009#'),
    ('X', 90, b'$X090#'), ('D', 100, b'$D100#'), ('X', 180, b'$X180#')])
def test_vendor_encoding(channel, angle, expected):
    assert encode(channel, angle) == expected


@pytest.mark.parametrize('channel,angle', [('Y', 90), ('a', 90), ('AA', 90),
    ('A', -1), ('A', 181), ('A', 90.1), ('A', True), ('A', float('nan'))])
def test_reject_bad_encoding(channel, angle):
    with pytest.raises(ValueError):
        encode(channel, angle)


def profile():
    return {name: {'channel': chr(65 + index), 'min_deg': 40, 'max_deg': 120}
            for index, name in enumerate(ACTUATORS)}


@pytest.mark.parametrize('mutation', ['unverified', 'duplicate', 'missing', 'reversed', 'outside'])
def test_bad_profile(tmp_path, mutation):
    data = {'calibration_verified': True, 'actuators': profile()}
    if mutation == 'unverified':
        data['calibration_verified'] = False
    elif mutation == 'duplicate':
        data['actuators']['gripper']['channel'] = 'A'
    elif mutation == 'missing':
        del data['actuators']['gripper']
    elif mutation == 'reversed':
        data['actuators']['gripper']['min_deg'] = 160
    else:
        data['actuators']['gripper']['max_deg'] = 270
    path = tmp_path / 'profile.yaml'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        load_profile(path)


def test_serial_pty_no_home_bounds_and_burst_rejection():
    master, slave = os.openpty()
    port = serial.Serial(os.ttyname(slave), 9600, timeout=0, write_timeout=0.2)
    try:
        now = [1.0]
        transport = ServoTransport(port, profile(), clock=lambda: now[0])
        assert not select.select([master], [], [], 0.02)[0]
        assert transport.send('joint_1', 90) == b'$A090#'
        assert select.select([master], [], [], 0.5)[0]
        assert os.read(master, 100) == b'$A090#'
        with pytest.raises(ValueError):
            transport.send('gripper', 100)
        now[0] += 1
        with pytest.raises(ValueError):
            transport.send('gripper', 180)
        with pytest.raises(ValueError):
            transport.send('joint_6', 90)
        assert not select.select([master], [], [], 0.02)[0]
        transport.send('gripper', 100)
        assert select.select([master], [], [], 0.5)[0]
        assert os.read(master, 100) == b'$F100#'
    finally:
        port.close()
        os.close(master)
        os.close(slave)


def test_partial_write_latches_fault_without_retry():
    class Partial:
        calls = 0
        def write(self, data):
            self.calls += 1
            return 3
    port = Partial()
    transport = ServoTransport(port, profile())
    with pytest.raises(IOError):
        transport.send('joint_1', 90)
    with pytest.raises(RuntimeError):
        transport.send('joint_1', 90)
    assert port.calls == 1


@pytest.mark.parametrize('interval', [True, False, None, '0.2', float('nan'), float('inf'), 0.099])
def test_invalid_command_interval_rejected(interval):
    with pytest.raises(ValueError, match='minimum_interval'):
        ServoTransport(None, profile(), minimum_interval=interval)


def test_concurrent_commands_are_serialized_and_burst_is_discarded():
    master, slave = os.openpty()
    port = serial.Serial(os.ttyname(slave), 9600, timeout=0, write_timeout=0.2)
    gate = threading.Barrier(3)
    results = []

    def command(actuator, angle):
        gate.wait(timeout=2)
        try:
            results.append(transport.send(actuator, angle))
        except ValueError as exc:
            results.append(exc)

    try:
        transport = ServoTransport(port, profile(), clock=lambda: 1.0)
        threads = [threading.Thread(target=command, args=('joint_1', 90)),
                   threading.Thread(target=command, args=('gripper', 100))]
        for thread in threads:
            thread.start()
        gate.wait(timeout=2)
        for thread in threads:
            thread.join(timeout=2)
            assert not thread.is_alive()

        packets = [result for result in results if isinstance(result, bytes)]
        rejected = [result for result in results if isinstance(result, ValueError)]
        assert len(packets) == len(rejected) == 1
        assert str(rejected[0]) == 'commands too close; command discarded, not queued'
        assert select.select([master], [], [], 0.5)[0]
        assert os.read(master, 100) == packets[0]
        assert packets[0] in (b'$A090#', b'$F100#')
        assert not select.select([master], [], [], 0.02)[0]
    finally:
        port.close()
        os.close(master)
        os.close(slave)


def test_pty_write_failure_latches_and_never_retries():
    master, slave = os.openpty()
    port = serial.Serial(os.ttyname(slave), 9600, timeout=0, write_timeout=0.2)
    transport = ServoTransport(port, profile())
    os.close(master)
    try:
        with pytest.raises(serial.SerialException):
            transport.send('joint_1', 90)
        assert transport.fault
        with pytest.raises(RuntimeError, match='serial fault latched'):
            transport.send('joint_1', 90)
    finally:
        port.close()
        os.close(slave)
