"""Vendor 24-channel ASCII protocol. No motion feedback is assumed."""
import math
import threading
import time

import yaml

ACTUATORS = tuple(f'joint_{i}' for i in range(1, 6)) + ('gripper',)


def encode(channel, degrees):
    if not isinstance(channel, str) or len(channel) != 1 or not 'A' <= channel <= 'X':
        raise ValueError('channel must be A..X')
    if type(degrees) is not int or not 0 <= degrees <= 180:
        raise ValueError('board angle must be an integer in 0..180 degrees')
    # Vendor Python example incorrectly formats 1..9 with only two digits.
    # C51 UART_Servo sends three digits, including both leading zeroes.
    return f'${channel}{degrees:03d}#'.encode('ascii')


def load_profile(filename):
    with open(filename, encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or data.get('calibration_verified') is not True:
        raise ValueError('arm calibration_verified must be true after physical calibration')
    channels = data.get('actuators', {})
    if not isinstance(channels, dict) or set(channels) != set(ACTUATORS):
        raise ValueError('profile requires exactly joint_1..joint_5 and gripper')
    used = set()
    for name, item in channels.items():
        if not isinstance(item, dict):
            raise ValueError(f'invalid profile for {name}')
        channel, low, high = item.get('channel'), item.get('min_deg'), item.get('max_deg')
        encode(channel, low)
        encode(channel, high)
        if low >= high or channel in used:
            raise ValueError('limits must increase and servo channels must be unique')
        used.add(channel)
    return channels


class ServoTransport:
    """One immediate command; reject bursts and latch I/O faults. Never replay."""
    def __init__(self, port, profile, minimum_interval=0.2, clock=time.monotonic):
        # ``bool`` is an ``int`` in Python, but accepting True as a one-second
        # interval is almost certainly a parameter/configuration mistake.
        try:
            valid_interval = (type(minimum_interval) in (int, float) and
                              math.isfinite(minimum_interval) and
                              minimum_interval >= 0.1)
        except (OverflowError, TypeError):
            valid_interval = False
        if not valid_interval:
            raise ValueError('minimum_interval must be finite and >= 0.1 seconds')
        self.port, self.profile, self.interval = port, profile, minimum_interval
        self.clock, self.last = clock, -math.inf
        self.fault = ''
        self.lock = threading.Lock()

    def send(self, actuator, degrees):
        with self.lock:
            if self.fault:
                raise RuntimeError('serial fault latched; inspect and restart: ' + self.fault)
            if actuator not in self.profile:
                raise ValueError('unknown actuator')
            item = self.profile[actuator]
            packet = encode(item['channel'], degrees)
            if not item['min_deg'] <= degrees <= item['max_deg']:
                raise ValueError('angle outside measured actuator limits')
            now = self.clock()
            if now - self.last < self.interval:
                raise ValueError('commands too close; command discarded, not queued')
            self.last = now
            try:
                if self.port.write(packet) != len(packet):
                    raise IOError('partial serial write; board state unknown')
            except Exception as exc:
                self.fault = str(exc)
                raise
            return packet
