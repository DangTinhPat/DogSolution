"""Record a short /imu/data balance window to JSON."""

import json
import math
import os
import statistics
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


os.environ['ROS_DOMAIN_ID'] = os.environ.get('ROS_DOMAIN_ID', '0')


def quat_to_rpy_deg(q):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return [math.degrees(v) for v in (roll, pitch, yaw)]


class ImuMeasure(Node):
    def __init__(self):
        super().__init__('megadog_imu_measure')
        self.samples = []
        self.create_subscription(Imu, '/imu/data', self._on_imu, 10)

    def _on_imu(self, msg):
        accel = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        gyro = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        self.samples.append({
            'wall_time': time.time(),
            'stamp': float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1.0e-9,
            'frame_id': msg.header.frame_id,
            'rpy_deg': quat_to_rpy_deg(msg.orientation),
            'accel': accel,
            'gyro': gyro,
            'accel_norm': math.sqrt(sum(v * v for v in accel)),
            'gyro_norm': math.sqrt(sum(v * v for v in gyro)),
        })


def _mean(values):
    return statistics.fmean(values) if values else 0.0


def _std(values):
    return statistics.pstdev(values) if len(values) > 1 else 0.0


def main():
    pose = os.environ.get('IMU_POSE', 'unknown')
    wait_seconds = float(os.environ.get('IMU_MEASURE_WAIT', '10'))
    duration_seconds = float(os.environ.get('IMU_MEASURE_DURATION', '10'))

    rclpy.init(args=None)
    node = ImuMeasure()
    print('Waiting for first /imu/data sample...')
    deadline = time.monotonic() + 30.0
    while rclpy.ok() and not node.samples and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not node.samples:
        print('No /imu/data received within 30 s.')
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(2)

    print(f'First sample received. Waiting {wait_seconds:.1f} s before recording...')
    settle_until = time.monotonic() + wait_seconds
    while rclpy.ok() and time.monotonic() < settle_until:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.samples.clear()
    print(f'Recording {duration_seconds:.1f} s for pose={pose}...')
    end_time = time.monotonic() + duration_seconds
    while rclpy.ok() and time.monotonic() < end_time:
        rclpy.spin_once(node, timeout_sec=0.02)

    samples = list(node.samples)
    rolls = [s['rpy_deg'][0] for s in samples]
    pitches = [s['rpy_deg'][1] for s in samples]
    yaws = [s['rpy_deg'][2] for s in samples]
    accel_norms = [s['accel_norm'] for s in samples]
    gyro_norms = [s['gyro_norm'] for s in samples]
    elapsed = samples[-1]['wall_time'] - samples[0]['wall_time'] if len(samples) > 1 else 0.0
    summary = {
        'pose': pose,
        'sample_count': len(samples),
        'duration_s': elapsed,
        'rate_hz': (len(samples) - 1) / elapsed if elapsed > 0.0 and len(samples) > 1 else 0.0,
        'roll_deg': {'mean': _mean(rolls), 'std': _std(rolls)},
        'pitch_deg': {'mean': _mean(pitches), 'std': _std(pitches)},
        'yaw_deg': {'mean': _mean(yaws), 'std': _std(yaws)},
        'acceleration_norm_m_s2': {'mean': _mean(accel_norms), 'std': _std(accel_norms)},
        'gyro_norm_rad_s': {'mean': _mean(gyro_norms), 'std': _std(gyro_norms)},
    }
    out = {'summary': summary, 'samples': samples}
    out_dir = Path.cwd() / 'log' / 'imu_balance'
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / time.strftime(f'{pose}_%Y%m%d_%H%M%S.json')
    out_path.write_text(json.dumps(out, indent=2), encoding='utf-8')
    print(json.dumps(summary, indent=2))
    print(f'Wrote {out_path}')
    node.destroy_node()
    rclpy.shutdown()
