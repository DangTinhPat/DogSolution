"""Read-only Tkinter monitor for megaDog's real IMU pipeline."""

import json
import math
import os
import queue
import threading
import time
from pathlib import Path
from tkinter import scrolledtext, ttk
import tkinter as tk

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from main_bot_hardware_msgs.msg import ImuRaw
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException, ShutdownException
from sensor_msgs.msg import Imu


os.environ['ROS_DOMAIN_ID'] = os.environ.get('ROS_DOMAIN_ID', '0')

BG = '#1e1e1e'
BG_PANEL = '#2d2d2d'
BG_ENTRY = '#101010'
BORDER = '#3c3c3c'
FG = '#d4d4d4'
FG_MUTED = '#9d9d9d'
ACCENT_OK = '#2ea043'
ACCENT_WARN = '#bb8009'
ACCENT_ERROR = '#da3633'
LOG_BG = '#0c0c0c'
LOG_FG = '#d4d4d4'


def _now_seconds():
    return time.monotonic()


def _ros_stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _quat_to_rpy_deg(q):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return tuple(math.degrees(v) for v in (roll, pitch, yaw))


def _fmt_triplet(values, digits=3):
    return f'{values[0]: .{digits}f}, {values[1]: .{digits}f}, {values[2]: .{digits}f}'


def _uint8_to_int(value):
    if isinstance(value, (bytes, bytearray)):
        return value[0] if value else 0
    return int(value)


class TopicStats:
    def __init__(self):
        self.count = 0
        self.first_time = None
        self.last_time = None

    def mark(self):
        now = _now_seconds()
        if self.first_time is None:
            self.first_time = now
        self.last_time = now
        self.count += 1

    def hz(self):
        if self.first_time is None or self.last_time is None or self.last_time <= self.first_time:
            return 0.0
        return max(0.0, (self.count - 1) / (self.last_time - self.first_time))

    def age_ms(self):
        if self.last_time is None:
            return None
        return (_now_seconds() - self.last_time) * 1000.0


class ImuMonitorNode(Node):
    def __init__(self, out_queue):
        super().__init__('megadog_imu_monitor')
        self.out_queue = out_queue
        self.raw_stats = TopicStats()
        self.filtered_stats = TopicStats()
        self.create_subscription(ImuRaw, '/imu/raw', self._on_raw, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu/data', self._on_filtered, qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, '/diagnostics', self._on_diagnostics, 10)

    def _on_raw(self, msg):
        self.raw_stats.mark()
        accel = [float(v) / 1000.0 for v in msg.linear_acceleration_milli_ms2]
        gyro = [float(v) / 1000.0 for v in msg.angular_velocity_mrad_s]
        self.out_queue.put({
            'kind': 'raw',
            'stamp_wall': time.time(),
            'stamp_ms': int(msg.stamp_ms),
            'status': _uint8_to_int(msg.status),
            'accel_wire': [int(v) for v in msg.linear_acceleration_milli_ms2],
            'gyro_wire': [int(v) for v in msg.angular_velocity_mrad_s],
            'accel': accel,
            'gyro': gyro,
            'hz': self.raw_stats.hz(),
            'age_ms': self.raw_stats.age_ms(),
            'count': self.raw_stats.count,
        })

    def _on_filtered(self, msg):
        self.filtered_stats.mark()
        accel = [
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ]
        gyro = [
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ]
        rpy = _quat_to_rpy_deg(msg.orientation)
        self.out_queue.put({
            'kind': 'filtered',
            'stamp_wall': time.time(),
            'stamp_ros': _ros_stamp_seconds(msg.header.stamp),
            'frame_id': msg.header.frame_id,
            'accel': accel,
            'gyro': gyro,
            'accel_norm': math.sqrt(sum(v * v for v in accel)),
            'quat': [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z],
            'rpy_deg': list(rpy),
            'orientation_covariance': list(msg.orientation_covariance),
            'hz': self.filtered_stats.hz(),
            'age_ms': self.filtered_stats.age_ms(),
            'count': self.filtered_stats.count,
        })

    def _on_diagnostics(self, msg):
        selected = None
        for status in msg.status:
            if 'imu_kalman' in status.name:
                selected = status
                break
        if selected is None and msg.status:
            selected = msg.status[0]
        if selected is None:
            return
        self.out_queue.put({
            'kind': 'diagnostics',
            'stamp_wall': time.time(),
            'name': selected.name,
            'level': _uint8_to_int(selected.level),
            'message': selected.message,
            'values': {value.key: value.value for value in selected.values},
        })


class ImuMonitorGui:
    def __init__(self, root):
        self.root = root
        self.root.title('megaDog IMU Monitor')
        self.root.geometry('980x760')
        self.root.minsize(780, 560)
        self.queue = queue.Queue()
        self.raw = None
        self.filtered = None
        self.diagnostics = None
        self.last_log_time = 0.0
        self.log_file = self._open_log_file()
        self.labels = {}

        self._build_style()
        self._build_widgets()

        rclpy.init(args=None)
        self.node = ImuMonitorNode(self.queue)
        self.spin_thread = threading.Thread(target=self._spin_node, daemon=True)
        self.spin_thread.start()

        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        self._timer_id = self.root.after(100, self._poll_queue)
        self._append_log(f'IMU monitor started. Logging to {self.log_file.name}\n')

    def _spin_node(self):
        try:
            rclpy.spin(self.node)
        except (ExternalShutdownException, ShutdownException):
            pass

    def _open_log_file(self):
        log_dir = Path.cwd() / 'log' / 'imu_monitor'
        log_dir.mkdir(parents=True, exist_ok=True)
        path = log_dir / time.strftime('imu_%Y%m%d_%H%M%S.jsonl')
        return path.open('a', encoding='utf-8')

    def _build_style(self):
        self.root.configure(bg=BG)
        style = ttk.Style(self.root)
        try:
            style.theme_use('clam')
        except tk.TclError:
            pass
        style.configure('TFrame', background=BG)
        style.configure('TLabelframe', background=BG, bordercolor=BORDER, relief='flat')
        style.configure('TLabelframe.Label', background=BG, foreground=FG_MUTED)
        style.configure('TLabel', background=BG, foreground=FG)
        style.configure('Muted.TLabel', background=BG, foreground=FG_MUTED)
        style.configure('Header.TLabel', background=BG, foreground=FG, font=('TkDefaultFont', 15, 'bold'))
        style.configure('Value.TLabel', background=BG, foreground=FG, font=('DejaVu Sans Mono', 10))
        style.configure('OK.TLabel', background=ACCENT_OK, foreground='white', padding=(10, 5))
        style.configure('WARN.TLabel', background=ACCENT_WARN, foreground='white', padding=(10, 5))
        style.configure('ERROR.TLabel', background=ACCENT_ERROR, foreground='white', padding=(10, 5))
        style.configure('WAIT.TLabel', background=BG_PANEL, foreground='white', padding=(10, 5))

    def _build_widgets(self):
        header = ttk.Frame(self.root, padding=(16, 14, 16, 6))
        header.pack(fill='x')
        ttk.Label(header, text='megaDog IMU Monitor', style='Header.TLabel').pack(side='left')
        self.labels['status_badge'] = ttk.Label(header, text='WAITING', style='WAIT.TLabel')
        self.labels['status_badge'].pack(side='right')

        topics = ttk.Frame(self.root, padding=(16, 0, 16, 4))
        topics.pack(fill='x')
        ttk.Label(topics, text='ROS_DOMAIN_ID=0  |  raw: /imu/raw  |  filtered: /imu/data  |  diagnostics: /diagnostics',
                  style='Muted.TLabel').pack(side='left')

        grid = ttk.Frame(self.root, padding=(16, 8))
        grid.pack(fill='x')
        self._build_panel(grid, 'raw', 'RAW /imu/raw', 0, [
            ('raw_rate', 'rate / age'),
            ('raw_status', 'status'),
            ('raw_stamp', 'mcu stamp ms'),
            ('raw_accel_wire', 'accel wire'),
            ('raw_gyro_wire', 'gyro wire'),
            ('raw_accel', 'accel m/s2'),
            ('raw_gyro', 'gyro rad/s'),
        ])
        self._build_panel(grid, 'filtered', 'FILTERED /imu/data', 1, [
            ('filtered_rate', 'rate / age'),
            ('filtered_frame', 'frame'),
            ('filtered_rpy', 'rpy deg'),
            ('filtered_quat', 'quat wxyz'),
            ('filtered_accel', 'accel m/s2'),
            ('filtered_accel_norm', 'accel norm'),
            ('filtered_gyro', 'gyro rad/s'),
            ('filtered_cov', 'orientation cov'),
        ])
        grid.columnconfigure(0, weight=1)
        grid.columnconfigure(1, weight=1)

        diag = ttk.LabelFrame(self.root, text='DIAGNOSTICS', padding=(14, 10))
        diag.pack(fill='x', padx=16, pady=(0, 8))
        self._add_row(diag, 0, 'diag_name', 'name')
        self._add_row(diag, 1, 'diag_message', 'message')
        self._add_row(diag, 2, 'diag_state', 'state')
        self._add_row(diag, 3, 'diag_counts', 'samples')
        self._add_row(diag, 4, 'diag_bias', 'gyro bias rad/s')
        self._add_row(diag, 5, 'diag_mounting', 'mounting / accel scale')

        log_frame = ttk.LabelFrame(self.root, text='5 HZ MERGED LOG', padding=(1, 1))
        log_frame.pack(fill='both', expand=True, padx=16, pady=(0, 16))
        self.log_text = scrolledtext.ScrolledText(
            log_frame, wrap='none', font=('DejaVu Sans Mono', 10),
            bg=LOG_BG, fg=LOG_FG, insertbackground=LOG_FG,
            selectbackground='#264f78', selectforeground='white',
            relief='flat', borderwidth=0, padx=10, pady=8)
        self.log_text.pack(fill='both', expand=True)
        self.log_text.bind('<Key>', self._block_edit)

    def _build_panel(self, parent, prefix, title, column, rows):
        panel = ttk.LabelFrame(parent, text=title, padding=(14, 10))
        panel.grid(row=0, column=column, sticky='nsew', padx=(0 if column == 0 else 8, 8 if column == 0 else 0))
        for row, (key, label) in enumerate(rows):
            self._add_row(panel, row, key, label)

    def _add_row(self, parent, row, key, label):
        ttk.Label(parent, text=label, style='Muted.TLabel', width=18).grid(
            row=row, column=0, sticky='w', pady=2)
        value = ttk.Label(parent, text='-', style='Value.TLabel')
        value.grid(row=row, column=1, sticky='w', pady=2)
        self.labels[key] = value

    def _block_edit(self, event):
        ctrl_held = bool(event.state & 0x4)
        if ctrl_held and event.keysym.lower() in ('c', 'a'):
            return None
        if event.keysym in ('Left', 'Right', 'Up', 'Down', 'Prior', 'Next', 'Home', 'End'):
            return None
        return 'break'

    def _append_log(self, text):
        self.log_text.insert('end', text)
        self.log_text.see('end')

    def _poll_queue(self):
        try:
            while True:
                item = self.queue.get_nowait()
                kind = item.get('kind')
                if kind == 'raw':
                    self.raw = item
                elif kind == 'filtered':
                    self.filtered = item
                elif kind == 'diagnostics':
                    self.diagnostics = item
        except queue.Empty:
            pass

        self._refresh_labels()
        now = _now_seconds()
        if now - self.last_log_time >= 0.2:
            self.last_log_time = now
            self._write_merged_log()
        self._timer_id = self.root.after(100, self._poll_queue)

    def _refresh_labels(self):
        if self.raw is not None:
            age = self.raw.get('age_ms')
            age_text = '-' if age is None else f'{age:.0f} ms'
            self.labels['raw_rate'].configure(text=f"{self.raw['hz']:.1f} Hz / {age_text}")
            self.labels['raw_status'].configure(text=self._raw_status_text(self.raw['status']))
            self.labels['raw_stamp'].configure(text=str(self.raw['stamp_ms']))
            self.labels['raw_accel_wire'].configure(text=str(self.raw['accel_wire']))
            self.labels['raw_gyro_wire'].configure(text=str(self.raw['gyro_wire']))
            self.labels['raw_accel'].configure(text=_fmt_triplet(self.raw['accel']))
            self.labels['raw_gyro'].configure(text=_fmt_triplet(self.raw['gyro']))

        if self.filtered is not None:
            age = self.filtered.get('age_ms')
            age_text = '-' if age is None else f'{age:.0f} ms'
            self.labels['filtered_rate'].configure(text=f"{self.filtered['hz']:.1f} Hz / {age_text}")
            self.labels['filtered_frame'].configure(text=self.filtered['frame_id'] or '-')
            self.labels['filtered_rpy'].configure(text=_fmt_triplet(self.filtered['rpy_deg'], digits=2))
            self.labels['filtered_quat'].configure(text=', '.join(f'{v:.4f}' for v in self.filtered['quat']))
            self.labels['filtered_accel'].configure(text=_fmt_triplet(self.filtered['accel']))
            self.labels['filtered_accel_norm'].configure(text=f"{self.filtered['accel_norm']:.3f} m/s2")
            self.labels['filtered_gyro'].configure(text=_fmt_triplet(self.filtered['gyro']))
            cov = self.filtered['orientation_covariance']
            self.labels['filtered_cov'].configure(text=f'{cov[0]:.4g}, {cov[4]:.4g}, {cov[8]:.4g}')

        if self.diagnostics is not None:
            values = self.diagnostics.get('values', {})
            level = self.diagnostics.get('level', 3)
            self.labels['status_badge'].configure(
                text=self._diag_level_text(level),
                style=self._diag_style(level))
            self.labels['diag_name'].configure(text=self.diagnostics.get('name', '-'))
            self.labels['diag_message'].configure(text=self.diagnostics.get('message', '-'))
            self.labels['diag_state'].configure(text=values.get('state', '-'))
            counts = (
                f"calib {values.get('calibration_samples', '-')}/{values.get('calibration_target', '-')}  "
                f"rx {values.get('received_samples', '-')}  "
                f"pub {values.get('published_samples', '-')}  "
                f"reject {values.get('rejected_samples', '-')}"
            )
            self.labels['diag_counts'].configure(text=counts)
            self.labels['diag_bias'].configure(text=values.get('gyro_bias_rad_s', '-'))
            mounting = (
                f"rpy {values.get('sensor_to_body_rpy_rad', '-')}  "
                f"scale {values.get('accel_scale', '-')}"
            )
            self.labels['diag_mounting'].configure(text=mounting)

    def _write_merged_log(self):
        entry = {
            'wall_time': time.time(),
            'raw': self.raw,
            'filtered': self.filtered,
            'diagnostics': self.diagnostics,
        }
        self.log_file.write(json.dumps(entry, separators=(',', ':')) + '\n')
        self.log_file.flush()
        raw_text = 'RAW waiting'
        if self.raw is not None:
            raw_text = (
                f"RAW {self.raw['hz']:.1f}Hz status={self._raw_status_text(self.raw['status'])} "
                f"acc=[{_fmt_triplet(self.raw['accel'])}] gyro=[{_fmt_triplet(self.raw['gyro'])}]"
            )
        filtered_text = 'FILTERED waiting'
        if self.filtered is not None:
            filtered_text = (
                f"FILTERED {self.filtered['hz']:.1f}Hz rpy=[{_fmt_triplet(self.filtered['rpy_deg'], 2)}] "
                f"|a|={self.filtered['accel_norm']:.3f}"
            )
        diag_text = 'DIAG waiting'
        if self.diagnostics is not None:
            diag_text = f"DIAG {self._diag_level_text(self.diagnostics.get('level', 3))} {self.diagnostics.get('message', '')}"
        self._append_log(time.strftime('%H:%M:%S ') + raw_text + ' | ' + filtered_text + ' | ' + diag_text + '\n')

    @staticmethod
    def _raw_status_text(status):
        names = []
        if status & ImuRaw.STATUS_OK:
            names.append('OK')
        if status & ImuRaw.STATUS_INIT_FAILED:
            names.append('INIT_FAILED')
        if status & ImuRaw.STATUS_READ_FAILED:
            names.append('READ_FAILED')
        return '|'.join(names) if names else f'0x{status:02x}'

    @staticmethod
    def _diag_level_text(level):
        return {0: 'OK', 1: 'WARN', 2: 'ERROR'}.get(level, 'WAITING')

    @staticmethod
    def _diag_style(level):
        return {0: 'OK.TLabel', 1: 'WARN.TLabel', 2: 'ERROR.TLabel'}.get(level, 'WAIT.TLabel')

    def _on_close(self):
        try:
            self.root.after_cancel(self._timer_id)
        except tk.TclError:
            pass
        self.node.destroy_node()
        rclpy.shutdown()
        self.log_file.close()
        self.root.destroy()


def main():
    root = tk.Tk()
    ImuMonitorGui(root)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
