"""Tkinter control panel for the megadog_description Gazebo sim, RViz view,
and megadog_controller's FSM (Home/Stand/Trot-in-place/Forward/Backward).

Process-control mechanism (subprocess.Popen'd `ros2 launch`, SIGINT->SIGTERM
escalation via psutil, log panel, kill/shutdown buttons) follows the same
general shape as other ROS2 sim control panels; no joystick_bridge package or
real-hardware launch exist for megaDog yet, so those rows are simply absent
rather than stubbed out.

Replaces the "open one terminal per task" workflow (launch the sim in one
terminal, RViz in another, publish FSM commands from a third, kill leftover
gz/ros processes in a fourth, ...) with a single window. sim.launch.py's own
bundled RViz (rviz:=true by default) already shows megadog_wbc's OCS2
desired/optimized trajectory and contact markers; this panel's separate RViz
button (rz_sim.launch.py) is for closing/reopening that view without
restarting the whole simulation. The FSM row publishes one-shot commands to
"/megadog/cmd" (see MegadogController.cpp) - MegadogController itself owns
the actual state machine.
"""

import os
import queue
import signal
import subprocess
import threading
import time
import tkinter as tk
import tkinter.font as tkfont
from tkinter import scrolledtext, ttk

import psutil

STOP_GRACE_SECONDS = 5

BG = '#1e1e1e'
BG_PANEL = '#2d2d2d'
BG_ENTRY = '#3c3c3c'
BORDER = '#3c3c3c'
FG = '#d4d4d4'
FG_MUTED = '#9d9d9d'
ACCENT_START = '#2ea043'
ACCENT_START_HOVER = '#3fb950'
ACCENT_STOP = '#da3633'
ACCENT_STOP_HOVER = '#f85149'
ACCENT_KILL = '#9e6a03'
ACCENT_KILL_HOVER = '#bb8009'
LOG_BG = '#0c0c0c'
LOG_FG = '#d4d4d4'

_STATUS_STYLES = {
    'idle': 'BadgeIdle.TLabel',
    'running': 'BadgeRunning.TLabel',
    'stopping': 'BadgeStopping.TLabel',
}


class SimControlGui:

    def __init__(self, root):
        self.root = root
        self.root.title('megaDog Control')
        self.root.geometry('900x680')
        self.root.minsize(700, 480)

        self.procs = {'sim': None, 'rviz': None}
        self.proc_widgets = {}
        self.fsm_widgets = []
        self.force_widgets = []
        self.log_queue = queue.Queue()

        self._build_widgets()
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        # ID of the recurring after() timer, so _on_close can cancel it
        # explicitly - otherwise it can fire after root.destroy() and Tk
        # raises "invalid command name ..." trying to run the dead callback.
        self._poll_log_after_id = self.root.after(100, self._poll_log_queue)

    def _setup_style(self):
        self.root.configure(bg=BG)
        tkfont.nametofont('TkDefaultFont').configure(family='TkDefaultFont', size=10)

        style = ttk.Style(self.root)
        try:
            style.theme_use('clam')
        except tk.TclError:
            pass

        style.configure('TFrame', background=BG)
        style.configure('TLabelframe', background=BG, bordercolor=BORDER, relief='flat')
        style.configure('TLabelframe.Label', background=BG, foreground=FG_MUTED,
                         font=('TkDefaultFont', 9, 'bold'))
        style.configure('TLabel', background=BG, foreground=FG)
        style.configure('Header.TLabel', background=BG, foreground=FG,
                         font=('TkDefaultFont', 15, 'bold'))
        style.configure('RowLabel.TLabel', background=BG, foreground=FG_MUTED,
                         font=('TkDefaultFont', 10, 'bold'))
        style.configure('TEntry', fieldbackground=BG_ENTRY, foreground=FG,
                         insertcolor=FG, bordercolor=BORDER,
                         lightcolor=BG_ENTRY, darkcolor=BG_ENTRY, padding=5)
        style.configure('TCombobox', fieldbackground=BG_ENTRY, foreground=FG,
                         background=BG_ENTRY, arrowcolor=FG, bordercolor=BORDER,
                         lightcolor=BG_ENTRY, darkcolor=BG_ENTRY, padding=5)
        style.configure('TSpinbox', fieldbackground=BG_ENTRY, foreground=FG,
                         insertcolor=FG, bordercolor=BORDER,
                         lightcolor=BG_ENTRY, darkcolor=BG_ENTRY, arrowsize=13,
                         padding=5)

        style.configure('TButton', background=BG_PANEL, foreground=FG,
                         font=('TkDefaultFont', 10), padding=(14, 8), borderwidth=0,
                         focusthickness=0)
        style.map('TButton',
                  background=[('active', '#3c3c3c'), ('disabled', '#2a2a2a')],
                  foreground=[('disabled', '#6a6a6a')])

        for name, base, hover in (
            ('Start.TButton', ACCENT_START, ACCENT_START_HOVER),
            ('Stop.TButton', ACCENT_STOP, ACCENT_STOP_HOVER),
            ('Kill.TButton', ACCENT_KILL, ACCENT_KILL_HOVER),
        ):
            style.configure(name, background=base, foreground='white',
                             font=('TkDefaultFont', 10, 'bold'), padding=(14, 8),
                             borderwidth=0, focusthickness=0)
            style.map(name,
                      background=[('active', hover), ('disabled', '#3c3c3c')],
                      foreground=[('disabled', '#6a6a6a')])

        for name, bg in (
            ('BadgeIdle.TLabel', '#3c3c3c'),
            ('BadgeRunning.TLabel', ACCENT_START),
            ('BadgeStopping.TLabel', ACCENT_KILL),
        ):
            style.configure(name, background=bg, foreground='white',
                             font=('TkDefaultFont', 10, 'bold'), padding=(10, 5))

    def _build_widgets(self):
        self._setup_style()

        header = ttk.Frame(self.root, padding=(16, 14, 16, 4))
        header.pack(fill='x')
        ttk.Label(header, text='megaDog Control', style='Header.TLabel').pack(side='left')

        params = ttk.LabelFrame(self.root, text='SPAWN PARAMETERS', padding=(14, 10))
        params.pack(fill='x', padx=16, pady=(10, 6))

        self.world_var = tk.StringVar(value='')
        self.robot_name_var = tk.StringVar(value='a1')
        self.x_var = tk.StringVar(value='0.0')
        self.y_var = tk.StringVar(value='0.0')
        self.z_var = tk.StringVar(value='0.5')

        fields = [
            ('World (blank = default)', self.world_var, 16),
            ('Robot name', self.robot_name_var, 10),
            ('X', self.x_var, 6),
            ('Y', self.y_var, 6),
            ('Z', self.z_var, 6),
        ]
        for col, (label, var, width) in enumerate(fields):
            ttk.Label(params, text=label).grid(
                row=0, column=2 * col, padx=(0 if col == 0 else 4, 6), pady=4, sticky='w')
            ttk.Entry(params, textvariable=var, width=width).grid(
                row=0, column=2 * col + 1, padx=(0, 14), pady=4, sticky='w')

        sim_row = ttk.Frame(self.root, padding=(16, 4))
        sim_row.pack(fill='x')
        ttk.Label(sim_row, text='Sim', style='RowLabel.TLabel', width=6).pack(side='left')
        self._build_process_controls(
            sim_row, key='sim', start_text='▶  Start sim', stop_text='■  Stop sim')
        ttk.Label(
            sim_row, foreground=FG_MUTED,
            text='  (Gazebo + megadog_controller: OCS2 SqpMpc + HierarchicalWbc)'
        ).pack(side='left')

        rviz_row = ttk.Frame(self.root, padding=(16, 4))
        rviz_row.pack(fill='x')
        ttk.Label(rviz_row, text='RViz', style='RowLabel.TLabel', width=6).pack(side='left')
        self._build_process_controls(
            rviz_row, key='rviz', start_text='▶  Open RViz', stop_text='■  Close RViz')
        ttk.Label(
            rviz_row, foreground=FG_MUTED,
            text=(
                '  (RobotModel/TF + OCS2 desired/optimized trajectory + contact markers - '
                'run alongside Sim above)'
            )
        ).pack(side='left')

        self._build_fsm_panel()
        self._build_force_panel()

        util_row = ttk.Frame(self.root, padding=(16, 4))
        util_row.pack(fill='x')
        ttk.Label(util_row, text='', width=6).pack(side='left')
        self.kill_button = ttk.Button(
            util_row, text='Kill gz/ros traces', style='Kill.TButton', command=self.kill_traces)
        self.kill_button.pack(side='left')

        self.shutdown_button = ttk.Button(
            util_row, text='⏻  Shutdown all & Quit', style='Stop.TButton',
            command=self.shutdown_everything)
        self.shutdown_button.pack(side='left', padx=(10, 0))

        log_frame = ttk.LabelFrame(self.root, text='LOG', padding=(1, 1))
        log_frame.pack(fill='both', expand=True, padx=16, pady=(6, 16))

        # Kept in 'normal' state (not 'disabled') so mouse selection and Ctrl+C
        # copy work reliably across Tk versions; _block_edit below blocks
        # actual typing while still letting programmatic .insert() calls through.
        self.log_text = scrolledtext.ScrolledText(
            log_frame, wrap='word', font=('DejaVu Sans Mono', 10),
            bg=LOG_BG, fg=LOG_FG, insertbackground=LOG_FG,
            selectbackground='#264f78', selectforeground='white',
            relief='flat', borderwidth=0, padx=10, pady=8)
        self.log_text.pack(fill='both', expand=True)
        self.log_text.bind('<Key>', self._block_edit)
        self.log_text.vbar.configure(
            bg=BG_PANEL, troughcolor=BG, activebackground=BORDER,
            bd=0, highlightthickness=0)

    def _build_process_controls(self, parent, key, start_text, stop_text):
        start_button = ttk.Button(
            parent, text=start_text, style='Start.TButton',
            command=lambda: self.start_process(key))
        start_button.pack(side='left')

        stop_button = ttk.Button(
            parent, text=stop_text, style='Stop.TButton',
            command=lambda: self.stop_process(key), state='disabled')
        stop_button.pack(side='left', padx=(10, 0))

        status_label = ttk.Label(parent, style='BadgeIdle.TLabel')
        status_label.pack(side='right')

        self.proc_widgets[key] = {'start': start_button, 'stop': stop_button, 'status': status_label}
        self._set_badge(status_label, 'Idle', 'idle')

    def _build_fsm_panel(self):
        # Publishes to megadog_controller's "/megadog/cmd" (std_msgs/String)
        # topic - see MegadogController.cpp's on_configure() subscription for
        # the accepted values. HOME (zero effort, robot rests under gravity)
        # is the state on controller activation; the other four are
        # one-shot commands that switch the WBC's active gait/velocity and
        # keep running until the next command changes it.
        fsm_frame = ttk.LabelFrame(
            self.root, text='FSM (needs Sim running)', padding=(14, 10))
        fsm_frame.pack(fill='x', padx=16, pady=(6, 6))

        fsm_row = ttk.Frame(fsm_frame)
        fsm_row.pack(fill='x')
        buttons = [
            ('Home (nằm xấp/init)', 'home'),
            ('Đứng lên', 'stand'),
            ('Trot tiến', 'forward'),
            ('Trot tại chỗ', 'trot_in_place'),
            ('Đi lùi', 'backward'),
        ]
        for label, value in buttons:
            button = ttk.Button(fsm_row, text=label, command=lambda v=value: self._publish_cmd(v))
            button.pack(side='left', padx=(0 if value == 'home' else 8, 0))
            self.fsm_widgets.append(button)
        self._set_fsm_controls_enabled(False)

    def _build_force_panel(self):
        force_frame = ttk.LabelFrame(
            self.root, text='FORCE TEST (needs Sim running)', padding=(14, 10))
        force_frame.pack(fill='x', padx=16, pady=(0, 6))

        force_row = ttk.Frame(force_frame)
        force_row.pack(fill='x')

        self.force_direction_var = tk.StringVar(value='+X forward')
        self.force_newton_var = tk.DoubleVar(value=5.0)
        self.force_newton_label_var = tk.StringVar(value='5 N')
        self.force_duration_ms_var = tk.StringVar(value='200')

        ttk.Label(force_row, text='Direction').pack(side='left')
        direction_box = ttk.Combobox(
            force_row, textvariable=self.force_direction_var, width=12,
            values=('+X forward', '-X backward', '+Y left', '-Y right', '+Z up'),
            state='readonly')
        direction_box.pack(side='left', padx=(6, 14))

        ttk.Label(force_row, text='Force').pack(side='left')
        force_slider = ttk.Scale(
            force_row, from_=1, to=30, orient='horizontal',
            variable=self.force_newton_var, command=self._on_force_slider_changed)
        force_slider.pack(side='left', fill='x', expand=True, padx=(6, 8))
        force_value_label = ttk.Label(force_row, textvariable=self.force_newton_label_var, width=7)
        force_value_label.pack(side='left', padx=(0, 14))

        ttk.Label(force_row, text='Duration ms').pack(side='left')
        duration_spin = ttk.Spinbox(
            force_row, from_=50, to=1000, increment=50, width=7,
            textvariable=self.force_duration_ms_var)
        duration_spin.pack(side='left', padx=(6, 14))

        push_button = ttk.Button(force_row, text='Apply push', command=self.apply_force)
        push_button.pack(side='left')

        self.force_widgets.extend([
            direction_box, force_slider, force_value_label, duration_spin, push_button])
        self._set_force_controls_enabled(False)

    def _set_fsm_controls_enabled(self, enabled):
        state = 'normal' if enabled else 'disabled'
        for widget in self.fsm_widgets:
            widget.configure(state=state)

    def _set_force_controls_enabled(self, enabled):
        for widget in self.force_widgets:
            if isinstance(widget, ttk.Combobox):
                widget.configure(state='readonly' if enabled else 'disabled')
            else:
                widget.configure(state='normal' if enabled else 'disabled')

    def _on_force_slider_changed(self, value):
        self.force_newton_label_var.set(f'{float(value):.0f} N')

    def _publish_cmd(self, value):
        self._append_log(f"$ ros2 topic pub --once /megadog/cmd std_msgs/msg/String \"{{data: {value}}}\"\n")
        threading.Thread(target=self._run_publish_cmd, args=(value,), daemon=True).start()

    def _run_publish_cmd(self, value):
        try:
            result = subprocess.run(
                ['ros2', 'topic', 'pub', '--once', '/megadog/cmd', 'std_msgs/msg/String', '{data: ' + value + '}'],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=10)
            self.log_queue.put(('log', None, result.stdout))
        except (OSError, subprocess.TimeoutExpired) as exc:
            self.log_queue.put(('log', None, f'Failed to publish cmd: {exc}\n'))

    def apply_force(self):
        try:
            force_n = float(self.force_newton_var.get())
            duration_ms = int(float(self.force_duration_ms_var.get()))
        except ValueError:
            self._append_log('Force test ignored: force and duration must be numbers.\n')
            return
        force_n = max(0.0, min(force_n, 30.0))
        duration_ms = max(1, min(duration_ms, 5000))
        self.force_newton_var.set(force_n)
        self.force_newton_label_var.set(f'{force_n:.0f} N')
        self.force_duration_ms_var.set(str(duration_ms))

        direction = self.force_direction_var.get()
        axis_forces = {
            '+X forward': (force_n, 0.0, 0.0),
            '-X backward': (-force_n, 0.0, 0.0),
            '+Y left': (0.0, force_n, 0.0),
            '-Y right': (0.0, -force_n, 0.0),
            '+Z up': (0.0, 0.0, force_n),
        }
        force_xyz = axis_forces.get(direction, (force_n, 0.0, 0.0))
        robot_name = self.robot_name_var.get().strip() or 'a1'
        threading.Thread(
            target=self._run_apply_force,
            args=(robot_name, force_xyz, duration_ms),
            daemon=True).start()

    def _run_apply_force(self, robot_name, force_xyz, duration_ms):
        force_x, force_y, force_z = force_xyz
        # Gazebo's ApplyLinkWrench resolves the spawned URDF link by its link
        # entity name in this world; /pose/info shows it as "base".
        target_link = 'base'
        request = (
            f'entity: {{name: "{target_link}", type: LINK}} '
            f'wrench: {{force: {{x: {force_x:g}, y: {force_y:g}, z: {force_z:g}}}}}'
        )
        apply_cmd = [
            'gz', 'topic', '-t', '/world/megadog_world/wrench/persistent',
            '-m', 'gz.msgs.EntityWrench',
            '-p', request,
        ]
        clear_cmd = [
            'gz', 'topic', '-t', '/world/megadog_world/wrench/clear',
            '-m', 'gz.msgs.Entity',
            '-p', f'name: "{target_link}", type: LINK',
        ]
        self.log_queue.put((
            'log', None,
            f'Applying {force_x:g} {force_y:g} {force_z:g} N to {target_link} '
            f'for {duration_ms} ms\n'
        ))
        self.log_queue.put(('log', None, '$ ' + ' '.join(apply_cmd) + '\n'))
        try:
            result = subprocess.run(
                apply_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, timeout=2)
            if result.stdout:
                self.log_queue.put(('log', None, result.stdout))
            if result.returncode != 0:
                self.log_queue.put(('log', None, f'Force publish failed with code {result.returncode}\n'))
                return
            time.sleep(duration_ms / 1000.0)
        except (OSError, subprocess.TimeoutExpired) as exc:
            self.log_queue.put(('log', None, f'Failed to apply force: {exc}\n'))
        finally:
            self.log_queue.put(('log', None, '$ ' + ' '.join(clear_cmd) + '\n'))
            try:
                result = subprocess.run(
                    clear_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, timeout=2)
                if result.stdout:
                    self.log_queue.put(('log', None, result.stdout))
                if result.returncode == 0:
                    self.log_queue.put(('log', None, 'Force cleared.\n'))
                else:
                    self.log_queue.put(('log', None, f'Force clear failed with code {result.returncode}\n'))
            except (OSError, subprocess.TimeoutExpired) as exc:
                self.log_queue.put(('log', None, f'Failed to clear force: {exc}\n'))

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

    def _set_badge(self, label, text, kind):
        label.configure(text='●  ' + text, style=_STATUS_STYLES[kind])

    def _command_for(self, key):
        if key == 'sim':
            cmd = [
                'ros2', 'launch', 'megadog_description', 'sim.launch.py',
                'robot_name:=' + self.robot_name_var.get(),
                'x:=' + self.x_var.get(),
                'y:=' + self.y_var.get(),
                'z:=' + self.z_var.get(),
                # sim.launch.py opens its own RViz by default; the GUI keeps a
                # separate dedicated RViz row/button, so skip the bundled one
                # here to avoid two RViz windows opening at once.
                'rviz:=false',
            ]
            world = self.world_var.get().strip()
            if world:
                cmd.append('world:=' + world)
            return cmd
        if key == 'rviz':
            return ['ros2', 'launch', 'megadog_description', 'rz_sim.launch.py', 'use_sim_time:=true']
        raise ValueError(key)

    def start_process(self, key):
        if self.procs[key] is not None:
            return

        cmd = self._command_for(key)
        self._append_log(f'$ [{key}] ' + ' '.join(cmd) + '\n')

        try:
            proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1, start_new_session=True)
        except OSError as exc:
            self._append_log(f'Failed to start {key}: {exc}\n')
            return

        self.procs[key] = proc
        widgets = self.proc_widgets[key]
        widgets['start'].configure(state='disabled')
        widgets['stop'].configure(state='normal')
        self._set_badge(widgets['status'], f'Running (pid {proc.pid})', 'running')
        if key == 'sim':
            self._set_fsm_controls_enabled(True)
            self._set_force_controls_enabled(True)

        threading.Thread(target=self._read_proc_output, args=(key, proc), daemon=True).start()

    def _read_proc_output(self, key, proc):
        for line in proc.stdout:
            self.log_queue.put(('log', key, line))
        returncode = proc.wait()
        self.log_queue.put(('exited', key, returncode))

    def _poll_log_queue(self):
        try:
            while True:
                kind, key, payload = self.log_queue.get_nowait()
                if kind == 'log':
                    self._append_log(payload)
                elif kind == 'exited':
                    self._append_log(f'--- [{key}] exited (code {payload}) ---\n')
                    self.procs[key] = None
                    widgets = self.proc_widgets[key]
                    widgets['start'].configure(state='normal')
                    widgets['stop'].configure(state='disabled')
                    self._set_badge(widgets['status'], 'Idle', 'idle')
                    if key == 'sim':
                        self._set_fsm_controls_enabled(False)
                        self._set_force_controls_enabled(False)
                elif kind == 'kill_done':
                    self.kill_button.configure(state='normal')
        except queue.Empty:
            pass
        self._poll_log_after_id = self.root.after(100, self._poll_log_queue)

    def stop_process(self, key):
        proc = self.procs[key]
        if proc is None:
            return
        self._set_badge(self.proc_widgets[key]['status'], 'Stopping...', 'stopping')
        pid = proc.pid
        self._send_signal_to_group(pid, signal.SIGINT)
        self.root.after(STOP_GRACE_SECONDS * 1000, lambda: self._escalate_stop(key, pid))

    def _escalate_stop(self, key, pid):
        proc = self.procs[key]
        if proc is None or proc.pid != pid:
            return
        self._append_log(f'--- [{key}] still alive after SIGINT, sending SIGTERM ---\n')
        self._send_signal_to_group(pid, signal.SIGTERM)

    def _send_signal_to_group(self, pid, sig):
        # os.killpg CHỈ hoạt động nếu mọi tiến trình con vẫn còn trong CÙNG
        # process group với pid gốc - nhưng `ros2 launch` tự đặt các tiến
        # trình nó sinh ra (ros2_control_node, robot_state_publisher,
        # spawner) vào session/process-group RIÊNG (chính nó làm vậy để
        # Ctrl+C ngoài terminal không giết con trước khi launch kịp đồng bộ
        # tắt hết) - nên chỉ killpg(pid gốc) là KHÔNG ĐỦ, để sót cả cây tiến
        # trình con sống ngầm. Dùng psutil đi hết cả cây con (đệ quy, không
        # phụ thuộc process group) rồi gửi tín hiệu riêng cho TỪNG tiến
        # trình, cộng với killpg như lớp dự phòng. Same pattern babyDog's
        # gui/main_window.py uses.
        try:
            children = psutil.Process(pid).children(recursive=True)
        except psutil.NoSuchProcess:
            children = []
        for child in children:
            try:
                child.send_signal(sig)
            except psutil.NoSuchProcess:
                pass
        try:
            os.killpg(os.getpgid(pid), sig)
        except ProcessLookupError:
            pass

    def kill_traces(self):
        self.kill_button.configure(state='disabled')
        threading.Thread(target=self._run_kill_traces, daemon=True).start()

    def _run_kill_traces(self):
        self.log_queue.put(('log', None, '$ ros2 run megadog_description kill_gz.sh\n'))
        try:
            result = subprocess.run(
                ['ros2', 'run', 'megadog_description', 'kill_gz.sh'],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            self.log_queue.put(('log', None, result.stdout))
        except OSError as exc:
            self.log_queue.put(('log', None, f'Failed to run kill_gz.sh: {exc}\n'))
        self.log_queue.put(('kill_done', None, None))

    def shutdown_everything(self):
        """One-click full shutdown: stop every running launch, sweep any stray
        gz/ros processes with kill_gz.sh, then close the app."""
        self.shutdown_button.configure(state='disabled', text='Shutting down...')
        self.kill_button.configure(state='disabled')
        for proc in self.procs.values():
            if proc is not None:
                self._send_signal_to_group(proc.pid, signal.SIGINT)
        # Poll instead of a blind STOP_GRACE_SECONDS wait - most of the time
        # every launch has already exited well before the grace period.
        self._shutdown_deadline = time.monotonic() + STOP_GRACE_SECONDS
        self._wait_for_shutdown()

    def _wait_for_shutdown(self):
        if all(proc is None for proc in self.procs.values()):
            self._finish_shutdown()
            return
        if time.monotonic() >= self._shutdown_deadline:
            self._finish_shutdown()
            return
        self.root.after(200, self._wait_for_shutdown)

    def _finish_shutdown(self):
        try:
            subprocess.run(
                ['ros2', 'run', 'megadog_description', 'kill_gz.sh'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)
        except (OSError, subprocess.TimeoutExpired):
            pass
        self._on_close()

    def _on_close(self):
        for proc in self.procs.values():
            if proc is not None:
                self._send_signal_to_group(proc.pid, signal.SIGINT)
        # Cancel the recurring timer explicitly - otherwise it can fire after
        # root.destroy() below and Tk raises "invalid command name ..." trying
        # to run a callback tied to a now-dead widget.
        self.root.after_cancel(self._poll_log_after_id)
        self.root.destroy()


def main():
    root = tk.Tk()
    gui = SimControlGui(root)
    try:
        root.mainloop()
    finally:
        # Last-resort safety net - if the GUI exits ABNORMALLY (Ctrl+C in the
        # terminal, terminal closed, killed from outside) instead of via the
        # X button/"Shutdown all & Quit" (both call _on_close() above, which
        # already stopped child processes), _on_close() does NOT run, leaving
        # child processes (sim/rviz) alive in the background. Harmless to
        # call again if _on_close() already ran (self.procs is all None by
        # then, the loop below does nothing).
        for proc in gui.procs.values():
            if proc is not None:
                gui._send_signal_to_group(proc.pid, signal.SIGINT)


if __name__ == '__main__':
    main()
