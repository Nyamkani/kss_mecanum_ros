"""Tkinter control panel; all widgets are accessed only from the main thread."""

import math
import queue
import re
import time
import tkinter as tk
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText

from robot_client import DEFAULT_PORT, RobotClient
from map_view import MapView

UI_INTERVAL_MS = 100
MAX_LOG_LINES = 200
MANUAL_REPEAT_MS = 100


class MecanumControl:
    def __init__(self, root, client=None):
        self.root = root
        self.client = client or RobotClient()
        self.pending = {}
        self.mode = None
        self.navigation_state = "IDLE"
        self._drive = None
        self._drive_after = None
        self._map_pose = None
        self._was_connected = False
        self._had_state = False
        self.root.title('Mecanum Control')
        self.root.minsize(1050, 760)
        self.root.bind('<ButtonRelease-1>', lambda event: self._end_drive())
        self.root.bind('<FocusOut>', lambda event: self.root.after_idle(self._focus_check))
        self.root.bind('<Unmap>', lambda event: self._end_drive() if event.widget is self.root else None)
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.host = tk.StringVar(value='')
        self.port = tk.StringVar(value=str(DEFAULT_PORT))
        self.map_name = tk.StringVar(value='room_01')
        self.values = {key: tk.StringVar(value='—') for key in
                       ('base_ready', 'mode', 'x', 'y', 'yaw', 'vx', 'vy', 'wz', 'motion', 'error', 'navigation')}
        self.connection = tk.StringVar(value='DISCONNECTED')
        self.freshness = tk.StringVar(value='No robot_state received')
        outer = ttk.Frame(root, padding=16)
        outer.grid(row=0, column=0, sticky='nsew')
        root.rowconfigure(0, weight=1)
        root.columnconfigure(0, weight=1)
        root.columnconfigure(1, weight=2)
        self.map_view = MapView(root)
        self.map_view.grid(row=0, column=1, sticky="nsew", padx=(0, 12), pady=16)
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(6, weight=1)

        network = ttk.LabelFrame(outer, text='Gateway connection', padding=10)
        network.grid(row=0, column=0, sticky='ew')
        network.columnconfigure(1, weight=1)
        ttk.Label(network, text='Robot IP').grid(row=0, column=0, padx=4)
        ttk.Entry(network, textvariable=self.host, width=22).grid(row=0, column=1, sticky='ew')
        ttk.Label(network, text='Port').grid(row=0, column=2, padx=4)
        ttk.Entry(network, textvariable=self.port, width=7).grid(row=0, column=3)
        self.connect_button = ttk.Button(network, text='Connect', command=self.connect)
        self.connect_button.grid(row=0, column=4, padx=4)
        self.disconnect_button = ttk.Button(network, text='Disconnect', command=self.disconnect)
        self.disconnect_button.grid(row=0, column=5)
        self.status_label = ttk.Label(network, textvariable=self.connection, foreground='#a32121')
        self.status_label.grid(row=1, column=0, columnspan=2, sticky='w', pady=(10, 0))
        ttk.Label(network, textvariable=self.freshness).grid(row=1, column=2, columnspan=4, sticky='w', pady=(10, 0))

        state = ttk.LabelFrame(outer, text='Robot state', padding=10)
        state.grid(row=1, column=0, sticky='ew', pady=10)
        for row, (label, key) in enumerate((('Base Ready', 'base_ready'), ('Mode', 'mode'), ('Motion', 'motion'))):
            ttk.Label(state, text=label).grid(row=row, column=0, sticky='w', padx=6)
            ttk.Label(state, textvariable=self.values[key]).grid(row=row, column=1, sticky='w', padx=12)
        for col, (title, keys, labels) in enumerate((
                ('Pose', ('x', 'y', 'yaw'), ('X', 'Y', 'Yaw')),
                ('Velocity', ('vx', 'vy', 'wz'), ('Vx', 'Vy', 'Wz'))), start=2):
            frame = ttk.LabelFrame(state, text=title, padding=6)
            frame.grid(row=0, column=col, rowspan=3, sticky='nsew', padx=10)
            state.columnconfigure(col, weight=1)
            for row, (key, label) in enumerate(zip(keys, labels)):
                ttk.Label(frame, text=label).grid(row=row, column=0, sticky='w', padx=4)
                ttk.Label(frame, textvariable=self.values[key]).grid(row=row, column=1, sticky='e')
        ttk.Label(state, text='Error').grid(row=3, column=0, sticky='nw', pady=(10, 0))
        ttk.Label(state, textvariable=self.values['error'], wraplength=470).grid(
            row=3, column=1, columnspan=3, sticky='w', pady=(10, 0))

        modes = ttk.LabelFrame(outer, text='Mode control', padding=10)
        modes.grid(row=2, column=0, sticky='ew')
        self.buttons = {}
        for col, (command, label) in enumerate((('start_mapping', 'Start Mapping'), ('stop_mode', 'Stop Mode'))):
            self.buttons[command] = ttk.Button(modes, text=label, command=lambda c=command: self.send(c))
            self.buttons[command].grid(row=0, column=col, padx=4)
        maps = ttk.LabelFrame(outer, text='Map control', padding=10)
        maps.grid(row=3, column=0, sticky='ew', pady=10)
        maps.columnconfigure(1, weight=1)
        ttk.Label(maps, text='Map Name').grid(row=0, column=0, padx=4)
        ttk.Entry(maps, textvariable=self.map_name).grid(row=0, column=1, sticky='ew')
        for col, (command, label) in enumerate((('save_map', 'Save Map'), ('start_navigation', 'Start Navigation')), start=2):
            self.buttons[command] = ttk.Button(maps, text=label, command=lambda c=command: self.send(c))
            self.buttons[command].grid(row=0, column=col, padx=4)
        ttk.Label(maps, text='Map name only; files are stored on the robot.').grid(
            row=1, column=0, columnspan=4, sticky='w', pady=(6, 0))
        manual = ttk.LabelFrame(outer, text='Manual mecanum drive — hold to move (MAPPING only)', padding=8)
        manual.grid(row=4, column=0, sticky='ew', pady=(0, 10))
        self.linear_speed = tk.StringVar(value='0.10')
        self.angular_speed = tk.StringVar(value='0.40')
        ttk.Label(manual, text='Linear (m/s)').grid(row=0, column=0)
        ttk.Entry(manual, textvariable=self.linear_speed, width=8).grid(row=0, column=1)
        ttk.Label(manual, text='Angular (rad/s)').grid(row=0, column=2)
        ttk.Entry(manual, textvariable=self.angular_speed, width=8).grid(row=0, column=3)
        self.manual_buttons = {}
        directions = [('Forward', (1, 0, 0), 1, 1), ('Backward', (-1, 0, 0), 3, 1),
                      ('Left', (0, 1, 0), 2, 0), ('Right', (0, -1, 0), 2, 2),
                      ('Rotate L', (0, 0, 1), 4, 0), ('Rotate R', (0, 0, -1), 4, 2)]
        for label, direction, row, col in directions:
            button = ttk.Button(manual, text=label)
            button.grid(row=row, column=col, padx=3, pady=2)
            button.bind('<ButtonPress-1>', lambda event, d=direction: self._begin_drive(d))
            self.manual_buttons[label] = button
        stop = ttk.Button(manual, text='Stop', command=lambda: self._end_drive(force=True))
        stop.grid(row=2, column=1, padx=3, pady=2)
        stop.bind('<ButtonPress-1>', lambda event: self._end_drive(force=True))
        self.manual_buttons['Stop'] = stop
        ttk.Label(outer, text='Recent commands and connection events').grid(row=5, column=0, sticky='w')
        self.log = ScrolledText(outer, height=10, state='disabled', wrap='word')
        self.log.grid(row=6, column=0, sticky='nsew', pady=(5, 0))
        nav = ttk.Frame(self.map_view)
        nav.grid(row=6, column=0, sticky='ew', pady=8)
        ttk.Label(nav, text='Navigation:').grid(row=0, column=0)
        ttk.Label(nav, textvariable=self.values['navigation']).grid(row=0, column=1)
        self.selection_buttons = {}
        for col, (kind, label) in enumerate((('initial_pose', 'Select Initial Pose'), ('goal', 'Select Goal'))):
            button = ttk.Button(nav, text=label, command=lambda k=kind: self.map_view.select_kind(k))
            button.grid(row=1, column=col, padx=4, pady=6)
            self.selection_buttons[kind] = button
        self.buttons['set_initial_pose'] = ttk.Button(nav, text='Apply Initial Pose',
                                                     command=lambda: self.send('set_initial_pose'))
        self.buttons['set_initial_pose'].grid(row=2, column=0, padx=4, pady=6)
        for col, (command, label) in enumerate((('navigation_goal', 'Send Goal'),
                                               ('cancel_navigation_goal', 'Cancel Goal'))):
            self.buttons[command] = ttk.Button(nav, text=label, command=lambda c=command: self.send(c))
            self.buttons[command].grid(row=3, column=col, padx=4, pady=6)
        self.map_view.on_goal_changed = self._update_buttons
        self._update_buttons()
        self._after = root.after(UI_INTERVAL_MS, self._poll)

    def _manual_allowed(self):
        return (self.client.is_connected and self.mode == 'MAPPING'
                and 'stop_mode' not in self.pending.values())

    def _send_manual(self, vx, vy, wz):
        try:
            self.client.send_command('manual_velocity', vx=vx, vy=vy, wz=wz)
        except (ValueError, RuntimeError, OSError) as exc:
            self._log(exc)
            return False
        return True

    def _begin_drive(self, direction):
        if not self._manual_allowed():
            return
        try:
            linear, angular = float(self.linear_speed.get()), float(self.angular_speed.get())
            if not all(math.isfinite(v) and v >= 0 for v in (linear, angular)):
                raise ValueError()
        except ValueError:
            self._log('Manual speeds must be finite, nonnegative numbers.')
            return
        self._end_drive()
        self._drive = (direction[0] * linear, direction[1] * linear, direction[2] * angular)
        self._repeat_drive()

    def _repeat_drive(self):
        self._drive_after = None
        if self._drive is None:
            return
        if not self._manual_allowed() or not self._send_manual(*self._drive):
            self._end_drive()
            return
        self._drive_after = self.root.after(MANUAL_REPEAT_MS, self._repeat_drive)

    def _end_drive(self, force=False):
        had_drive = self._drive is not None
        self._drive = None
        if self._drive_after is not None:
            self.root.after_cancel(self._drive_after)
            self._drive_after = None
        if (had_drive or force) and self.client.is_connected and self.mode == 'MAPPING':
            self._send_manual(0.0, 0.0, 0.0)

    def _focus_check(self):
        focus = self.root.focus_displayof()
        if focus is None or focus.winfo_toplevel() is not self.root:
            self._end_drive(force=True)

    def _log(self, message):
        message = str(message).replace('\r', ' ').replace('\n', ' | ')
        self.log.configure(state='normal')
        self.log.insert('end', time.strftime('[%H:%M:%S] ') + message[:3000] + '\n')
        lines = int(self.log.index('end-1c').split('.')[0]) - 1
        if lines > MAX_LOG_LINES:
            self.log.delete('1.0', f'{lines - MAX_LOG_LINES + 1}.0')
        self.log.see('end')
        self.log.configure(state='disabled')

    def _clear_pending(self):
        if self.pending:
            self._log('Connection lost; result unknown for requests: ' + ', '.join(map(str, self.pending)))
            self.pending.clear()

    def connect(self):
        self._end_drive()
        try:
            port = int(self.port.get())
            self.client.connect(self.host.get(), port)
        except (ValueError, OSError) as exc:
            self._log(exc)
            return
        self._clear_pending()
        self.mode = None
        self.map_view.clear()
        self._map_pose = None
        self._had_state = False
        self._was_connected = False
        self._log('Connecting to ' + self.host.get() + ':' + str(port))
        self._update_buttons()

    def disconnect(self):
        self._end_drive(force=True)
        self.client.flush_manual()
        self.client.disconnect()
        self._clear_pending()
        self._update_buttons()

    def send(self, command):
        if self.buttons[command].instate(['disabled']):
            return
        if command == "stop_mode":
            self._end_drive(force=True)
        fields = {}
        if command == "navigation_goal":
            fields.update(self.map_view.goal)
        elif command == "set_initial_pose":
            fields.update(self.map_view.initial_pose)
        if command in ('save_map', 'start_navigation'):
            name = self.map_name.get().strip()
            if not re.fullmatch(r'[A-Za-z0-9_.-]{1,200}', name) or name in ('.', '..'):
                self._log('Invalid map name: use A-Z, a-z, 0-9, _, -, .; no paths.')
                return
            fields['map_name'] = name
        try:
            request_id = self.client.send_command(command, **fields)
        except (ValueError, RuntimeError, OSError) as exc:
            self._log(exc)
            return
        self.pending[request_id] = command
        self._log(f'#{request_id} {command} : SENT')
        self._update_buttons()

    def _show_packet(self, packet):
        if packet['type'] == 'robot_state':
            self._had_state = True
            self.mode = packet['mode']
            self.navigation_state = packet.get('navigation_state', 'IDLE')
            self.values['navigation'].set(self.navigation_state)
            self._map_pose = packet.get('map_pose')
            self.values['mode'].set(self.mode)
            self.values['base_ready'].set('YES' if packet['base_ready'] else 'NO')
            self.values['motion'].set(packet['motion_state'])
            self.values['error'].set(packet['error'] or '—')
            for key, unit in (('x', 'm'), ('y', 'm'), ('yaw', 'rad')):
                self.values[key].set(f'{packet["pose"][key]:.3f} {unit}')
            for key, unit in (('vx', 'm/s'), ('vy', 'm/s'), ('wz', 'rad/s')):
                self.values[key].set(f'{packet["velocity"][key]:.3f} {unit}')
        elif packet['type'] == 'map':
            self.map_view.set_map(packet)
        elif packet['type'] == 'command_result':
            if packet.get('command') == 'manual_velocity' and packet['success']:
                return
            request_id = packet.get('request_id')
            command = self.pending.pop(request_id, packet.get('command', 'unknown'))
            status = 'OK' if packet['success'] else 'FAILED'
            self._log(f'#{request_id} {command} : {status} - {packet["message"]}')

    def _update_buttons(self):
        connected = self.client.is_connected
        self.connection.set('CONNECTED' if connected else 'DISCONNECTED')
        self.status_label.configure(foreground='#167344' if connected else '#a32121')
        self.freshness.set('Live robot_state' if connected else (
            'Connecting…' if self.client.connecting else
            'Stale data — controls disabled' if self._had_state else 'Waiting for robot_state'))
        self.connect_button.configure(state='disabled' if connected or self.client.connecting else 'normal')
        self.disconnect_button.configure(state='normal' if self.client.active else 'disabled')
        manual_allowed = self._manual_allowed()
        if not manual_allowed and self._drive is not None:
            self._end_drive()
        for button in self.manual_buttons.values():
            button.configure(state='normal' if manual_allowed else 'disabled')
        self.map_view.set_pose(self._map_pose if connected and self.mode != 'BASE' else None)
        allowed = {'BASE': {'start_mapping', 'start_navigation'},
                   'MAPPING': {'stop_mode', 'save_map'}, 'NAVIGATION': {'stop_mode'}}.get(self.mode, set())
        busy = set(self.pending.values())
        nav_allowed = connected and self.mode == 'NAVIGATION' and 'stop_mode' not in busy
        self.map_view.enable_goal(nav_allowed)
        for button in self.selection_buttons.values():
            button.configure(state="normal" if nav_allowed else "disabled")
        allowed = set(allowed)
        if nav_allowed and self.map_view.initial_pose is not None:
            allowed.add('set_initial_pose')
        if nav_allowed and self.map_view.goal is not None:
            allowed.add('navigation_goal')
        if nav_allowed and self.navigation_state in ('EXECUTING', 'ACCEPTED'):
            allowed.add('cancel_navigation_goal')
        for command, button in self.buttons.items():
            enabled = connected and command in allowed and command not in busy
            if busy & {'start_mapping', 'start_navigation', 'stop_mode'}:
                enabled = False
            button.configure(state='normal' if enabled else 'disabled')

    def _poll(self):
        for _ in range(128):
            try:
                kind, value = self.client.events.get_nowait()
            except queue.Empty:
                break
            if kind == 'packet':
                self._show_packet(value)
            else:
                self._log(value)
                if kind == 'disconnected':
                    self._clear_pending()
        connected = self.client.is_connected
        if self._was_connected and not connected:
            self._log('DISCONNECTED — socket closed or robot_state timed out')
        self._was_connected = connected
        self._update_buttons()
        self._after = self.root.after(UI_INTERVAL_MS, self._poll)

    def close(self):
        self._end_drive(force=True)
        self.client.flush_manual()
        self.root.after_cancel(self._after)
        self.client.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    MecanumControl(root)
    root.mainloop()


if __name__ == '__main__':
    main()
