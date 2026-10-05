"""OccupancyGrid decoding and Tk map view; standard library only."""

import base64
import math
import tkinter as tk
from tkinter import ttk
import zlib

MAX_MAP_CELLS = 1024 * 1024


def decode_map(packet):
    if packet.get('encoding') != 'zlib+base64:int8':
        raise ValueError('unsupported map encoding')
    width, height = packet.get('width'), packet.get('height')
    if type(width) is not int or type(height) is not int or width <= 0 or height <= 0:
        raise ValueError('invalid map dimensions')
    if width * height > MAX_MAP_CELLS:
        raise ValueError('map exceeds cell limit')
    if type(packet.get('sequence')) is not int or packet['sequence'] < 0:
        raise ValueError('invalid map sequence')
    resolution = packet.get('resolution')
    origin = packet.get('origin')
    if not isinstance(origin, dict):
        raise ValueError('invalid map origin')
    values = [resolution] + [origin.get(key) for key in ('x', 'y', 'qz', 'qw')]
    if any(type(value) not in (float, int) or not math.isfinite(value) for value in values):
        raise ValueError('invalid map metadata')
    norm = math.hypot(origin['qz'], origin['qw'])
    if resolution <= 0 or not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('invalid map resolution/orientation')
    compressed = base64.b64decode(packet['data'], validate=True)
    decoder = zlib.decompressobj()
    cells = decoder.decompress(compressed, width * height + 1)
    if len(cells) != width * height or not decoder.eof or decoder.unused_data:
        raise ValueError('map data length/compression mismatch')
    if any(value > 100 and value != 255 for value in cells):
        raise ValueError('invalid occupancy value')
    result = dict(packet)
    result.pop('data')
    result['cells'] = cells
    return result


class MapView(ttk.LabelFrame):
    def __init__(self, parent):
        super().__init__(parent, text='Live map', padding=8)
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)
        self.canvas = tk.Canvas(self, width=480, height=480, background='#dadada', highlightthickness=0)
        self.canvas.grid(row=0, column=0, sticky='nsew')
        self.info = tk.StringVar(value='Waiting for /map')
        self.pose_info = tk.StringVar(value='Robot overlay unavailable: waiting for map_pose / TF')
        ttk.Label(self, textvariable=self.info, wraplength=450).grid(row=1, column=0, sticky='w')
        ttk.Label(self, textvariable=self.pose_info, wraplength=450).grid(row=2, column=0, sticky='w')
        self.goal = None
        self.initial_pose = None
        self.selection_kind = "goal"
        self._goal_drag = None
        self.goal_enabled = False
        self.on_goal_changed = lambda: None
        self.map = None
        self.pose = None
        self.image = None
        self.scale_x = self.scale_y = 1.0
        self.offset_x = self.offset_y = 0.0
        self._resize_job = None
        self.canvas.bind('<Configure>', self._resize)
        self.canvas.bind('<ButtonPress-1>', self._goal_press)
        self.canvas.bind('<B1-Motion>', self._goal_motion)
        self.canvas.bind('<ButtonRelease-1>', self._goal_release)
        self.goal_info = tk.StringVar(value='Goal: not selected')
        ttk.Label(self, textvariable=self.goal_info).grid(row=3, column=0, sticky='w')
        self.initial_pose_info = tk.StringVar(value='Initial Pose: not selected')
        ttk.Label(self, textvariable=self.initial_pose_info).grid(row=4, column=0, sticky='w')
        self.selection_info = tk.StringVar(value='Selecting: Goal (orange)')
        ttk.Label(self, textvariable=self.selection_info).grid(row=5, column=0, sticky='w')

    def _resize(self, _event):
        if self._resize_job is not None:
            self.after_cancel(self._resize_job)
        self._resize_job = self.after(100, self.render)

    def set_map(self, packet):
        self.map = packet
        self.render()

    def clear(self):
        self.initial_pose = None
        self.initial_pose_info.set("Initial Pose: not selected")
        self.select_kind("goal")
        self.goal = None
        self._goal_drag = None
        self.goal_info.set("Goal: not selected")
        self.map = None
        self.pose = None
        self.image = None
        self.canvas.delete('all')
        self.info.set('Waiting for /map')
        self.set_pose(None)

    def render(self):
        if self._resize_job is not None:
            self.after_cancel(self._resize_job)
            self._resize_job = None
        if self.map is None:
            return
        m = self.map
        width, height = m['width'], m['height']
        canvas_w, canvas_h = max(1, self.canvas.winfo_width()), max(1, self.canvas.winfo_height())
        scale = min(canvas_w / width, canvas_h / height)
        # Bound GUI work even on a very large desktop. Metadata always retains full resolution.
        scale = min(scale, 800 / width, 800 / height)
        out_w, out_h = max(1, int(width * scale)), max(1, int(height * scale))
        self.scale_x, self.scale_y = out_w / width, out_h / height
        self.offset_x, self.offset_y = (canvas_w - out_w) / 2, (canvas_h - out_h) / 2
        palette = [bytes([round(255 * (1 - value / 100))]) * 3 for value in range(101)]
        palette.extend([b'\x80\x80\x80'] * (256 - len(palette)))
        pixels = bytearray()
        xs = [min(width - 1, int(x / self.scale_x)) for x in range(out_w)]
        for y in range(out_h):
            source_y = height - 1 - min(height - 1, int(y / self.scale_y))
            row = source_y * width
            pixels.extend(b''.join(palette[m['cells'][row + x]] for x in xs))
        self.image = tk.PhotoImage(data=f'P6\n{out_w} {out_h}\n255\n'.encode() + pixels, format='PPM')
        self.canvas.delete('map')
        self.canvas.create_image(self.offset_x, self.offset_y, anchor='nw', image=self.image, tags='map')
        self.canvas.tag_lower('map')
        self.info.set(f"Map #{m['sequence']} · {width} × {height} · {m['resolution']:.3f} m/cell")
        self.set_pose(self.pose)
        self._draw_goal()

    def world_to_canvas(self, x, y):
        m = self.map
        origin = m['origin']
        norm = math.hypot(origin['qz'], origin['qw'])
        qz, qw = origin['qz'] / norm, origin['qw'] / norm
        yaw = math.atan2(2 * qw * qz, 1 - 2 * qz * qz)
        dx, dy = x - origin['x'], y - origin['y']
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        return (self.offset_x + local_x / m['resolution'] * self.scale_x,
                self.offset_y + (m['height'] - local_y / m['resolution']) * self.scale_y)

    def set_pose(self, pose):
        self.pose = pose
        self.canvas.delete('robot')
        if not self.map or not isinstance(pose, dict) or any(
                type(pose.get(key)) not in (int, float) or not math.isfinite(pose[key])
                for key in ('x', 'y', 'yaw')):
            self.pose_info.set('Robot overlay unavailable: waiting for map_pose / TF')
            return
        x, y = self.world_to_canvas(pose['x'], pose['y'])
        hx, hy = self.world_to_canvas(pose['x'] + math.cos(pose['yaw']), pose['y'] + math.sin(pose['yaw']))
        norm = math.hypot(hx - x, hy - y)
        if norm:
            hx, hy = x + 18 * (hx - x) / norm, y + 18 * (hy - y) / norm
        self.canvas.create_oval(x-4, y-4, x+4, y+4, fill='#1673dd', outline='white', tags='robot')
        self.canvas.create_line(x, y, hx, hy, fill='#1673dd', width=2, arrow='last', tags='robot')
        self.pose_info.set('Robot pose in map frame (TF transformed)')

    def canvas_to_world(self, x, y):
        m = self.map
        origin = m['origin']
        norm = math.hypot(origin['qz'], origin['qw'])
        qz, qw = origin['qz'] / norm, origin['qw'] / norm
        yaw = math.atan2(2 * qw * qz, 1 - 2 * qz * qz)
        local_x = (x - self.offset_x) / self.scale_x * m['resolution']
        local_y = (m['height'] - (y - self.offset_y) / self.scale_y) * m['resolution']
        return (origin['x'] + math.cos(yaw) * local_x - math.sin(yaw) * local_y,
                origin['y'] + math.sin(yaw) * local_x + math.cos(yaw) * local_y)

    def enable_goal(self, enabled):
        self.goal_enabled = enabled
        if not enabled and self._goal_drag is not None:
            self._goal_drag = None
            self._draw_goal()

    def select_kind(self, kind):
        if kind not in ('goal', 'initial_pose'):
            raise ValueError('unknown selection kind')
        self._goal_drag = None
        self.selection_kind = kind
        self.selection_info.set('Selecting: Goal (orange)' if kind == 'goal'
                                else 'Selecting: Initial Pose (purple)')
        self._draw_goal()

    def _goal_press(self, event):
        if not self.goal_enabled or self.map is None:
            return
        if not (self.offset_x <= event.x <= self.offset_x + self.map['width'] * self.scale_x
                and self.offset_y <= event.y <= self.offset_y + self.map['height'] * self.scale_y):
            return
        x, y = self.canvas_to_world(event.x, event.y)
        previous = self.goal if self.selection_kind == 'goal' else self.initial_pose
        yaw = self.pose['yaw'] if self.pose is not None else previous['yaw'] if previous else 0.0
        self._goal_drag = dict(x=x, y=y, yaw=yaw)
        self._draw_goal()

    def _goal_motion(self, event):
        if not self.goal_enabled or self._goal_drag is None:
            return
        goal = self._goal_drag
        px, py = self.world_to_canvas(goal['x'], goal['y'])
        if math.hypot(event.x - px, event.y - py) >= 5.0:
            x, y = self.canvas_to_world(event.x, event.y)
            goal['yaw'] = math.atan2(y - goal['y'], x - goal['x'])
        self._draw_goal()

    def _goal_release(self, event):
        if not self.goal_enabled or self._goal_drag is None:
            return
        self._goal_motion(event)
        if self.selection_kind == "goal":
            self.goal = self._goal_drag
        else:
            self.initial_pose = self._goal_drag
        self._goal_drag = None
        self._draw_goal()
        self.on_goal_changed()

    def _draw_goal(self):
        for kind, candidate, color, label, info in (
                ('goal', self.goal, '#d05000', 'Goal', self.goal_info),
                ('initial_pose', self.initial_pose, '#8b35c9', 'Initial Pose', self.initial_pose_info)):
            self.canvas.delete(kind)
            pose = self._goal_drag if self.selection_kind == kind and self._goal_drag else candidate
            if pose is None or self.map is None:
                info.set(f'{label}: not selected')
                continue
            x, y = self.world_to_canvas(pose['x'], pose['y'])
            hx, hy = self.world_to_canvas(pose['x'] + math.cos(pose['yaw']), pose['y'] + math.sin(pose['yaw']))
            length = math.hypot(hx - x, hy - y)
            if length:
                hx, hy = x + 25 * (hx - x) / length, y + 25 * (hy - y) / length
            marker = self.canvas.create_rectangle if kind == 'initial_pose' else self.canvas.create_oval
            marker(x-6, y-6, x+6, y+6, outline=color, width=2, tags=kind)
            self.canvas.create_line(x, y, hx, hy, fill=color, width=3, arrow='last', tags=kind)
            info.set(f"{label} X {pose['x']:.3f} m   Y {pose['y']:.3f} m   Yaw {pose['yaw']:.3f} rad")
