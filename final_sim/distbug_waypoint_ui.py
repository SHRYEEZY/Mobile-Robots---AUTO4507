#!/usr/bin/env python3
"""
distbug_waypoint_ui_live.py

Enhanced Tkinter UI for Part 3 DistBug.

Features:
  - Go To Waypoints button
  - Live /distbug_mode status display
  - Live camera preview from /camera/image
  - Scrollable landmark table loaded from ~/ros2_ws/landmarks.csv
  - Start/stop MP4 video recording of the camera feed
  - Live map canvas from /distbug_live_map

Publishes:
  /distbug_command   std_msgs/String

Subscribes:
  /distbug_mode      std_msgs/String
  /camera/image      sensor_msgs/Image, configurable by ROS parameter camera_topic
  /distbug_live_map  std_msgs/String JSON from DistBugController
"""

import csv
import os
import json
import math
from datetime import datetime
from typing import Optional

import cv2
import tkinter as tk
from tkinter import ttk

try:
    from PIL import Image as PILImage
    from PIL import ImageTk
    PIL_AVAILABLE = True
except Exception:
    PIL_AVAILABLE = False

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class DistBugUiNode(Node):
    def __init__(self):
        super().__init__('distbug_waypoint_ui')

        self.declare_parameter('camera_topic', '/camera/image')
        self.declare_parameter('landmarks_csv', os.path.expanduser('~/ros2_ws/landmarks.csv'))
        self.declare_parameter('record_dir', os.path.expanduser('~/ros2_ws/videos'))

        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.landmarks_csv = self.get_parameter('landmarks_csv').get_parameter_value().string_value
        self.record_dir = self.get_parameter('record_dir').get_parameter_value().string_value

        self.command_pub = self.create_publisher(String, '/distbug_command', 10)
        self.mode_sub = self.create_subscription(String, '/distbug_mode', self.mode_callback, 10)
        self.image_sub = self.create_subscription(Image, self.camera_topic, self.image_callback, 10)
        self.live_map_sub = self.create_subscription(
            String, '/distbug_live_map', self.live_map_callback, 10
        )

        self.bridge = CvBridge()
        self.latest_mode = 'Waiting for /distbug_mode...'
        self.latest_frame_bgr: Optional[object] = None
        self.latest_frame_stamp = None
        self.frame_count = 0
        self.latest_live_map = None

    def mode_callback(self, msg: String):
        self.latest_mode = msg.data

    def live_map_callback(self, msg: String):
        try:
            self.latest_live_map = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f'Failed to decode live map JSON: {e}')

    def image_callback(self, msg: Image):
        try:
            self.latest_frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_frame_stamp = msg.header.stamp
            self.frame_count += 1
        except Exception as e:
            self.get_logger().warn(f'Failed to convert camera image: {e}')

    def send_command(self, command: str):
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        self.get_logger().info(f'Sent UI command: {command}')


class DistBugUiApp:
    def __init__(self, root: tk.Tk, node: DistBugUiNode):
        self.root = root
        self.node = node

        self.photo_image = None
        self.video_writer = None
        self.video_path = None
        self.recording = False
        self.last_recorded_frame_count = -1
        self.last_landmark_mtime = None
        self.landmark_rows = []
        self.map_scale_padding = 1.0

        self.root.title('DistBug Part 3 Mission Control')
        self.root.geometry('1100x720')
        self.root.minsize(980, 620)

        self.build_layout()
        self.root.after(50, self.spin_ros)
        self.root.after(100, self.update_camera_view)
        self.root.after(100, self.update_live_map_view)
        self.root.after(500, self.update_landmark_table)

    def build_layout(self):
        main = ttk.Frame(self.root, padding=12)
        main.pack(fill='both', expand=True)

        top = ttk.Frame(main)
        top.pack(fill='x', pady=(0, 10))

        title = ttk.Label(top, text='DistBug Part 3 Mission Control', font=('Arial', 18, 'bold'))
        title.pack(side='left')

        button_row = ttk.Frame(top)
        button_row.pack(side='right')

        ttk.Button(
            button_row,
            text='Go To Waypoints',
            command=lambda: self.node.send_command('START_WAYPOINTS'),
        ).pack(side='left', padx=(0, 8))

        self.record_button = ttk.Button(
            button_row,
            text='Start Recording',
            command=self.toggle_recording,
        )
        self.record_button.pack(side='left')

        status_frame = ttk.LabelFrame(main, text='Robot state')
        status_frame.pack(fill='x', pady=(0, 10))

        self.status_var = tk.StringVar(value='Waiting for status...')
        ttk.Label(
            status_frame,
            textvariable=self.status_var,
            font=('Arial', 11),
            wraplength=1000,
            padding=8,
        ).pack(fill='x')

        self.record_status_var = tk.StringVar(value='Recording: OFF')
        ttk.Label(status_frame, textvariable=self.record_status_var, padding=(8, 0, 8, 8)).pack(anchor='w')

        content = ttk.Frame(main)
        content.pack(fill='both', expand=True)

        camera_frame = ttk.LabelFrame(content, text=f'Live camera: {self.node.camera_topic}')
        camera_frame.pack(side='left', fill='both', expand=True, padx=(0, 10))

        self.camera_label = ttk.Label(camera_frame, text='Waiting for camera frame...', anchor='center')
        self.camera_label.pack(fill='both', expand=True, padx=8, pady=8)

        right = ttk.Frame(content)
        right.pack(side='left', fill='both', expand=True)

        map_frame = ttk.LabelFrame(right, text='Live map')
        map_frame.pack(fill='both', expand=True, pady=(0, 10))

        self.map_canvas = tk.Canvas(map_frame, bg='white', height=320)
        self.map_canvas.pack(fill='both', expand=True, padx=8, pady=8)

        self.map_info_var = tk.StringVar(value='Waiting for /distbug_live_map...')
        ttk.Label(map_frame, textvariable=self.map_info_var, padding=(8, 0, 8, 8)).pack(anchor='w')

        landmark_frame = ttk.LabelFrame(right, text=f'Landmarks: {self.node.landmarks_csv}')
        landmark_frame.pack(fill='both', expand=True)

        columns = ('label', 'type', 'colour', 'x', 'y', 'distance', 'timestamp')
        self.tree = ttk.Treeview(landmark_frame, columns=columns, show='headings', height=18)
        for col in columns:
            self.tree.heading(col, text=col)

        self.tree.column('label', width=95, anchor='w')
        self.tree.column('type', width=90, anchor='center')
        self.tree.column('colour', width=70, anchor='center')
        self.tree.column('x', width=70, anchor='e')
        self.tree.column('y', width=70, anchor='e')
        self.tree.column('distance', width=90, anchor='e')
        self.tree.column('timestamp', width=155, anchor='w')

        yscroll = ttk.Scrollbar(landmark_frame, orient='vertical', command=self.tree.yview)
        self.tree.configure(yscrollcommand=yscroll.set)
        self.tree.pack(side='left', fill='both', expand=True, padx=(8, 0), pady=8)
        yscroll.pack(side='left', fill='y', pady=8)

        self.landmark_summary_var = tk.StringVar(value='Waypoints: 0 | Objects: 0 | Unknown: 0')
        ttk.Label(right, textvariable=self.landmark_summary_var, padding=(4, 8)).pack(anchor='w')

    def spin_ros(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.status_var.set(self.node.latest_mode)
        self.root.after(30, self.spin_ros)

    def update_camera_view(self):
        frame = self.node.latest_frame_bgr
        if frame is not None:
            display = self.resize_for_display(frame, max_w=620, max_h=430)
            rgb = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)

            if PIL_AVAILABLE:
                img = PILImage.fromarray(rgb)
                self.photo_image = ImageTk.PhotoImage(image=img)
                self.camera_label.configure(image=self.photo_image, text='')
            else:
                # Fallback: no PIL installed. Keep UI running but do not display image.
                self.camera_label.configure(text='Camera frames received. Install python3-pil.imagetk for live preview.')

            if self.recording and self.video_writer is not None:
                if self.node.frame_count != self.last_recorded_frame_count:
                    self.video_writer.write(frame)
                    self.last_recorded_frame_count = self.node.frame_count

        self.root.after(100, self.update_camera_view)

    def resize_for_display(self, frame, max_w=620, max_h=430):
        h, w = frame.shape[:2]
        scale = min(max_w / max(1, w), max_h / max(1, h), 1.0)
        new_w = int(w * scale)
        new_h = int(h * scale)
        return cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA)

    def toggle_recording(self):
        if self.recording:
            self.stop_recording()
        else:
            self.start_recording()

    def start_recording(self):
        frame = self.node.latest_frame_bgr
        if frame is None:
            self.record_status_var.set('Recording: cannot start yet, no camera frame')
            return

        os.makedirs(self.node.record_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.video_path = os.path.join(self.node.record_dir, f'distbug_journey_{timestamp}.mp4')

        h, w = frame.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        fps = 10.0
        self.video_writer = cv2.VideoWriter(self.video_path, fourcc, fps, (w, h))

        if not self.video_writer.isOpened():
            self.video_writer = None
            self.video_path = None
            self.record_status_var.set('Recording: failed to open video writer')
            return

        self.recording = True
        self.last_recorded_frame_count = -1
        self.record_button.configure(text='Stop Recording')
        self.record_status_var.set(f'Recording: ON -> {self.video_path}')
        self.node.get_logger().info(f'Started video recording: {self.video_path}')

    def stop_recording(self):
        if self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None

        path = self.video_path
        self.recording = False
        self.video_path = None
        self.record_button.configure(text='Start Recording')
        self.record_status_var.set(f'Recording: OFF. Saved: {path}')
        self.node.get_logger().info(f'Stopped video recording: {path}')

    def update_landmark_table(self):
        path = self.node.landmarks_csv
        try:
            if os.path.exists(path):
                mtime = os.path.getmtime(path)
                if self.last_landmark_mtime != mtime:
                    self.last_landmark_mtime = mtime
                    self.reload_landmarks(path)
        except Exception as e:
            self.landmark_summary_var.set(f'Landmark load error: {e}')

        self.root.after(1000, self.update_landmark_table)

    def reload_landmarks(self, path: str):
        for item in self.tree.get_children():
            self.tree.delete(item)

        self.landmark_rows = []

        wp = obj = unk = 0
        rows = []

        with open(path, 'r', newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                label = row.get('label', '')
                ltype = row.get('type', '')
                colour = row.get('colour', '')
                x = row.get('x', '')
                y = row.get('y', '')
                dist = row.get('relative_distance_m', '')
                timestamp = row.get('timestamp', '')
                rows.append((label, ltype, colour, x, y, dist, timestamp))

                try:
                    self.landmark_rows.append({
                        'label': label,
                        'type': ltype,
                        'colour': colour,
                        'x': float(x),
                        'y': float(y),
                        'distance': dist,
                        'timestamp': timestamp,
                    })
                except Exception:
                    pass

                t = ltype.strip().upper()
                if t == 'WAYPOINT':
                    wp += 1
                elif t == 'OBJECT':
                    obj += 1
                else:
                    unk += 1

        for row in rows:
            self.tree.insert('', 'end', values=row)

        self.landmark_summary_var.set(f'Waypoints: {wp} | Objects: {obj} | Unknown: {unk}')

    def update_live_map_view(self):
        data = self.node.latest_live_map

        if data is None:
            self.map_info_var.set('Waiting for /distbug_live_map...')
            self.root.after(100, self.update_live_map_view)
            return

        try:
            self.draw_live_map(data)
        except Exception as e:
            self.map_info_var.set(f'Live map draw error: {e}')

        self.root.after(100, self.update_live_map_view)

    def draw_live_map(self, data):
        c = self.map_canvas
        c.delete('all')

        width = max(1, c.winfo_width())
        height = max(1, c.winfo_height())

        points = []

        def add_xy(x, y):
            if x is None or y is None:
                return
            try:
                points.append((float(x), float(y)))
            except Exception:
                pass

        for pnt in data.get('path', []):
            add_xy(pnt.get('x'), pnt.get('y'))
        for pnt in data.get('hits', []):
            add_xy(pnt.get('x'), pnt.get('y'))
        for pnt in data.get('leaves', []):
            add_xy(pnt.get('x'), pnt.get('y'))
        for pnt in data.get('waypoints', []):
            add_xy(pnt.get('x'), pnt.get('y'))
        for pnt in data.get('completed', []):
            add_xy(pnt.get('x'), pnt.get('y'))
        for lm in self.landmark_rows:
            add_xy(lm.get('x'), lm.get('y'))

        pose = data.get('pose')
        if pose:
            add_xy(pose.get('x'), pose.get('y'))

        home = data.get('home')
        if home:
            add_xy(home.get('x'), home.get('y'))

        origin = data.get('origin')
        if origin:
            add_xy(origin.get('x'), origin.get('y'))

        goal = data.get('current_goal')
        if goal:
            add_xy(goal.get('x'), goal.get('y'))

        if not points:
            c.create_text(width / 2, height / 2, text='No live map points yet', fill='black')
            return

        xs = [pnt[0] for pnt in points]
        ys = [pnt[1] for pnt in points]

        padding_m = self.map_scale_padding
        min_x = min(xs) - padding_m
        max_x = max(xs) + padding_m
        min_y = min(ys) - padding_m
        max_y = max(ys) + padding_m

        if abs(max_x - min_x) < 0.5:
            min_x -= 0.5
            max_x += 0.5
        if abs(max_y - min_y) < 0.5:
            min_y -= 0.5
            max_y += 0.5

        margin = 28
        usable_w = max(1, width - 2 * margin)
        usable_h = max(1, height - 2 * margin)

        scale_x = usable_w / (max_x - min_x)
        scale_y = usable_h / (max_y - min_y)
        scale = min(scale_x, scale_y)

        def sx(x):
            return margin + (x - min_x) * scale

        def sy(y):
            return height - margin - (y - min_y) * scale

        def draw_circle(x, y, r, fill, outline='black', label=None):
            px = sx(x)
            py = sy(y)
            c.create_oval(px - r, py - r, px + r, py + r, fill=fill, outline=outline, width=1)
            if label:
                c.create_text(px + r + 4, py, text=label, anchor='w', fill='black', font=('Arial', 8))

        if min_x <= 0.0 <= max_x:
            c.create_line(sx(0.0), margin, sx(0.0), height - margin, fill='#dddddd')
        if min_y <= 0.0 <= max_y:
            c.create_line(margin, sy(0.0), width - margin, sy(0.0), fill='#dddddd')

        c.create_rectangle(margin, margin, width - margin, height - margin, outline='#cccccc')

        path_points = data.get('path', [])
        if len(path_points) >= 2:
            coords = []
            for pnt in path_points:
                coords.extend([sx(float(pnt['x'])), sy(float(pnt['y']))])
            c.create_line(*coords, fill='blue', width=2)

        ray = data.get('ray', {})
        if ray.get('active') and ray.get('target_x') is not None and pose:
            c.create_line(
                sx(float(pose['x'])),
                sy(float(pose['y'])),
                sx(float(ray['target_x'])),
                sy(float(ray['target_y'])),
                fill='#999999',
                dash=(4, 3),
                width=1,
            )

        for lm in self.landmark_rows:
            x = lm['x']
            y = lm['y']
            label = lm.get('label', '')
            ltype = lm.get('type', '').strip().upper()

            if ltype == 'WAYPOINT':
                draw_circle(x, y, 5, fill='cyan', label=label)
            elif ltype == 'OBJECT':
                draw_circle(x, y, 5, fill='orange', label=label)
            else:
                draw_circle(x, y, 5, fill='gray', label=label)

        for wp in data.get('waypoints', []):
            x = float(wp['x'])
            y = float(wp['y'])
            idx = wp.get('index', '')
            px = sx(x)
            py = sy(y)
            c.create_rectangle(px - 6, py - 6, px + 6, py + 6, outline='purple', width=2)
            c.create_text(px + 8, py - 8, text=f'WP{idx}', anchor='w', fill='purple', font=('Arial', 8, 'bold'))

        for i, hp in enumerate(data.get('hits', []), start=1):
            x = float(hp['x'])
            y = float(hp['y'])
            px = sx(x)
            py = sy(y)
            c.create_line(px - 5, py - 5, px + 5, py + 5, fill='red', width=2)
            c.create_line(px - 5, py + 5, px + 5, py - 5, fill='red', width=2)
            c.create_text(px + 7, py, text=f'H{i}', anchor='w', fill='red', font=('Arial', 8))

        for i, lp in enumerate(data.get('leaves', []), start=1):
            x = float(lp['x'])
            y = float(lp['y'])
            px = sx(x)
            py = sy(y)
            c.create_polygon(px, py - 6, px - 6, py + 6, px + 6, py + 6, fill='green', outline='black')
            c.create_text(px + 7, py, text=f'L{i}', anchor='w', fill='green', font=('Arial', 8))

        for cp in data.get('completed', []):
            x = float(cp['x'])
            y = float(cp['y'])
            idx = cp.get('index', '')
            draw_circle(x, y, 6, fill='lime', label=f'DONE {idx}')

        origin = data.get('origin')
        if origin:
            self.draw_star(float(origin['x']), float(origin['y']), sx, sy, c, label='Origin')

        home = data.get('home')
        if home:
            draw_circle(float(home['x']), float(home['y']), 5, fill='white', outline='black', label='Scan Home')

        goal = data.get('current_goal')
        if goal:
            gx = float(goal['x'])
            gy = float(goal['y'])
            px = sx(gx)
            py = sy(gy)
            c.create_oval(px - 10, py - 10, px + 10, py + 10, outline='magenta', width=2)
            c.create_text(px + 12, py, text=goal.get('label', 'GOAL'), anchor='w', fill='magenta', font=('Arial', 8, 'bold'))

        if pose:
            x = float(pose['x'])
            y = float(pose['y'])
            yaw = float(pose['yaw'])

            px = sx(x)
            py = sy(y)

            c.create_oval(px - 7, py - 7, px + 7, py + 7, fill='red', outline='black')

            arrow_len = 22
            ax = px + arrow_len * math.cos(yaw)
            ay = py - arrow_len * math.sin(yaw)
            c.create_line(px, py, ax, ay, fill='red', width=3, arrow=tk.LAST)
            c.create_text(px + 10, py + 12, text='Robot', anchor='w', fill='red', font=('Arial', 8, 'bold'))

        status = data.get('status', '')
        pose_text = ''
        if pose:
            pose_text = f"x={pose['x']:.2f}, y={pose['y']:.2f}, yaw={math.degrees(pose['yaw']):.1f} deg"

        self.map_info_var.set(f'{pose_text} | {status}')

    def draw_star(self, x, y, sx, sy, canvas, label='Home'):
        px = sx(x)
        py = sy(y)
        r_outer = 9
        r_inner = 4
        points = []

        for i in range(10):
            angle = -math.pi / 2 + i * math.pi / 5
            r = r_outer if i % 2 == 0 else r_inner
            points.extend([
                px + r * math.cos(angle),
                py + r * math.sin(angle),
            ])

        canvas.create_polygon(points, fill='yellow', outline='black')
        canvas.create_text(px + 12, py, text=label, anchor='w', fill='black', font=('Arial', 8, 'bold'))

    def on_close(self):
        if self.recording:
            self.stop_recording()
        self.root.destroy()


def main(args=None):
    rclpy.init(args=args)
    node = DistBugUiNode()

    root = tk.Tk()
    app = DistBugUiApp(root, node)
    root.protocol('WM_DELETE_WINDOW', app.on_close)

    try:
        root.mainloop()
    finally:
        if app.recording:
            app.stop_recording()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
