import sys
import threading
import numpy as np
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QSpinBox, QScrollArea, QLineEdit, QFrame
)
import cv2


class UI(QMainWindow):
    def __init__(self, coordinator, vision):
        self._app = QApplication.instance() or QApplication(sys.argv)
        super().__init__()

        self.coordinator = coordinator
        self.vision = vision
        
        self.task_coords = {}  # {id: {"start": (row, col), "end": (row, col), "deadline": int}}
        self.selected_task = None
        self.edit_mode = None
        
        self.setWindowTitle("Robot Environment")
        self.setGeometry(100, 100, 1200, 800)
        
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        
        self.img_label = QLabel()
        self.img_label.setMouseTracking(True)
        self.img_label.installEventFilter(self)
        main_layout.addWidget(self.img_label)
        
        task_panel = QWidget()
        task_panel.setFixedWidth(300)
        task_layout = QVBoxLayout(task_panel)
        
        add_task_btn = QPushButton('Add Task')
        add_task_btn.clicked.connect(self.add_task)
        task_layout.addWidget(add_task_btn)
        
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        self.task_layout = QVBoxLayout(scroll_content)
        scroll.setWidget(scroll_content)
        task_layout.addWidget(scroll)
        
        self.solver_btn = QPushButton('Run Solver')
        self.solver_btn.clicked.connect(self.run_solver_button_handler)
        task_layout.addWidget(self.solver_btn)
        
        self.send_instructions_btn = QPushButton('Send Instructions')
        self.send_instructions_btn.setEnabled(False)
        self.send_instructions_btn.clicked.connect(self.send_instructions_button_handler)
        task_layout.addWidget(self.send_instructions_btn)
        
        main_layout.addWidget(task_panel)
        
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(33)  # ~30 fps
        
        self.show()


    def pixel_to_grid(self, x_px, y_px):
        if self.vision.homography is None:
            return None
        pt = cv2.perspectiveTransform(np.array([[[x_px, y_px]]], dtype=np.float32), self.vision.homography)[0][0]
        row = min(max(int(pt[1] // self.vision.GRID_SIZE_CM), 0), self.vision.ROWS - 1)
        col = min(max(int(pt[0] // self.vision.GRID_SIZE_CM), 0), self.vision.COLS - 1)
        return (row, col)


    def grid_to_pixel(self, row, col):
        if self.vision.inverse_homography is None:
            return None
        pt = cv2.perspectiveTransform(np.array([[[col + 0.5, row + 0.5]]], dtype=np.float32) * self.vision.GRID_SIZE_CM, self.vision.inverse_homography)[0][0] # center of cell
        return (int(pt[0]), int(pt[1])) # (x, y)
    

    def update_frame(self):
        ok, frame = self.vision.get_frame()
        if not ok:
            return
        
        robot_colors = [(0, 255, 0), (255, 165, 0), (0, 0, 255), (255, 0, 255), (0, 255, 255), (255, 0, 0)]

        self.vision.process_frame(frame)
        if self.vision.homography is not None:
            frame = self.draw_robots(frame, robot_colors)
            frame = self.draw_grid_overlay(frame)
            if self.coordinator.collision_free_paths is not None:
                frame = self.draw_collision_free_paths(frame, self.coordinator.collision_free_paths, robot_colors)
            else:
                #TODO: Draw waypoints for dynamically added tasks after solver runs
                frame = self.draw_unassigned_waypoints(frame)

        h, w, ch = frame.shape
        img = QImage(frame.data, w, h, ch * w, QImage.Format_BGR888)
        self.img_label.setPixmap(QPixmap.fromImage(img))


    def draw_grid_overlay(self, frame):
        pink = (200, 20, 255) # grid lines
        red = (0, 0, 255) # obstacles
        overlay = frame.copy()

        if self.vision.inverse_homography is None:
            return overlay

        for row in range(self.vision.ROWS + 1):
            p1 = self.grid_to_pixel(row - 0.5, -0.5)
            p2 = self.grid_to_pixel(row - 0.5, self.vision.COLS - 0.5)
            if p1 and p2:
                cv2.line(overlay, p1, p2, pink, 1)
        for col in range(self.vision.COLS + 1):
            p1 = self.grid_to_pixel(-0.5, col - 0.5)
            p2 = self.grid_to_pixel(self.vision.ROWS - 0.5, col - 0.5)
            if p1 and p2:
                cv2.line(overlay, p1, p2, pink, 1)

        if self.vision.obstacle_grid is not None:
            obstacle_overlay = overlay.copy()
            for i in range(self.vision.obstacle_grid.shape[0]):
                for j in range(self.vision.obstacle_grid.shape[1]):
                    if self.vision.obstacle_grid[i, j] == 1:
                        corners = [
                            self.grid_to_pixel(i - 0.5, j - 0.5),
                            self.grid_to_pixel(i - 0.5, j + 0.5),
                            self.grid_to_pixel(i + 0.5, j + 0.5),
                            self.grid_to_pixel(i + 0.5, j - 0.5)
                        ]
                        if None not in corners:
                            pts = np.array([corners], dtype=np.int32)
                            cv2.fillPoly(obstacle_overlay, [pts], red)
            alpha = 0.4
            cv2.addWeighted(obstacle_overlay, alpha, overlay, 1 - alpha, 0, overlay)
        return overlay


    def draw_robots(self, frame, robot_colors):
        robot_coords = self.vision.find_robots(frame)
        overlay = frame.copy()
        for robot_id, (row, col) in robot_coords.items():
            pt = self.grid_to_pixel(row, col)
            color = robot_colors[robot_id % len(robot_colors)]
            cv2.circle(overlay, tuple(map(int, pt)), 8, color, -1)
            # Draw robot ID in top right corner of the cell it is currently occupying
            top_right = self.grid_to_pixel(row - 0.5, col + 0.5)
            if top_right:
                cv2.putText(overlay, f"R{robot_id}", (int(top_right[0] - 40), int(top_right[1] + 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        return overlay


    def draw_unassigned_waypoints(self, frame):
        overlay = frame.copy()
        for task_id, task in self.task_coords.items():
            if task['start']:
                px = self.grid_to_pixel(task['start'][0], task['start'][1])
                if px:
                    cv2.circle(overlay, px, 8, (0, 255, 0), -1)
                    cv2.putText(overlay, f"P{task_id}", (px[0]-10, px[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2) # green
            if task['end']:
                px = self.grid_to_pixel(task['end'][0], task['end'][1])
                if px:
                    cv2.circle(overlay, px, 8, (0, 0, 255), -1)
                    cv2.putText(overlay, f"D{task_id}", (px[0]-10, px[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2) # red
        return overlay
    

    def draw_collision_free_paths(self, frame, paths, robot_colors):
        overlay = frame.copy()
        thickness = 3

        robot_ids = list(paths.keys())
        num_robots = len(robot_ids)
        offset_px = 10
        # For each robot, assign an offset index: first is highest/rightmost, middle is 0, last is lowest/leftmost
        robot_offset_indices = {robot_id: (num_robots - 1) / 2 - idx for idx, robot_id in enumerate(robot_ids)}

        for robot_id, path in paths.items():
            if not path or len(path) < 2:
                continue
            color = robot_colors[robot_id % len(robot_colors)]
            pts = []
            for step in path:
                row, col = int(step[1]), int(step[2])
                pt = self.grid_to_pixel(row, col)
                if pt:
                    pts.append(pt)
            idx = robot_offset_indices[robot_id]
            
            for i in range(len(pts) - 1):
                p0, p1 = pts[i], pts[i+1]
                dx = p1[0] - p0[0]
                dy = p1[1] - p0[1]
                if abs(dx) > abs(dy):
                    offset = (0, offset_px * idx)
                elif abs(dy) > abs(dx):
                    offset = (offset_px * idx, 0)
                else:
                    offset = (0, 0)
                p0_offset = (int(p0[0] + offset[0]), int(p0[1] + offset[1]))
                p1_offset = (int(p1[0] + offset[0]), int(p1[1] + offset[1]))
                cv2.line(overlay, p0_offset, p1_offset, color, thickness)
        
        # Draw waypoints
        if self.coordinator.schedules is not None:
            for robot_id, schedule in self.coordinator.schedules.items():
                color = robot_colors[int(robot_id) % len(robot_colors)]
                for i, waypoint in enumerate(schedule):
                    if waypoint['location'] is None:
                        continue
                    row, col = int(waypoint['location'][0]), int(waypoint['location'][1])
                    center = self.grid_to_pixel(row, col)
                    if center:
                        # Waypoints remain centered
                        cv2.circle(overlay, center, 5, color, -1)
                        # Waypoint labels put in top left of the cell
                        top_left = self.grid_to_pixel(row - 0.5, col - 0.5)
                        if waypoint['action'] == 'PICKUP':
                            label = f"PICK T{waypoint['task_id']}"
                            if top_left:
                                cv2.putText(overlay, label, (int(top_left[0] + 2), int(top_left[1] + 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                        elif waypoint['action'] == 'DROPOFF':
                            label = f"DROP T{waypoint['task_id']}"
                            if top_left:
                                cv2.putText(overlay, label, (int(top_left[0] + 2), int(top_left[1] + 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        return overlay
    

# Button handlers
    def add_task(self):
        task_id = len(self.task_coords)
        self.task_coords[task_id] = {
            "id": task_id,
            "start": None,
            "end": None,
            "deadline": 99999
        }
        self.selected_task = task_id
        self.edit_mode = 'start'
        self.update_task_display()


    def remove_task(self, task_id):
        if task_id in self.task_coords:
            del self.task_coords[task_id]
            updated_tasks = {}
            for i, (_, task) in enumerate(sorted(self.task_coords.items())):
                task['id'] = i
                updated_tasks[i] = task
            self.task_coords = updated_tasks
        self.update_task_display()


    def edit_waypoint(self, task_id, mode):
        self.selected_task = task_id
        self.edit_mode = mode


    def update_deadline(self, task_id, deadline):
        if task_id in self.task_coords:
            self.task_coords[task_id]['deadline'] = deadline


    def update_waypoint_from_input(self, task_id, mode, input_widget):
        text = input_widget.text()
        try:
            x, y = map(int, text.split(','))
            if task_id in self.task_coords:
                # Clamp to grid bounds
                row = max(0, min(self.vision.ROWS - 1, x))
                col = max(0, min(self.vision.COLS - 1, y))
                self.task_coords[task_id][mode] = (row, col)
                self.update_task_display()
        except ValueError:
            input_widget.setText('')


    def run_solver_button_handler(self):
        threading.Thread(target=self.solver_thread, daemon=True).start()


    def solver_thread(self):
        try:
            self.coordinator.run_solver()
            if self.coordinator.collision_free_paths is not None:
                self.send_instructions_btn.setEnabled(True)
                print("[UI] Solver completed successfully")
            else:
                print("[UI] Solver failed")
        except Exception as e:
            print(f"[UI] Solver error: {e}")


    def send_instructions_button_handler(self):
        threading.Thread(target=self.coordinator_execute_thread, daemon=True).start()


    def coordinator_execute_thread(self):
        try:
            self.coordinator.execute_instructions()
            print("[UI] Instructions sent")
        except Exception as e:
            print(f"[UI] Failed to send instructions: {e}")


    def update_task_display(self):
        while self.task_layout.count():
            widget = self.task_layout.takeAt(0).widget()
            if widget:
                widget.deleteLater()

        for task_id, task in self.task_coords.items():
            task_widget = QWidget()
            task_widget.setFixedHeight(200)
            task_layout = QVBoxLayout(task_widget)
            task_layout.setSpacing(8)
            task_layout.setContentsMargins(8, 8, 8, 8)

            header_widget = QWidget()
            header_widget.setFixedHeight(40)
            header_layout = QHBoxLayout(header_widget)
            header_layout.setContentsMargins(0, 0, 0, 0)

            task_label = QLabel(f'Task {task_id}')
            remove_btn = QPushButton('Remove')
            remove_btn.setFixedSize(80, 30)
            remove_btn.clicked.connect(lambda checked, x=task_id: self.remove_task(x))
            header_layout.addWidget(task_label)
            header_layout.addWidget(remove_btn)
            task_layout.addWidget(header_widget)

            start_widget = QWidget()
            start_widget.setFixedHeight(40)
            start_layout = QHBoxLayout(start_widget)
            start_layout.setContentsMargins(0, 0, 0, 0)

            start_label = QLabel('Start Point:')
            start_btn = QPushButton(str(task['start']) if task['start'] else 'Set Pickup')
            start_btn.setFixedSize(120, 30)
            start_btn.clicked.connect(lambda checked, x=task_id: self.edit_waypoint(x, 'start'))
            
            start_input = QLineEdit()
            start_input.setFixedSize(120, 30)
            start_input.setPlaceholderText('x, y')
            if task['start']:
                start_input.setText(f"{task['start'][0]}, {task['start'][1]}")
            start_input.editingFinished.connect(
                lambda x=task_id, input=start_input: self.update_waypoint_from_input(x, 'start', input)
            )

            start_layout.addWidget(start_label)
            start_layout.addWidget(start_btn)
            start_layout.addWidget(start_input)
            task_layout.addWidget(start_widget)

            end_widget = QWidget()
            end_widget.setFixedHeight(40)
            end_layout = QHBoxLayout(end_widget)
            end_layout.setContentsMargins(0, 0, 0, 0)

            end_label = QLabel('End Point:')
            end_btn = QPushButton(str(task['end']) if task['end'] else 'Set Dropoff')
            end_btn.setFixedSize(120, 30)
            end_btn.clicked.connect(lambda checked, x=task_id: self.edit_waypoint(x, 'end'))
            
            end_input = QLineEdit()
            end_input.setFixedSize(120, 30)
            end_input.setPlaceholderText('x, y')
            if task['end']:
                end_input.setText(f"{task['end'][0]}, {task['end'][1]}")
            end_input.editingFinished.connect(
                lambda x=task_id, input=end_input: self.update_waypoint_from_input(x, 'end', input)
            )

            end_layout.addWidget(end_label)
            end_layout.addWidget(end_btn)
            end_layout.addWidget(end_input)
            task_layout.addWidget(end_widget)

            deadline_widget = QWidget()
            deadline_widget.setFixedHeight(40)
            deadline_layout = QHBoxLayout(deadline_widget)
            deadline_layout.setContentsMargins(0, 0, 0, 0)

            deadline_label = QLabel('Deadline (ms):')
            deadline_spin = QSpinBox()
            deadline_spin.setFixedSize(120, 30)
            deadline_spin.setRange(0, 1000000)
            deadline_spin.setValue(task['deadline'])
            deadline_spin.valueChanged.connect(lambda value, x=task_id: self.update_deadline(x, value))

            deadline_layout.addWidget(deadline_label)
            deadline_layout.addWidget(deadline_spin)
            task_layout.addWidget(deadline_widget)

            if task_id < len(self.task_coords) - 1:
                line = QFrame()
                line.setFrameShape(QFrame.HLine)
                line.setFrameShadow(QFrame.Sunken)
                task_layout.addWidget(line)
            
            self.task_layout.addWidget(task_widget)
        
        self.task_layout.addStretch()


    def eventFilter(self, source, event):
        """Handle mouse clicks on the camera view for setting waypoints."""
        if (source is self.img_label and 
            event.type() == event.MouseButtonPress and 
            event.button() == Qt.LeftButton and 
            self.edit_mode):
            
            grid_pos = self.vision.pixel_to_grid(event.pos().x(), event.pos().y())
            if grid_pos is not None and self.selected_task is not None:
                row, col = grid_pos
                if self.edit_mode == 'start':
                    self.task_coords[self.selected_task]['start'] = (row, col)
                    self.edit_mode = 'end'
                elif self.edit_mode == 'end':
                    self.task_coords[self.selected_task]['end'] = (row, col)
                    self.edit_mode = None
                    self.selected_task = None
                self.update_task_display()
        return super().eventFilter(source, event)
    

    def closeEvent(self, event):
        self.vision.close()
        event.accept()