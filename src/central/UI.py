import sys
import threading
import numpy as np
import networkx as nx
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QSpinBox, QScrollArea, QLineEdit, QFrame, QTableWidget, QTableWidgetItem
)
import cv2


class UI(QMainWindow):
    def __init__(self, coordinator, camera_input):
        self._app = QApplication.instance() or QApplication(sys.argv)
        super().__init__()

        self.coordinator = coordinator

        self.cap = cv2.VideoCapture(camera_input)
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera index {camera_input}")

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        self.homography = None
        self.inverse_homography = None
        self.robot_coords = {} # {id: (row, col)}

        self.obstacle_grid = None # ROWS x COLS binary grid, 1 = square contains >=1 tape pixel
        self.graph = None # nodes are (row, col) grid coordinates, edges to adjacent cells with weight=1


        self.task_coords = {}  # {id: {"start": (row, col), "end": (row, col), "deadline": int}}
        self.selected_task = None
        self.edit_mode = None

        self._pending_schedules = None
        self.schedules = None
    
        
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

        main_layout.addWidget(task_panel)

        self.schedule_panel  = QWidget()
        self.schedule_layout = QVBoxLayout(self.schedule_panel)
        self.schedule_panel.setFixedWidth(320)

        self.time_label  = QLabel("Current time step: 0")
        self.schedule_layout.addWidget(self.time_label)   # first thing in the column

        self.plan_widget = QWidget()                   # everything after this label
        self.plan_layout = QVBoxLayout(self.plan_widget)
        self.schedule_layout.addWidget(self.plan_widget)

        main_layout.addWidget(self.schedule_panel)
        
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(33)  # ~30 fps
        
        self.show()


    def pixel_to_grid(self, x_px, y_px):
        if self.homography is None:
            return None
        pt = cv2.perspectiveTransform(np.array([[[x_px, y_px]]], dtype=np.float32), self.homography)[0][0]
        row = min(max(int(pt[1] // self.coordinator.GRID_SIZE_CM), 0), self.coordinator.ROWS - 1)
        col = min(max(int(pt[0] // self.coordinator.GRID_SIZE_CM), 0), self.coordinator.COLS - 1)
        return (row, col)


    def grid_to_pixel(self, row, col):
        if self.inverse_homography is None:
            return None
        pt = cv2.perspectiveTransform(np.array([[[col + 0.5, row + 0.5]]], dtype=np.float32) * self.coordinator.GRID_SIZE_CM, self.inverse_homography)[0][0] # center of cell
        return (int(pt[0]), int(pt[1])) # (x, y)
    

    def update_frame(self):
        ok, frame = self.cap.read()
        if not ok:
            return
        self.time_label.setText(f"Current time step: {self.coordinator.last_completed_step}")

        if self._pending_schedules is not None:
            self.set_schedules(self._pending_schedules)
            self._pending_schedules = None


        if self.schedules:
            for robot_id, plan in self.schedules.items():
                table = self.schedule_panel.findChild(QTableWidget, f"schedule_table_{robot_id}")
                if table is None:
                    continue

                for r in range(table.rowCount()):
                    for c in range(table.columnCount()):
                        table.item(r, c).setBackground(Qt.white)

                for r, row_info in enumerate(plan):
                    if row_info["time"] >= self.coordinator.last_completed_step:
                        for c in range(table.columnCount()):
                            table.item(r, c).setBackground(Qt.yellow)
                        break

        
        robot_colors = [(0, 255, 0), (255, 165, 0), (0, 0, 255), (255, 0, 255), (0, 255, 255), (255, 0, 0)]

        if self.homography is None:
            self.compute_homography(frame.copy())
        elif self.obstacle_grid is None:
            # TODO: Update grid and graph periodically for dynamic obstacles
            self.update_obstacle_grid(frame.copy())
            self.update_graph()
        if self.homography is not None:
            frame = self.draw_robots(frame, robot_colors)
            frame = self.draw_grid_overlay(frame)
            if self.coordinator.collision_free_paths is not None:
                frame = self.draw_collision_free_paths(frame, self.coordinator.collision_free_paths, robot_colors)
                frame = self.draw_new_tasks(frame)
            else:
                #TODO: Draw waypoints for dynamically added tasks after solver runs
                frame = self.draw_unassigned_waypoints(frame)
        

        h, w, ch = frame.shape
        img = QImage(frame.data, w, h, ch * w, QImage.Format_BGR888)
        self.img_label.setPixmap(QPixmap.fromImage(img))


    def compute_homography(self, frame):
        corners, ids, _ = self.aruco_detector.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))

        if ids is None:
            print(" No markers detected.")
            return

        # Map marker ID to its corners
        marker_corners = {mid: c[0] for c, mid in zip(corners, ids.flatten())}
        required = {96, 97, 98, 99}
        if not required.issubset(marker_corners):
            detected = set(marker_corners.keys())
            missing = required - detected
            print(f" Missing markers: {missing}. Detected: {sorted(detected)}")
            return

        # Uses the inner corner of each marker
        # OpenCV ArUco corner order: 0=top-left, 1=top-right, 2=bottom-right, 3=bottom-left

        src = np.float32([
            marker_corners[96][2],  # Top Left marker, bottom-right corner 
            marker_corners[97][3],  # Top Right marker, bottom-left corner
            marker_corners[98][1],  # Bottom Left marker, top-right corner
            marker_corners[99][0],  # Bottom Right marker, top-left corner
        ])
        dst = np.float32([
            [0, 0],
            [self.coordinator.ENV_WIDTH_CM, 0],
            [0, self.coordinator.ENV_HEIGHT_CM],
            [self.coordinator.ENV_WIDTH_CM, self.coordinator.ENV_HEIGHT_CM],
        ])

        # print(f"Homography src points: {src}")
        # print(f"Homography dst points: {dst}")
        self.homography = cv2.getPerspectiveTransform(src, dst)
        self.inverse_homography = np.linalg.inv(self.homography)
        print("Corners detected, homography established.")
        

    def find_robots(self, frame):
        marker_corners, ids, _ = self.aruco_detector.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        if ids is not None:
            rows, cols = int(self.coordinator.ENV_HEIGHT_CM / self.coordinator.GRID_SIZE_CM), int(self.coordinator.ENV_WIDTH_CM / self.coordinator.GRID_SIZE_CM)
            for i in range(len(ids)):
                marker_id = ids[i][0]
                center = np.mean(marker_corners[i][0], axis=0)
                if marker_id < 96:
                    # Store as center of the cell
                    grid_pos = cv2.perspectiveTransform(np.array([[center]]), self.homography)[0][0]
                    col = int(min(max(grid_pos[0] / self.coordinator.GRID_SIZE_CM, 0), cols - 1))
                    row = int(min(max(grid_pos[1] / self.coordinator.GRID_SIZE_CM, 0), rows - 1))
                    
                    if marker_id == 3: # Temporary, accidentally skipped marker 2
                        self.robot_coords[2] = (row, col)
                    else:
                        self.robot_coords[marker_id] = (row, col)  # (row, col)
        return self.robot_coords


    def update_obstacle_grid(self, frame):
        """ROWS x COLS binary grid, 1 if square contains >=1 tape pixel."""

        lower_obstacle_range = np.array([72, 30, 110], np.uint8)   # teal tape, OpenCV HSV
        upper_obstacle_range = np.array([92, 255, 255], np.uint8)

        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_obstacle_range, upper_obstacle_range)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        warped = cv2.warpPerspective(mask, self.homography, (self.coordinator.ENV_WIDTH_CM, self.coordinator.ENV_HEIGHT_CM), flags=cv2.INTER_NEAREST)
        grid = warped.reshape(self.coordinator.ROWS, self.coordinator.GRID_SIZE_CM, self.coordinator.COLS, self.coordinator.GRID_SIZE_CM).max(1).max(2)

        self.obstacle_grid = (grid > 0).astype(np.uint8)
        print("number of unreachable cells:", np.sum(self.obstacle_grid))
        return self.obstacle_grid


    def update_graph(self):
        g = nx.Graph()
        for r in range(self.coordinator.ROWS):
            for c in range(self.coordinator.COLS):
                if self.obstacle_grid[r, c] > 0:
                    continue
                g.add_node((r, c))
                for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
                    neighbor_r, neighbor_c = r+dr, c+dc
                    if 0 <= neighbor_r < self.coordinator.ROWS and 0 <= neighbor_c < self.coordinator.COLS and self.obstacle_grid[neighbor_r, neighbor_c] == 0:
                        g.add_edge((r,c), (neighbor_r,neighbor_c), weight=1)
        self.graph = g
    
    def get_obstacle_coordinates(self):
        obstacles = []
        if self.obstacle_grid is not None:
            for row in range(self.obstacle_grid.shape[0]):
                for col in range(self.obstacle_grid.shape[1]):
                    if self.obstacle_grid[row, col]:
                        obstacles.append((row, col))
        return obstacles


    def draw_grid_overlay(self, frame):
        pink = (200, 20, 255) # grid lines
        red = (0, 0, 255) # obstacles
        overlay = frame.copy()

        if self.inverse_homography is None:
            return overlay

        for row in range(self.coordinator.ROWS + 1):
            p1 = self.grid_to_pixel(row - 0.5, -0.5)
            p2 = self.grid_to_pixel(row - 0.5, self.coordinator.COLS - 0.5)
            if p1 and p2:
                cv2.line(overlay, p1, p2, pink, 1)
        for col in range(self.coordinator.COLS + 1):
            p1 = self.grid_to_pixel(-0.5, col - 0.5)
            p2 = self.grid_to_pixel(self.coordinator.ROWS - 0.5, col - 0.5)
            if p1 and p2:
                cv2.line(overlay, p1, p2, pink, 1)

        if self.obstacle_grid is not None:
            obstacle_overlay = overlay.copy()
            for i in range(self.obstacle_grid.shape[0]):
                for j in range(self.obstacle_grid.shape[1]):
                    if self.obstacle_grid[i, j] == 1:
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
        robot_coords = self.find_robots(frame)
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
    
    def draw_new_tasks(self, frame):
        overlay = frame.copy()
        for task_id, task in self.task_coords.items():
            if task_id >= self.coordinator.num_tasks:
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
            for step in path[self.coordinator.last_completed_step+1:]:
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
                row = max(0, min(self.coordinator.ROWS - 1, x))
                col = max(0, min(self.coordinator.COLS - 1, y))
                self.task_coords[task_id][mode] = (row, col)
                self.update_task_display()
        except ValueError:
            input_widget.setText('')


    def run_solver_button_handler(self):
        if self.coordinator.execution_running:
            # If execution is running, stop it
            threading.Thread(target=self.stop_execution_thread, daemon=True).start()
        else:
            # If not running, start new execution
            threading.Thread(target=self.solve_and_execute_thread, daemon=True).start()


    def stop_execution_thread(self):
        try:
            self.solver_btn.setText('Stopping...')
            self.solver_btn.setEnabled(False)
            self.coordinator.stop_execution_event.set()
            if self.coordinator.execution_thread and self.coordinator.execution_thread.is_alive():
                self.coordinator.execution_thread.join(timeout=5.0)

            self.coordinator.execution_running = False
            self.coordinator.stop_execution_event.clear()
            print("[UI] Execution stopped")
            threading.Thread(target=self.solve_and_execute_thread, daemon=True).start()
            self.solver_btn.setEnabled(True)
        except Exception as e:
            print(f"[UI] Failed to stop execution: {e}")
            self.solver_btn.setText('Solve and Execute')
            self.solver_btn.setEnabled(True)


    def solve_and_execute_thread(self):
        try:
            # Update button text to show execution is starting
            self.solver_btn.setText('Solving...')
            self.solver_btn.setEnabled(False)
            
            success = self.coordinator.run_solver()
            if success:
                print("[UI] Solve and execute completed successfully")
                # Update button text to show execution is running
                self.solver_btn.setText('Stop and Re-solve')
                self.solver_btn.setEnabled(True)
            else:
                print("[UI] Solve and execute failed")
                self.solver_btn.setText('Solve and Execute')
                self.solver_btn.setEnabled(True)
        except Exception as e:
            print(f"[UI] Solve and execute error: {e}")
            self.solver_btn.setText('Solve and Execute')
            self.solver_btn.setEnabled(True)


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

            start_label = QLabel('Pickup:')
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

            end_label = QLabel('Dropoff:')
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


    def set_schedules(self, schedules: dict):
        self.schedules = schedules

        # clear everything that sits *below* the “Current time step” label
        while self.plan_layout.count():
            w = self.plan_layout.takeAt(0).widget()
            if w:
                w.deleteLater()

        if not self.schedules:
            return

        for robot_id, plan in self.schedules.items():
            header = QLabel(f"<b>Robot {robot_id} Plan</b>")
            self.plan_layout.addWidget(header)

            table = QTableWidget(len(plan), 4)
            table.setHorizontalHeaderLabels(["Time", "Location", "Action", "Task"])
            table.verticalHeader().setVisible(False)
            table.setEditTriggers(QTableWidget.NoEditTriggers)
            table.setSelectionMode(QTableWidget.NoSelection)

            for row, step in enumerate(plan):
                table.setItem(row, 0, QTableWidgetItem(str(step["time"])))
                table.setItem(row, 1, QTableWidgetItem(str(step["location"])))
                table.setItem(row, 2, QTableWidgetItem(step["action"]))
                task_txt = "N/A" if step["task_id"] is None else f"Task {step['task_id']}"
                table.setItem(row, 3, QTableWidgetItem(task_txt))

            table.resizeColumnsToContents()
            table.setFixedHeight(min(200, table.verticalHeader().length() + 25))
            table.setObjectName(f"schedule_table_{robot_id}")
            self.plan_layout.addWidget(table)

        self.plan_layout.addStretch()


    def eventFilter(self, source, event):
        """Handle mouse clicks on the camera view for setting waypoints."""
        if (source is self.img_label and 
            event.type() == event.MouseButtonPress and 
            event.button() == Qt.LeftButton and 
            self.edit_mode):
            
            grid_pos = self.pixel_to_grid(event.pos().x(), event.pos().y())
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
        if self.cap:
            self.cap.release()
        event.accept()