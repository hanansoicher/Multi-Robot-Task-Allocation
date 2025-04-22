import sys
import threading
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (QMainWindow, QWidget, QFrame, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QSpinBox, QScrollArea, QLineEdit)

class UI(QMainWindow):
  def __init__(self, vision, coordinator):
    super().__init__()
    self.vision = vision
    self.coordinator = coordinator
    self.task_coords = {}  # {id: {"start": (x,y), "end": (x,y), "deadline": int}}
    self.selected_task = None
    self.edit_mode = None
    
    self.setWindowTitle('Maze UI')
    self.setGeometry(100, 100, 1200, 800)
    
    main_widget = QWidget()
    self.setCentralWidget(main_widget)
    main_layout = QHBoxLayout(main_widget)
    
    self.image_label = QLabel()
    self.image_label.setMouseTracking(True)
    self.image_label.installEventFilter(self)
    main_layout.addWidget(self.image_label)
    
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
    self.send_instructions_btn = QPushButton('Send Instructions')
    self.send_instructions_btn.setEnabled(True)
    task_layout.addWidget(self.solver_btn)
    task_layout.addWidget(self.send_instructions_btn)
    main_layout.addWidget(task_panel)
    self.solver_btn.clicked.connect(self.run_solver)
    self.send_instructions_btn.clicked.connect(self.coordinator.execute)

  def eventFilter(self, source, event):
    if (source is self.image_label and event.type() == event.MouseButtonPress and event.button() == Qt.LeftButton and self.edit_mode):      
      grid_pos = self.vision.pixel_to_grid(event.pos().x(), event.pos().y())
      if grid_pos is not None and self.selected_task is not None:
        if self.edit_mode == 'start':
            self.task_coords[self.selected_task]['start'] = grid_pos
            self.edit_mode = 'end'
        elif self.edit_mode == 'end':
            self.task_coords[self.selected_task]['end'] = grid_pos
            self.edit_mode = None
            self.selected_task = None
        self.update_task_display()
    return super().eventFilter(source, event)


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

  # def update_task_display(self):
  #   while self.task_layout.count():
  #     widget = self.task_layout.takeAt(0).widget()
  #     if widget:
  #       widget.deleteLater()

  #   for task_id, task in self.task_coords.items():
  #     task_widget = QWidget()
  #     task_widget.setFixedHeight(180)
  #     task_layout = QVBoxLayout(task_widget)
  #     task_layout.setSpacing(8)
  #     task_layout.setContentsMargins(8, 8, 8, 8)
      
  #     header_widget = QWidget()
  #     header_widget.setFixedHeight(40)
  #     header_layout = QHBoxLayout(header_widget)
  #     header_layout.setContentsMargins(0, 0, 0, 0)
      
  #     task_label = QLabel(f'Task {task_id}')
  #     remove_btn = QPushButton('Remove')
  #     remove_btn.setFixedSize(80, 30)
  #     remove_btn.clicked.connect(lambda checked, x=task_id: self.remove_task(x))      
  #     header_layout.addWidget(task_label)
  #     header_layout.addWidget(remove_btn)
  #     task_layout.addWidget(header_widget)
      
  #     start_widget = QWidget()
  #     start_widget.setFixedHeight(40)
  #     start_layout = QHBoxLayout(start_widget)
  #     start_layout.setContentsMargins(0, 0, 0, 0)
      
  #     start_label = QLabel('Start Point:')
  #     start_btn = QPushButton(str(task['start']) if task['start'] else 'Set Pickup')
  #     start_btn.setFixedSize(120, 30)
  #     start_btn.clicked.connect(lambda checked, x=task_id: self.edit_waypoint(x, 'start'))      
  #     start_layout.addWidget(start_label)
  #     start_layout.addWidget(start_btn)
  #     task_layout.addWidget(start_widget)
      
  #     end_widget = QWidget()
  #     end_widget.setFixedHeight(40)
  #     end_layout = QHBoxLayout(end_widget)
  #     end_layout.setContentsMargins(0, 0, 0, 0)
      
  #     end_label = QLabel('End Point:')
  #     end_btn = QPushButton(str(task['end']) if task['end'] else 'Set Dropoff')
  #     end_btn.setFixedSize(120, 30)  # Fixed size for end button
  #     end_btn.clicked.connect(lambda checked, x=task_id: self.edit_waypoint(x, 'end'))
      
  #     end_layout.addWidget(end_label)
  #     end_layout.addWidget(end_btn)
  #     task_layout.addWidget(end_widget)
      
  #     deadline_widget = QWidget()
  #     deadline_widget.setFixedHeight(40)
  #     deadline_layout = QHBoxLayout(deadline_widget)
  #     deadline_layout.setContentsMargins(0, 0, 0, 0)
      
  #     deadline_label = QLabel('Deadline (ms):')
  #     deadline_spin = QSpinBox()
  #     deadline_spin.setFixedSize(120, 30)
  #     deadline_spin.setRange(0, 1000000)
  #     deadline_spin.setValue(task['deadline'])
  #     deadline_spin.valueChanged.connect(lambda value, x=task_id: self.update_deadline(x, value))
      
  #     deadline_layout.addWidget(deadline_label)
  #     deadline_layout.addWidget(deadline_spin)
  #     task_layout.addWidget(deadline_widget)
      
  #     if task_id < len(self.task_coords) - 1:
  #       line = QFrame()
  #       line.setFrameShape(QFrame.HLine)
  #       line.setFrameShadow(QFrame.Sunken)
  #       task_layout.addWidget(line)
  #     self.task_layout.addWidget(task_widget)
  #   self.task_layout.addStretch()

  def remove_task(self, task_id):
    if task_id in self.task_coords:
      del self.task_coords[task_id]
      new_tasks = {}
      for i, (_, task) in enumerate(sorted(self.task_coords.items())):
          new_tasks[i] = task
      self.task_coords = new_tasks
    self.update_task_display()

  def edit_waypoint(self, task_id, mode):
    self.selected_task = task_id
    self.edit_mode = mode

  def update_deadline(self, task_id, deadline):
    if task_id in self.task_coords:
      self.task_coords[task_id]['deadline'] = deadline    

  def run_solver(self):
    self.solver_btn.setEnabled(False)
    def solver_thread():
      schedules = self.coordinator.run_solver()
      if schedules is not None:
        self.vision.solver_ran = True
        self.send_instructions_btn.setEnabled(True)
      else:
        self.vision.solver_ran = False
      self.solver_btn.setEnabled(True)
    threading.Thread(target=solver_thread, daemon=True).start()

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
      start_input.editingFinished.connect(lambda x=task_id, input=start_input: self.update_waypoint_from_input(x, 'start', input))

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
      end_input.editingFinished.connect(lambda x=task_id, input=end_input: self.update_waypoint_from_input(x, 'end', input))

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

  def update_waypoint_from_input(self, task_id, mode, input_widget):
    text = input_widget.text()
    try:
      x, y = map(int, text.split(','))
      if task_id in self.task_coords:
        self.task_coords[task_id][mode] = (x, y)
        self.update_task_display()
    except ValueError:
      input_widget.setText('')  # Clear invalid input
