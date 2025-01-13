import sys
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QFrame, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QSpinBox, QScrollArea)
import threading
from PyQt5.QtCore import Qt, pyqtSignal, QObject

class SignalEmitter(QObject):
  solver_completed = pyqtSignal(bool)

class UI(QMainWindow):
  def __init__(self, vision):
    super().__init__()
    self.vision = vision
    self.setWindowTitle('Maze UI')
    self.setGeometry(100, 100, 1200, 800)

    main_widget = QWidget()
    self.setCentralWidget(main_widget)
    main_layout = QHBoxLayout(main_widget) 

    camera_container = QWidget()
    camera_layout = QVBoxLayout(camera_container)
    self.image_label = QLabel()
    self.image_label.setMouseTracking(True)
    self.image_label.installEventFilter(self)
    camera_layout.addWidget(self.image_label)
    main_layout.addWidget(camera_container)

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
    self.solver_btn.clicked.connect(self.run_solver)
    task_layout.addWidget(self.solver_btn)
    self.signals = SignalEmitter()
    self.signals.solver_completed.connect(self.on_solver_completed)

    self.send_instructions_btn = QPushButton('Send Instructions')
    self.send_instructions_btn.setEnabled(False)
    self.send_instructions_btn.clicked.connect(self.send_instructions)
    task_layout.addWidget(self.send_instructions_btn)

    main_layout.addWidget(task_panel)

    self.update_task_display()

  def eventFilter(self, source, event):
    if (source is self.image_label and event.type() == event.MouseButtonPress and event.button() == Qt.LeftButton and self.vision.edit_mode):
      x = event.pos().x()
      y = event.pos().y()
      
      grid_pos = self.vision.pixel_to_grid(x, y)
      
      if grid_pos is not None and self.vision.selected_task is not None:
        if self.vision.edit_mode == 'start':
            self.vision.tasks[self.vision.selected_task]['start'] = grid_pos
        elif self.vision.edit_mode == 'end':
            self.vision.tasks[self.vision.selected_task]['end'] = grid_pos
        
        self.vision.edit_mode = None
        self.vision.selected_task = None
        
        self.update_task_display()
    
    return super().eventFilter(source, event)

  def add_task(self):
    task_id = len(self.vision.tasks)
    self.vision.tasks[task_id] = {
        "id": task_id,
        "start": None,
        "end": None,
        "deadline": 10000
    }
    self.update_task_display()

  def update_task_display(self):
    while self.task_layout.count():
      widget = self.task_layout.takeAt(0).widget()
      if widget:
        widget.deleteLater()

    for task_id, task in self.vision.tasks.items():
      task_widget = QWidget()
      task_widget.setFixedHeight(180)
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
      start_btn = QPushButton(str(task['start']) if task['start'] else 'Set Start')
      start_btn.setFixedSize(120, 30)
      start_btn.clicked.connect(lambda checked, x=task_id: self.edit_waypoint(x, 'start'))
      
      start_layout.addWidget(start_label)
      start_layout.addWidget(start_btn)
      task_layout.addWidget(start_widget)
      
      end_widget = QWidget()
      end_widget.setFixedHeight(40)
      end_layout = QHBoxLayout(end_widget)
      end_layout.setContentsMargins(0, 0, 0, 0)
      
      end_label = QLabel('End Point:')
      end_btn = QPushButton(str(task['end']) if task['end'] else 'Set End')
      end_btn.setFixedSize(120, 30)  # Fixed size for end button
      end_btn.clicked.connect(lambda checked, x=task_id: self.edit_waypoint(x, 'end'))
      
      end_layout.addWidget(end_label)
      end_layout.addWidget(end_btn)
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
      
      # Add line separator between tasks
      if task_id < len(self.vision.tasks) - 1:
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        task_layout.addWidget(line)
      self.task_layout.addWidget(task_widget)
    self.task_layout.addStretch()

  def remove_task(self, task_id):
    if task_id in self.vision.tasks:
      del self.vision.tasks[task_id]
      new_tasks = {}
      for i, (_, task) in enumerate(sorted(self.vision.tasks.items())):
          new_tasks[i] = task
      self.vision.tasks = new_tasks
    self.update_task_display()

  def edit_waypoint(self, task_id, mode):
    self.vision.selected_task = task_id
    self.vision.edit_mode = mode

  def update_deadline(self, task_id, deadline):
    if task_id in self.vision.tasks:
      self.vision.tasks[task_id]['deadline'] = deadline

  def on_solver_completed(self, success):
    if success:
      self.vision.solver_status = 'completed'
      self.send_instructions_btn.setEnabled(True)
    else:
      self.vision.solver_status = 'idle'
    self.solver_btn.setEnabled(True)

  def run_solver(self):
    self.vision.solver_status = 'running'
    self.solver_btn.setEnabled(False)
    def solver_thread():
      schedules = self.vision.callback()
      self.signals.solver_completed.emit(bool(schedules))
    threading.Thread(target=solver_thread, daemon=True).start()

  def send_instructions(self):
    if self.vision.solver_status == 'completed':
      instructions_set = self.vision.coordinator.generate_p2p_movement_instructions(self.vision.schedules)
      self.vision.coordinator.send_instructions(instructions_set)

def create_ui(vision):
  app = QApplication(sys.argv)
  window = UI(vision)
  window.show()
  return app, window