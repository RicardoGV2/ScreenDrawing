from PyQt5.QtWidgets import QApplication, QOpenGLWidget, QLabel, QMainWindow, QGraphicsView, QGraphicsScene, QGraphicsPolygonItem, QPushButton, QSlider, QCheckBox
from PyQt5.QtCore import Qt, QTimer, QPointF
from PyQt5.QtGui import QBrush, QColor, QPainter, QCursor, QPolygonF, QPen, QSurfaceFormat
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import sys
import numpy as np
import screeninfo
import math

##comment to see the window shape
# Set up the surface format
format = QSurfaceFormat()
format.setAlphaBufferSize(8)
QSurfaceFormat.setDefaultFormat(format)

# Set up points for the initial cube
vertices = [
    [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
    [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]
]

class CubeWidget(QOpenGLWidget):
    def __init__(self, screens, boundaries):
        super().__init__()

        self.window_scale_factor = 0.5  # Initial scaling factor for window size
        # Cube scaling factors
        self.scale_factor = 1.0  # Initial cube scale factor
        self.min_cube_scale = 0.1
        self.max_cube_scale = 2.0

        # Window size fractions
        self.min_window_fraction = 0.1  # Minimum window size is 20% of screen size
        self.max_window_fraction = 0.6  # Maximum window size is 60% of screen size
        self.angle = 0
        self.velocity = [3.0, 2.0]  # Velocity in x and y directions (floats)

        # Set boundaries based on user input
        self.boundaries = boundaries
        self.screen_width = screens[0].width  # Centered on screen 1
        self.screen_height = screens[0].height

        # Set up cube initial position
        self.x_pos = screens[0].width // 2 - 300
        self.y_pos = screens[0].height // 2 - 300
        self.setGeometry(self.x_pos, self.y_pos, 600, 600)
        #self.setAttribute(Qt.WA_TranslucentBackground)
        self.setWindowOpacity(0.8)
        #self.setAttribute(Qt.WA_NoSystemBackground)
        #self.setWindowFlags(Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint | Qt.Tool)
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint)

        # Set up a timer to update the cube's position
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_position)
        # Do not start the timer here; start it after pressing the toggle loop button

        # Store polygon boundaries
        self.polygon_points = []
        self.hull_points = []
        self.loop_enabled = False
        self.cube_positions = []  # List to store cube position markers
        self.max_cube_positions = 1000  # Adjust as needed
        self.polygon_items = []  # List to store polygon items
        self.cube_position_marker = None
        self.mouse_path_items = []  # List to store mouse path items
        self.mouse_path_visible = True  # Variable to track mouse path visibility
        self.mouse_path_points = []  # List to store mouse path points


        

        # Set up a label to display mouse coordinates
        self.mouse_label = QLabel()
        self.mouse_label.setWindowFlags(Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint)
        self.mouse_label.setAttribute(Qt.WA_TranslucentBackground)
        self.mouse_label.setStyleSheet("color: white; background: rgba(0, 0, 0, 150);")
        self.mouse_label.setGeometry(50, 50, 300, 50)
        self.mouse_label.show()

 


        # Set up a window to visualize the polygon boundaries and cube position
        self.polygon_scene = QGraphicsScene()
        self.polygon_view = QGraphicsView()
        self.polygon_view.setScene(self.polygon_scene)
        self.polygon_view.setWindowTitle("Polygon Boundary Visualization")
        self.polygon_view.setGeometry(100, 100, 800, 600)
        self.polygon_view.show()
        self.polygon_view.scale(0.2, 0.2)  # Zoom out to see the entire drawing

        # Set up a button to start/stop cube movement
        self.loop_button = QPushButton("Start Cube Movement", self.polygon_view)
        self.loop_button.setGeometry(10, 10, 150, 30)
        self.loop_button.clicked.connect(self.toggle_loop)
        self.loop_button.show()

        # Timer to draw mouse path
        self.mouse_path_timer = QTimer()
        self.mouse_path_timer.timeout.connect(self.draw_mouse_path)
        self.mouse_path_timer.start(50)  # Update every 50 ms

        # Timer to draw cube position
        self.cube_position_timer = QTimer()
        self.cube_position_timer.timeout.connect(self.draw_cube_position)
        self.cube_position_timer.start(50)  # Update every 50 ms


        # Set up a slider to control the cube size
        self.size_slider = QSlider(Qt.Horizontal, self.polygon_view)
        self.size_slider.setGeometry(10, 50, 150, 30)
        self.size_slider.setMinimum(10)
        self.size_slider.setMaximum(150)
        self.size_slider.setValue(100)  # Default value corresponds to scale factor 1.0
        self.size_slider.setTickPosition(QSlider.TicksBelow)
        self.size_slider.setTickInterval(10)
        self.size_slider.valueChanged.connect(self.change_cube_size)
        self.size_slider.show()

         # Variable to track if polygon updating is enabled
        self.polygon_update_enabled = True

        # Add a button to pause/resume polygon updating
        self.update_button = QPushButton("Pause Polygon Update", self.polygon_view)
        self.update_button.setGeometry(10, 90, 150, 30)
        self.update_button.clicked.connect(self.toggle_polygon_update)
        self.update_button.show()

        # In __init__
        self.clear_button = QPushButton("Clear Polygon", self.polygon_view)
        self.clear_button.setGeometry(10, 130, 150, 30)
        self.clear_button.clicked.connect(self.clear_polygon)
        self.clear_button.show()

        self.cube_tracking_enabled = True  # Variable to track cube position drawing state

        self.cube_tracking_button = QPushButton("Pause Cube Tracking", self.polygon_view)
        self.cube_tracking_button.setGeometry(10, 170, 150, 30)
        self.cube_tracking_button.clicked.connect(self.toggle_cube_tracking)
        self.cube_tracking_button.show()

        # Add a button to clear mouse tracking drawings
        self.clear_mouse_path_button = QPushButton("Clear Mouse Path", self.polygon_view)
        self.clear_mouse_path_button.setGeometry(10, 210, 150, 30)
        self.clear_mouse_path_button.clicked.connect(self.clear_mouse_path)
        self.clear_mouse_path_button.show()

        # Add a checkbox to show/hide the mouse path
        self.show_mouse_path_checkbox = QCheckBox("Show Mouse Path", self.polygon_view)
        self.show_mouse_path_checkbox.setGeometry(10, 250, 150, 30)
        self.show_mouse_path_checkbox.setChecked(True)
        self.show_mouse_path_checkbox.stateChanged.connect(self.toggle_mouse_path_visibility)
        self.show_mouse_path_checkbox.show()

        # Add a label for the window scale slider
        self.window_scale_label = QLabel("Window Scale", self.polygon_view)
        self.window_scale_label.setGeometry(10, 290, 150, 20)
        self.window_scale_label.show()

        # Set up a slider to control the window scaling factor
        self.window_scale_slider = QSlider(Qt.Horizontal, self.polygon_view)
        self.window_scale_slider.setGeometry(10, 310, 150, 30)
        self.window_scale_slider.setMinimum(0)
        self.window_scale_slider.setMaximum(100)
        self.window_scale_slider.setValue(int(self.window_scale_factor * 100))
        self.window_scale_slider.setTickPosition(QSlider.TicksBelow)
        self.window_scale_slider.setTickInterval(10)
        self.window_scale_slider.valueChanged.connect(self.change_window_scale_factor)
        self.window_scale_slider.show()

    def change_window_scale_factor(self, value):
        self.window_scale_factor = value / 100.0  # Convert to a float between 0.0 and 1.0
        print(f"Window scale factor changed to: {self.window_scale_factor}")
        # Update the cube size to apply the new window scale factor
        self.change_cube_size(self.size_slider.value())


    def toggle_mouse_path_visibility(self, state):
        if state == Qt.Checked:
            self.mouse_path_visible = True
            # Show existing mouse path items
            for item in self.mouse_path_items:
                item.show()
            print("Mouse path visibility enabled.")
        else:
            self.mouse_path_visible = False
            # Hide existing mouse path items
            for item in self.mouse_path_items:
                item.hide()
            print("Mouse path visibility disabled.")


    def toggle_cube_tracking(self):
        self.cube_tracking_enabled = not self.cube_tracking_enabled
        if self.cube_tracking_enabled:
            self.cube_tracking_button.setText("Pause Cube Tracking")
            # Restart the timer
            self.cube_position_timer.start(50)
        else:
            self.cube_tracking_button.setText("Resume Cube Tracking")
            # Stop the timer
            self.cube_position_timer.stop()
            # Clear existing cube position markers
            self.clear_cube_positions()
        print(f"Cube tracking enabled: {self.cube_tracking_enabled}")


    def clear_polygon(self):
        self.polygon_points.clear()
        # Remove polygon items from the scene
        for item in self.polygon_items:
            self.polygon_scene.removeItem(item)
        self.polygon_items.clear()
        print("Polygon cleared.")



    def toggle_polygon_update(self):
        self.polygon_update_enabled = not self.polygon_update_enabled
        if self.polygon_update_enabled:
            self.update_button.setText("Pause Polygon Update")
            # Restart the timer
            self.mouse_path_timer.start(50)
        else:
            self.update_button.setText("Resume Polygon Update")
            # Stop the timer
            self.mouse_path_timer.stop()
        print(f"Polygon updating enabled: {self.polygon_update_enabled}")

    def change_cube_size(self, value):
        # Map the slider value to a cube scale factor
        self.scale_factor = value / 100.0  # Cube scale factor

        # Clamp the cube scale factor
        cube_scale = max(self.min_cube_scale, min(self.scale_factor, self.max_cube_scale))

        # Calculate the window size fraction
        window_fraction = self.min_window_fraction + (
            (self.max_window_fraction - self.min_window_fraction) * (cube_scale - self.min_cube_scale)
            / (self.max_cube_scale - self.min_cube_scale)
        )

        # Use a base dimension to compute the window size
        base_dimension = min(self.screen_width, self.screen_height)

        # Compute the new window size
        window_size = int(base_dimension * window_fraction)

        # Ensure the window remains square
        window_width = window_size
        window_height = window_size

        # Calculate the center of the cube before resizing
        center_x = self.x_pos + self.width() // 2
        center_y = self.y_pos + self.height() // 2

        # Update the window size and position to keep the cube centered
        self.setGeometry(
            int(center_x - window_width // 2), int(center_y - window_height // 2), window_width, window_height
        )

        # Update x_pos and y_pos to the new position
        self.x_pos = self.x()
        self.y_pos = self.y()

        # Trigger repaint
        self.update()


        
    def initializeGL(self):
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glClearColor(0.0, 0.0, 0.0, 0.0)  # Set clear color to fully transparent
        glEnable(GL_DEPTH_TEST)
        glMatrixMode(GL_PROJECTION)
        aspect_ratio = self.width() / self.height() if self.height() != 0 else 1
        gluPerspective(45, aspect_ratio, 0.1, 50.0)
        glMatrixMode(GL_MODELVIEW)
        glTranslatef(0.0, 0.0, -7)  # Move the camera back further to see the cube better

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glPushMatrix()
        # Apply scaling
        glScalef(self.scale_factor, self.scale_factor, self.scale_factor)
        glRotatef(self.angle, 1, 1, 1)
        self.draw_cube()
        glPopMatrix()
        # No need to call self.update() here
        
    def draw_cube(self):
        glColor4f(1.0, 1.0, 1.0, 1.0)  # Set cube color to white
        glBegin(GL_LINES)
        for edge in [(0, 1), (1, 2), (2, 3), (3, 0),
                     (4, 5), (5, 6), (6, 7), (7, 4),
                     (0, 4), (1, 5), (2, 6), (3, 7)]:
            for vertex in edge:
                glVertex3fv(vertices[vertex])
        glEnd()

    def resizeGL(self, w, h):
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        aspect_ratio = w / h if h != 0 else 1
        gluPerspective(45, aspect_ratio, 0.1, 50.0)
        glMatrixMode(GL_MODELVIEW)

    def update_position(self):
        # Save current position
        x_prev = self.x_pos
        y_prev = self.y_pos

        # Tentative new position
        x_new = self.x_pos + self.velocity[0]
        y_new = self.y_pos + self.velocity[1]

        # Movement line segment (from cube's center)
        p1 = (x_prev + self.width() // 2, y_prev + self.height() // 2)
        p2 = (x_new + self.width() // 2, y_new + self.height() // 2)

        # Flag to check if collision occurred
        collision_occurred = False

        # Check for collision with each edge
        if len(self.polygon_points) > 2:
            hull_points = self.convex_hull(self.polygon_points)
            n = len(hull_points)
            for i in range(n):
                p3 = hull_points[i]
                p4 = hull_points[(i + 1) % n]
                intersect, collision_point = self.line_segments_intersect(p1, p2, p3, p4)
                if intersect:
                    collision_occurred = True
                    # Compute the normal vector of the edge
                    edge_vector = (p4[0] - p3[0], p4[1] - p3[1])
                    normal_vector = (-edge_vector[1], edge_vector[0])  # Perpendicular to edge vector
                    # Normalize the normal vector
                    normal_length = math.hypot(normal_vector[0], normal_vector[1])
                    if normal_length != 0:
                        normal_vector = (normal_vector[0] / normal_length, normal_vector[1] / normal_length)
                    else:
                        normal_vector = (0, 0)
                    # Reflect the velocity vector
                    v = self.velocity
                    v_dot_n = v[0] * normal_vector[0] + v[1] * normal_vector[1]
                    v_reflected = [
                        v[0] - 2 * v_dot_n * normal_vector[0],
                        v[1] - 2 * v_dot_n * normal_vector[1]
                    ]
                    self.velocity = v_reflected
                    # Adjust the position to the collision point to prevent getting stuck
                    x_new = collision_point[0] - self.width() // 2 + self.velocity[0]
                    y_new = collision_point[1] - self.height() // 2 + self.velocity[1]
                    break  # Only handle one collision per update

        # Update position
        self.x_pos = x_new
        self.y_pos = y_new

        # Move the window to the new position
        self.move(int(self.x_pos), int(self.y_pos))

        # Update the rotation angle
        self.angle += 1

        # Trigger repaint
        self.update()

    def line_segments_intersect(self, p1, p2, p3, p4):
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3
        x4, y4 = p4

        den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if den == 0:
            return False, None  # Lines are parallel

        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den
        u = ((x1 - x3) * (y1 - y2) - (y1 - y3) * (x1 - x2)) / den
        if 0 <= t <= 1 and 0 <= u <= 1:
            # Intersection point
            ix = x1 + t * (x2 - x1)
            iy = y1 + t * (y2 - y1)
            return True, (ix, iy)
        else:
            return False, None

    def point_in_polygon(self, x, y):
        # Use the convex hull for collision detection
        if len(self.polygon_points) < 3:
            return False
        hull_points = self.convex_hull(self.polygon_points)
        n = len(hull_points)
        inside = False
        p1x, p1y = hull_points[0]
        for i in range(n + 1):
            p2x, p2y = hull_points[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        else:
                            xinters = x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape:
            QApplication.quit()

    def update_mouse_label(self):
        cursor_pos = QCursor.pos()
        screen_num = 0
        for screen in screeninfo.get_monitors():
            if screen.x <= cursor_pos.x() < screen.x + screen.width and screen.y <= cursor_pos.y() < screen.y + screen.height:
                break
            screen_num += 1
        local_x = cursor_pos.x() - screens[screen_num].x
        local_y = cursor_pos.y() - screens[screen_num].y
        self.mouse_label.setText(f"Global: ({cursor_pos.x()}, {cursor_pos.y()})\nScreen {screen_num + 1}: ({local_x}, {local_y})")
        self.mouse_label.adjustSize()
        self.mouse_label.move(cursor_pos.x() + 20, cursor_pos.y() + 20)

    def update_polygon_view(self):
        # Remove existing polygon items from the scene
        for item in self.polygon_items:
            self.polygon_scene.removeItem(item)
        self.polygon_items.clear()
        
        if len(self.polygon_points) > 2:
            # Compute convex hull only if new points were added
            if self.polygon_update_enabled:
                self.hull_points = self.convex_hull(self.polygon_points)
            # Use cached hull_points for visualization
            polygon = QPolygonF([QPointF(x, y) for x, y in self.hull_points])
            polygon_item = QGraphicsPolygonItem(polygon)
            polygon_item.setBrush(QBrush(QColor(255, 255, 255, 50)))
            polygon_item.setPen(QPen(QColor(0, 0, 255), 2))  # Blue outline for visibility
            self.polygon_scene.addItem(polygon_item)
            self.polygon_items.append(polygon_item)
        self.polygon_view.raise_()
        self.polygon_view.activateWindow()


    def draw_mouse_path(self):
        if not self.polygon_update_enabled:
            return  # Do nothing if updating is paused

        cursor_pos = QCursor.pos()

        # Update mouse path points
        self.mouse_path_points.append((cursor_pos.x(), cursor_pos.y()))

        if self.mouse_path_visible:
            if self.mouse_path_visible:
                pen = QPen(QColor(255, 0, 0), 2)  # Red line to indicate mouse path
                if len(self.mouse_path_points) > 1:
                    last_point = self.mouse_path_points[-2]
                    # Draw line from last point to current cursor position
                    line_item = self.polygon_scene.addLine(last_point[0], last_point[1], cursor_pos.x(), cursor_pos.y(), pen)
                    self.mouse_path_items.append(line_item)
                else:
                    # First point, just draw a point
                    ellipse_item = self.polygon_scene.addEllipse(cursor_pos.x() - 1, cursor_pos.y() - 1, 2, 2, pen)
                    self.mouse_path_items.append(ellipse_item)

        # Update polygon points and polygon view
        self.polygon_points.append((cursor_pos.x(), cursor_pos.y()))
        self.update_polygon_view()


    def clear_mouse_path(self):
        # Remove mouse path items from the scene
        for item in self.mouse_path_items:
            self.polygon_scene.removeItem(item)
        self.mouse_path_items.clear()
        # Clear the mouse path points but not the polygon points
        self.mouse_path_points.clear()
        print("Mouse path cleared.")



    def draw_cube_position(self):
        if not self.cube_tracking_enabled:
            return  # Do nothing if cube tracking is paused

        # Draw the current position of the cube on the visualization window
        pen = QPen(QColor(0, 255, 0), 4)  # Green dot to indicate cube position
        cube_center_x = self.x_pos + self.width() // 2
        cube_center_y = self.y_pos + self.height() // 2
        ellipse_item = self.polygon_scene.addEllipse(
            cube_center_x - 5, cube_center_y - 5, 10, 10, pen, QBrush(QColor(0, 255, 0))
        )

        # Add the new marker to the list
        self.cube_positions.append(ellipse_item)

        # Enforce maximum number of cube positions
        if len(self.cube_positions) > self.max_cube_positions:
            # Remove the oldest marker from the scene and the list
            old_item = self.cube_positions.pop(0)
            self.polygon_scene.removeItem(old_item)

    def clear_cube_positions(self):
        # Remove all cube position markers from the scene
        if self.cube_positions:
            for item in self.cube_positions:
                self.polygon_scene.removeItem(item)
            self.cube_positions.clear()
        # If using a single marker
        if self.cube_position_marker is not None:
            self.polygon_scene.removeItem(self.cube_position_marker)
            self.cube_position_marker = None


    def toggle_loop(self):
        self.loop_enabled = not self.loop_enabled
        if self.loop_enabled:
            # Start the cube movement and rotation
            self.timer.start(16)  # Approximately 60 FPS
            self.loop_button.setText("Stop Cube Movement")
        else:
            # Stop the cube movement and rotation
            self.timer.stop()
            self.loop_button.setText("Start Cube Movement")
        self.update_polygon_view()

    def convex_hull(self, points):
        # Graham's Scan algorithm
        if len(points) <= 1:
            return points
        P0 = min(points, key=lambda p: (p[1], p[0]))

        # Step 2: Sort points by polar angle with P0
        def polar_angle(p):
            y_span = p[1] - P0[1]
            x_span = p[0] - P0[0]
            return math.atan2(y_span, x_span)

        def distance_squared(p1, p2):
            return (p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2

        sorted_points = sorted(points, key=lambda p: (polar_angle(p), -distance_squared(P0, p)))

        # Step 3: Initialize stack
        stack = [P0]

        # Step 4: Process points
        def orientation(A, B, C):
            return (B[0] - A[0]) * (C[1] - A[1]) - (B[1] - A[1]) * (C[0] - A[0])

        for point in sorted_points[1:]:
            while len(stack) > 1 and orientation(stack[-2], stack[-1], point) <= 0:
                stack.pop()
            stack.append(point)

        return stack

    def mousePressEvent(self, event):


        if event.button() == Qt.LeftButton:
            # Get the click position relative to the cube widget
            click_x = event.x()
            click_y = event.y()
            
            # Calculate the center of the cube widget
            center_x = self.width() / 2
            center_y = self.height() / 2
            
            # Compute the direction vector from the cube center to the click position
            dir_x = click_x - center_x
            dir_y = click_y - center_y
            
            # Normalize the direction vector
            length = math.hypot(dir_x, dir_y)
            if length != 0:
                dir_x /= length
                dir_y /= length
            else:
                # Clicked exactly at the center; apply a small random direction
                dir_x, dir_y = 0, -1  # For example, push upwards
            
            # Scale the direction vector to set the new velocity magnitude
            push_strength = 5.0  # Adjust this value to control the push strength
            self.velocity = [dir_x * push_strength, dir_y * push_strength]
        elif event.button() == Qt.RightButton:
            # Right-click detected; open the visualization window if it's closed
            if not self.polygon_view.isVisible():
                self.polygon_view.show()
                print("Polygon visualization window opened.")
            else:
                # Bring the window to the front if it's already open
                self.polygon_view.raise_()
        self.polygon_view.activateWindow()



if __name__ == '__main__':
    app = QApplication(sys.argv)

    screens = screeninfo.get_monitors()
    boundaries = [sum(screen.width for screen in screens), max(screen.height for screen in screens)]

    window = CubeWidget(screens, boundaries)
    window.show()

    # Timer to update mouse coordinates label
    mouse_timer = QTimer()
    mouse_timer.timeout.connect(window.update_mouse_label)
    mouse_timer.start(50)  # Update every 50 ms

    sys.exit(app.exec_())
