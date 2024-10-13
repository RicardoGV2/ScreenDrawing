import sys
from PyQt5.QtWidgets import (
    QApplication, QOpenGLWidget, QLabel, QMainWindow,
    QGraphicsView, QGraphicsScene, QGraphicsPolygonItem,
    QPushButton, QSlider, QWidget, QVBoxLayout, QHBoxLayout
)
from PyQt5.QtCore import Qt, QTimer, QPointF, QRectF
from PyQt5.QtGui import (
    QBrush, QColor, QPainter, QPolygonF, QPen, QSurfaceFormat
)
from OpenGL.GL import *
from OpenGL.GLU import *
import screeninfo
import math

# Set up the surface format
format = QSurfaceFormat()
format.setAlphaBufferSize(8)
QSurfaceFormat.setDefaultFormat(format)

# Define the cube vertices
vertices = [
    [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
    [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]
]

class BoundaryWindow(QMainWindow):
    """
    Separate window for creating and visualizing polygon boundaries.
    Allows freehand drawing by tracking mouse movements.
    """
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Polygon Boundary Visualization")
        self.setGeometry(100, 100, 800, 600)  # Adjust position and size as needed

        # Set up the central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # Set up layout
        layout = QVBoxLayout()
        central_widget.setLayout(layout)

        # Set up QGraphicsScene and QGraphicsView
        self.polygon_scene = QGraphicsScene()
        self.polygon_view = QGraphicsView(self.polygon_scene)
        self.polygon_view.setRenderHint(QPainter.Antialiasing)
        self.polygon_view.setFixedSize(800, 600)  # Adjust as needed
        layout.addWidget(self.polygon_view)

        # Initialize polygon points
        self.polygon_points = []
        self.hull_points = []
        self.is_drawing = False

        # Create polygon item
        self.polygon_item = QGraphicsPolygonItem()
        self.polygon_item.setBrush(QBrush(QColor(0, 0, 255, 50)))  # Semi-transparent fill
        self.polygon_item.setPen(QPen(QColor(0, 0, 255), 2))  # Blue outline
        self.polygon_scene.addItem(self.polygon_item)

        # Create a temporary path for drawing
        self.temp_path = QGraphicsPolygonItem()
        self.temp_path.setBrush(QBrush(QColor(255, 0, 0, 50)))  # Semi-transparent red fill for temporary path
        self.temp_path.setPen(QPen(QColor(255, 0, 0), 1, Qt.DashLine))  # Red dashed outline
        self.polygon_scene.addItem(self.temp_path)

        # Set up control buttons
        buttons_layout = QHBoxLayout()
        layout.addLayout(buttons_layout)

        # Finalize Polygon Button
        self.finalize_button = QPushButton("Finalize Polygon")
        self.finalize_button.clicked.connect(self.finalize_polygon)
        buttons_layout.addWidget(self.finalize_button)

        # Clear Polygon Button
        self.clear_button = QPushButton("Clear Polygon")
        self.clear_button.clicked.connect(self.clear_polygon)
        buttons_layout.addWidget(self.clear_button)

        # Instructions Label
        self.instructions_label = QLabel("Hold Left-Button and draw to create polygon. Click 'Finalize Polygon' when done.")
        layout.addWidget(self.instructions_label)

        # Mouse Tracking
        self.polygon_view.setMouseTracking(True)
        self.polygon_view.viewport().installEventFilter(self)

    def eventFilter(self, source, event):
        if source == self.polygon_view.viewport():
            if event.type() == event.MouseButtonPress and event.button() == Qt.LeftButton:
                self.is_drawing = True
                self.polygon_points = []
                self.temp_path.setPolygon(QPolygonF())
                pos = event.pos()
                scene_pos = self.polygon_view.mapToScene(pos)
                self.polygon_points.append((scene_pos.x(), scene_pos.y()))
                self.update_temp_path()
                return True
            elif event.type() == event.MouseMove and self.is_drawing:
                pos = event.pos()
                scene_pos = self.polygon_view.mapToScene(pos)
                self.polygon_points.append((scene_pos.x(), scene_pos.y()))
                self.update_temp_path()
                return True
            elif event.type() == event.MouseButtonRelease and event.button() == Qt.LeftButton:
                self.is_drawing = False
                return True
        return super().eventFilter(source, event)

    def update_temp_path(self):
        """
        Update the temporary path while drawing.
        """
        if len(self.polygon_points) < 2:
            self.temp_path.setPolygon(QPolygonF())
            return
        polygon = QPolygonF([QPointF(x, y) for x, y in self.polygon_points])
        self.temp_path.setPolygon(polygon)

    def finalize_polygon(self):
        """
        Finalize the polygon by computing its convex hull and displaying it.
        """
        if len(self.polygon_points) < 3:
            print("Need at least 3 points to finalize the polygon.")
            return
        self.hull_points = self.convex_hull(self.polygon_points)
        polygon = QPolygonF([QPointF(x, y) for x, y in self.hull_points])
        self.polygon_item.setPolygon(polygon)
        self.temp_path.setPolygon(QPolygonF())  # Clear temporary path
        print("Polygon finalized.")

    def clear_polygon(self):
        """
        Clear the current polygon and temporary path.
        """
        self.polygon_points = []
        self.hull_points = []
        self.polygon_item.setPolygon(QPolygonF())
        self.temp_path.setPolygon(QPolygonF())
        self.is_drawing = False
        print("Polygon cleared.")

    def convex_hull(self, points):
        """
        Compute the convex hull of a set of 2D points using Graham's Scan algorithm.
        """
        if len(points) <= 1:
            return points

        # Sort the points lexicographically (first by x, then by y)
        points = sorted(points, key=lambda p: (p[0], p[1]))

        # Define the orientation function
        def orientation(p, q, r):
            return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

        # Build the lower hull
        lower = []
        for p in points:
            while len(lower) >= 2 and orientation(lower[-2], lower[-1], p) <= 0:
                lower.pop()
            lower.append(p)

        # Build the upper hull
        upper = []
        for p in reversed(points):
            while len(upper) >= 2 and orientation(upper[-2], upper[-1], p) <= 0:
                upper.pop()
            upper.append(p)

        # Concatenate lower and upper to get full hull
        # Last point of each list is omitted because it is duplicated at the beginning of the other list
        full_hull = lower[:-1] + upper[:-1]
        return full_hull

class CubeWidget(QOpenGLWidget):
    """
    OpenGL Widget to display a rotating cube that moves and bounces off polygon boundaries.
    """
    def __init__(self, screens, boundaries, boundary_window):
        super().__init__()

        # Initialize scale_factor early to prevent AttributeError
        self.scale_factor = 0.5  # Default scale factor corresponding to slider value

        self.boundary_window = boundary_window  # Reference to BoundaryWindow

        # Initialize rotation angles
        self.angle_x = 0
        self.angle_y = 0
        self.angle_z = 0
        print("Initialized rotation angles.")

        # Initialize other attributes
        self.line_color = QColor(0, 255, 0)  # Green for additional visuals if needed
        print("Initialized polygon attributes.")

        # Initialize UI after setting up attributes
        self.initUI(screens, boundaries)
        print("UI initialized.")

        # Timer for cube rotation and movement
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_rotation)
        self.timer.timeout.connect(self.update_position)
        # Do not start the timer here; start it after pressing the toggle loop button

        # Velocity in x and y directions
        self.velocity = [3.0, 2.0]  # Initial velocity

        # Initialize cube position
        self.x_pos = self.x()
        self.y_pos = self.y()

    def initUI(self, screens, boundaries):
        """
        Initialize the UI components of the CubeWidget.
        """
        # Remove window borders and make it frameless and stay on top
        self.setWindowFlags(Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint)

        # Make the window transparent
        self.setAttribute(Qt.WA_TranslucentBackground)

        # Ensure the window accepts mouse events
        self.setAttribute(Qt.WA_TransparentForMouseEvents, False)

        # Calculate the combined geometry of all screens (for multi-monitor setups)
        if screens:
            # Find the bounding rectangle that includes all screens
            min_x = min(screen.x for screen in screens)
            min_y = min(screen.y for screen in screens)
            max_x = max(screen.x + screen.width for screen in screens)
            max_y = max(screen.y + screen.height for screen in screens)
            total_width = max_x - min_x
            total_height = max_y - min_y
            self.setGeometry(min_x, min_y, total_width, total_height)
        else:
            self.setGeometry(100, 100, 800, 600)  # Default size if no screens found

        # Initialize position based on current geometry
        self.x_pos = self.x()
        self.y_pos = self.y()

        # Show the window
        self.show()
        print("Overlay window shown.")

        # Set up a label to display mouse coordinates
        self.mouse_label = QLabel(self)
        self.mouse_label.setWindowFlags(Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint)
        self.mouse_label.setAttribute(Qt.WA_TranslucentBackground)
        self.mouse_label.setStyleSheet("color: white; background: rgba(0, 0, 0, 150);")
        self.mouse_label.setGeometry(50, 50, 300, 50)
        self.mouse_label.show()

        # Set up a slider to control the cube size
        self.size_slider = QSlider(Qt.Horizontal, self)
        self.size_slider.setGeometry(10, 10, 150, 30)
        self.size_slider.setMinimum(10)
        self.size_slider.setMaximum(150)
        self.size_slider.setValue(int(self.scale_factor * 100))  # Default value corresponds to scale factor 0.5
        self.size_slider.setTickPosition(QSlider.TicksBelow)
        self.size_slider.setTickInterval(10)
        self.size_slider.valueChanged.connect(self.change_cube_size)
        self.size_slider.show()

        # Set up a button to start/stop cube movement
        self.loop_button = QPushButton("Start Cube Movement", self)
        self.loop_button.setGeometry(170, 10, 150, 30)
        self.loop_button.clicked.connect(self.toggle_loop)
        self.loop_button.show()

        # Timer to update mouse coordinates label
        self.mouse_timer = QTimer()
        self.mouse_timer.timeout.connect(self.update_mouse_label)
        self.mouse_timer.start(50)  # Update every 50 ms

    def initializeGL(self):
        """
        Initialize OpenGL settings.
        """
        print("Initializing OpenGL.")
        # Initialize OpenGL settings
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glClearColor(0.0, 0.0, 0.0, 0.0)  # Fully transparent background

        glMatrixMode(GL_PROJECTION)
        aspect_ratio = self.width() / self.height() if self.height() != 0 else 1
        gluPerspective(45, aspect_ratio, 0.1, 100.0)

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glTranslatef(0.0, 0.0, -5)  # Move the cube back to be visible
        print("OpenGL initialized.")

    def resizeGL(self, w, h):
        """
        Handle window resizing.
        """
        print(f"Resizing OpenGL viewport to {w}x{h}.")
        # Adjust the OpenGL viewport and projection matrix when the window is resized
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, w / h if h != 0 else 1, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glTranslatef(0.0, 0.0, -5)  # Ensure the cube stays in view after resizing

    def paintGL(self):
        """
        Render the OpenGL scene.
        """
        # Clear buffers
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glPushMatrix()
        # Apply scaling based on the slider value
        glScalef(self.scale_factor, self.scale_factor, self.scale_factor)
        # Apply rotations
        glRotatef(self.angle_x, 1, 0, 0)
        glRotatef(self.angle_y, 0, 1, 0)
        glRotatef(self.angle_z, 0, 0, 1)
        # Draw the cube
        self.draw_cube()
        glPopMatrix()

    def draw_cube(self):
        """
        Draw the cube using OpenGL lines.
        """
        # Define the 8 vertices of the cube
        glColor3f(1.0, 1.0, 1.0)  # White color
        glBegin(GL_LINES)
        for edge in [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7)
        ]:
            for vertex in edge:
                glVertex3fv(vertices[vertex])
        glEnd()

    def update_rotation(self):
        """
        Update rotation angles and trigger a repaint.
        """
        # Update rotation angles
        self.angle_x = (self.angle_x + 1) % 360
        self.angle_y = (self.angle_y + 1) % 360
        self.angle_z = (self.angle_z + 1) % 360
        self.update()  # Trigger a repaint
        print(f"Updated rotation angles: x={self.angle_x}, y={self.angle_y}, z={self.angle_z}")

    def toggle_loop(self):
        """
        Toggle the cube's movement and rotation.
        """
        if self.timer.isActive():
            self.timer.stop()
            self.loop_button.setText("Start Cube Movement")
            print("Cube movement stopped.")
        else:
            self.timer.start(16)  # Approximately 60 FPS
            self.loop_button.setText("Stop Cube Movement")
            print("Cube movement started.")

    def change_cube_size(self, value):
        """
        Adjust the cube's scale factor based on the slider value.
        """
        # Slider range: 10 - 150 -> Scale factor: 0.1 - 1.5
        scale_factor = value / 100.0  # 0.1 - 1.5
        print(f"Changing cube scale factor to: {scale_factor}")
        self.scale_factor = scale_factor
        self.update()

    def mousePressEvent(self, event):
        """
        Handle mouse press events.
        Left-Click: Update cube's velocity direction.
        Right-Click: Open the boundary visualization window.
        """
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
                # Clicked exactly at the center; apply a default direction
                dir_x, dir_y = 0, -1  # Push upwards
            
            # Scale the direction vector to set the new velocity magnitude
            push_strength = 5.0  # Adjust this value to control the push strength
            self.velocity = [dir_x * push_strength, dir_y * push_strength]
            print(f"Updated velocity to: {self.velocity}")
        elif event.button() == Qt.RightButton:
            # Right-click detected; open the visualization window if it's closed
            if not self.boundary_window.isVisible():
                self.boundary_window.show()
                print("Polygon visualization window opened.")
            else:
                # Bring the window to the front if it's already open
                self.boundary_window.raise_()
        super().mousePressEvent(event)

    def update_position(self):
        """
        Update the cube's position based on its velocity and handle collisions.
        """
        # Current position
        x_prev = self.x_pos
        y_prev = self.y_pos

        # Tentative new position
        x_new = self.x_pos + self.velocity[0]
        y_new = self.y_pos + self.velocity[1]

        # Movement line segment (from previous to new position)
        p1 = (x_prev + self.width() / 2, y_prev + self.height() / 2)
        p2 = (x_new + self.width() / 2, y_new + self.height() / 2)

        collision_occurred = False

        # Check for collision with each edge of the polygon
        if len(self.boundary_window.hull_points) > 2:
            hull_points = self.boundary_window.hull_points
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
                    print(f"Collision detected. New velocity: {self.velocity}")
                    # Adjust the position to the collision point to prevent getting stuck
                    x_new = collision_point[0] - self.width() / 2 + self.velocity[0]
                    y_new = collision_point[1] - self.height() / 2 + self.velocity[1]
                    break  # Only handle one collision per update

        # Update position
        self.x_pos = x_new
        self.y_pos = y_new

        # Move the window to the new position
        self.move(int(self.x_pos), int(self.y_pos))

        # Update the rotation angles
        self.angle_x = (self.angle_x + 1) % 360
        self.angle_y = (self.angle_y + 1) % 360
        self.angle_z = (self.angle_z + 1) % 360

        # Trigger repaint
        self.update()

    def line_segments_intersect(self, p1, p2, p3, p4):
        """
        Check if two line segments (p1 to p2) and (p3 to p4) intersect.
        Returns a tuple (True/False, (x, y) intersection point or None)
        """
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

    def update_mouse_label(self):
        """
        Update the mouse coordinates label based on the global mouse position.
        """
        cursor_pos = self.cursor().pos()
        screen_num = 0
        for screen in screeninfo.get_monitors():
            if screen.x <= cursor_pos.x() < screen.x + screen.width and \
               screen.y <= cursor_pos.y() < screen.y + screen.height:
                break
            screen_num += 1
        local_x = cursor_pos.x() - screeninfo.get_monitors()[screen_num].x
        local_y = cursor_pos.y() - screeninfo.get_monitors()[screen_num].y
        self.mouse_label.setText(f"Global: ({cursor_pos.x()}, {cursor_pos.y()})\n"
                                 f"Screen {screen_num + 1}: ({local_x}, {local_y})")
        self.mouse_label.adjustSize()
        self.mouse_label.move(cursor_pos.x() + 20, cursor_pos.y() + 20)

if __name__ == '__main__':
    app = QApplication(sys.argv)

    # Get monitor information
    screens = screeninfo.get_monitors()
    boundaries = [sum(screen.width for screen in screens), max(screen.height for screen in screens)]

    # Create BoundaryWindow
    boundary_window = BoundaryWindow()
    boundary_window.show()

    # Create and show CubeWidget
    window = CubeWidget(screens, boundaries, boundary_window)
    window.show()

    sys.exit(app.exec_())
