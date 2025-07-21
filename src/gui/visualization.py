import pygame
import threading
import time
import math
from typing import Dict, List, Tuple, Optional, Any
from datetime import datetime
import requests
import json

class SwarmDashGUI:
    """
    Real-time GUI visualization for SwarmDash multi-agent delivery system.
    
    This demonstrates:
    - Presentation tier of multi-tiered architecture
    - Real-time data visualization
    - Interactive user interface
    - Integration with distributed services
    """
    
    def __init__(self, width: int = 1200, height: int = 800, 
                 grid_width: int = 50, grid_height: int = 50,
                 server_url: str = "http://localhost:5000"):
        # Initialize pygame
        pygame.init()
        
        # Display settings
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("SwarmDash - Multi-Agent Delivery System")
        
        # Grid settings
        self.grid_width = grid_width
        self.grid_height = grid_height
        self.cell_width = (width - 300) // grid_width  # Leave space for sidebar
        self.cell_height = (height - 100) // grid_height  # Leave space for header
        self.grid_offset_x = 20
        self.grid_offset_y = 60
        
        # Server connection
        self.server_url = server_url.rstrip('/')
        self.last_update = None
        self.connection_status = "disconnected"
        
        # Data storage
        self.robots: Dict[str, Dict] = {}
        self.tasks: Dict[str, Dict] = {}
        self.system_stats = {}
        self.grid_state = {}
        
        # GUI state
        self.running = False
        self.selected_robot = None
        self.selected_task = None
        self.show_paths = True
        self.show_battery = True
        self.show_task_details = True
        
        # Update thread
        self.update_thread = None
        self.update_interval = 1.0  # seconds
        
        # Colors
        self.colors = {
            'background': (240, 240, 240),
            'grid_line': (200, 200, 200),
            'obstacle': (80, 80, 80),
            'charging_station': (255, 215, 0),  # Gold
            'robot_idle': (0, 128, 255),  # Blue
            'robot_en_route': (255, 165, 0),  # Orange
            'robot_delivering': (50, 205, 50),  # Lime green
            'robot_charging': (255, 20, 147),  # Deep pink
            'robot_failed': (255, 0, 0),  # Red
            'task_pending': (128, 0, 128),  # Purple
            'task_assigned': (255, 140, 0),  # Dark orange
            'task_in_progress': (34, 139, 34),  # Forest green
            'path': (255, 192, 203),  # Pink
            'text': (50, 50, 50),
            'header': (70, 130, 180),  # Steel blue
            'sidebar': (248, 248, 255)  # Ghost white
        }
        
        # Fonts
        self.font_large = pygame.font.Font(None, 24)
        self.font_medium = pygame.font.Font(None, 18)
        self.font_small = pygame.font.Font(None, 14)
        
    def start_visualization(self):
        """Start the GUI visualization with real-time updates."""
        self.running = True
        
        # Start data update thread
        self.update_thread = threading.Thread(
            target=self._update_data_loop,
            daemon=True
        )
        self.update_thread.start()
        
        # Main GUI loop
        clock = pygame.time.Clock()
        
        while self.running:
            # Handle events
            for event in pygame.event.get():
                self._handle_event(event)
                
            # Draw everything
            self._draw_frame()
            
            # Update display
            pygame.display.flip()
            clock.tick(60)  # 60 FPS
            
        pygame.quit()
        
    def _handle_event(self, event):
        """Handle pygame events."""
        if event.type == pygame.QUIT:
            self.running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                self.running = False
            elif event.key == pygame.K_p:
                self.show_paths = not self.show_paths
            elif event.key == pygame.K_b:
                self.show_battery = not self.show_battery
            elif event.key == pygame.K_t:
                self.show_task_details = not self.show_task_details
            elif event.key == pygame.K_r:
                self._refresh_data()
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:  # Left click
                self._handle_mouse_click(event.pos)
                
    def _handle_mouse_click(self, pos: Tuple[int, int]):
        """Handle mouse clicks for robot/task selection."""
        grid_x = (pos[0] - self.grid_offset_x) // self.cell_width
        grid_y = (pos[1] - self.grid_offset_y) // self.cell_height
        
        if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
            # Check if clicked on a robot
            for robot_id, robot_data in self.robots.items():
                robot_pos = robot_data.get('location', [0, 0])
                if robot_pos[0] == grid_x and robot_pos[1] == grid_y:
                    self.selected_robot = robot_id
                    self.selected_task = None
                    return
                    
            # Check if clicked on a task
            for task_id, task_data in self.tasks.items():
                pickup_pos = task_data.get('pickup', [0, 0])
                dropoff_pos = task_data.get('dropoff', [0, 0])
                if ((pickup_pos[0] == grid_x and pickup_pos[1] == grid_y) or
                    (dropoff_pos[0] == grid_x and dropoff_pos[1] == grid_y)):
                    self.selected_task = task_id
                    self.selected_robot = None
                    return
                    
            # Clear selection if clicked on empty space
            self.selected_robot = None
            self.selected_task = None
            
    def _update_data_loop(self):
        """Background thread to fetch data from server."""
        while self.running:
            try:
                self._fetch_system_data()
                self.connection_status = "connected"
                self.last_update = datetime.now()
            except Exception as e:
                self.connection_status = "error"
                print(f"Data update error: {e}")
                
            time.sleep(self.update_interval)
            
    def _fetch_system_data(self):
        """Fetch current system state from server."""
        try:
            # Get system status
            response = requests.get(f"{self.server_url}/api/system/status", timeout=2.0)
            if response.status_code == 200:
                self.system_stats = response.json()
                
            # Get robot data (if we have robots)
            if 'grid_statistics' in self.system_stats:
                grid_stats = self.system_stats['grid_statistics']
                if 'robots' in grid_stats:
                    self.robots = grid_stats['robots']
                if 'active_tasks' in grid_stats:
                    self.tasks = grid_stats['active_tasks']
                    
        except requests.RequestException as e:
            raise e
            
    def _refresh_data(self):
        """Manually refresh data from server."""
        try:
            self._fetch_system_data()
            print("Data refreshed manually")
        except Exception as e:
            print(f"Manual refresh failed: {e}")
            
    def _draw_frame(self):
        """Draw the complete GUI frame."""
        # Clear screen
        self.screen.fill(self.colors['background'])
        
        # Draw header
        self._draw_header()
        
        # Draw grid
        self._draw_grid()
        
        # Draw obstacles and charging stations
        self._draw_static_elements()
        
        # Draw tasks
        self._draw_tasks()
        
        # Draw robots
        self._draw_robots()
        
        # Draw paths if enabled
        if self.show_paths:
            self._draw_paths()
            
        # Draw sidebar
        self._draw_sidebar()
        
        # Draw selection highlights
        self._draw_selection_highlights()
        
    def _draw_header(self):
        """Draw the header with system information."""
        header_rect = pygame.Rect(0, 0, self.width, 50)
        pygame.draw.rect(self.screen, self.colors['header'], header_rect)
        
        # Title
        title_text = self.font_large.render("SwarmDash - Multi-Agent Delivery System", True, (255, 255, 255))
        self.screen.blit(title_text, (10, 15))
        
        # Connection status
        status_color = (0, 255, 0) if self.connection_status == "connected" else (255, 0, 0)
        status_text = self.font_medium.render(f"Status: {self.connection_status}", True, status_color)
        self.screen.blit(status_text, (self.width - 200, 10))
        
        # Last update time
        if self.last_update:
            time_text = self.font_small.render(f"Updated: {self.last_update.strftime('%H:%M:%S')}", True, (255, 255, 255))
            self.screen.blit(time_text, (self.width - 200, 30))
            
    def _draw_grid(self):
        """Draw the grid lines."""
        grid_rect = pygame.Rect(
            self.grid_offset_x, 
            self.grid_offset_y,
            self.grid_width * self.cell_width,
            self.grid_height * self.cell_height
        )
        pygame.draw.rect(self.screen, (255, 255, 255), grid_rect)
        
        # Vertical lines
        for x in range(self.grid_width + 1):
            start_x = self.grid_offset_x + x * self.cell_width
            pygame.draw.line(
                self.screen, 
                self.colors['grid_line'],
                (start_x, self.grid_offset_y),
                (start_x, self.grid_offset_y + self.grid_height * self.cell_height)
            )
            
        # Horizontal lines
        for y in range(self.grid_height + 1):
            start_y = self.grid_offset_y + y * self.cell_height
            pygame.draw.line(
                self.screen,
                self.colors['grid_line'],
                (self.grid_offset_x, start_y),
                (self.grid_offset_x + self.grid_width * self.cell_width, start_y)
            )
            
    def _draw_static_elements(self):
        """Draw obstacles and charging stations."""
        # This would normally come from server data
        # For now, draw some example charging stations
        charging_stations = [(0, 0), (49, 0), (0, 49), (49, 49), (25, 25)]
        
        for station in charging_stations:
            self._draw_cell(station[0], station[1], self.colors['charging_station'])
            
    def _draw_tasks(self):
        """Draw delivery tasks (pickup and dropoff locations)."""
        for task_id, task_data in self.tasks.items():
            pickup = task_data.get('pickup', [0, 0])
            dropoff = task_data.get('dropoff', [0, 0])
            status = task_data.get('status', 'pending')
            priority = task_data.get('priority', 1)
            
            # Choose color based on status
            color = self.colors.get(f'task_{status}', self.colors['task_pending'])
            
            # Draw pickup location (square)
            self._draw_cell(pickup[0], pickup[1], color, 'square')
            
            # Draw dropoff location (diamond)
            self._draw_cell(dropoff[0], dropoff[1], color, 'diamond')
            
            # Draw connection line
            pickup_center = self._grid_to_screen(pickup[0], pickup[1])
            dropoff_center = self._grid_to_screen(dropoff[0], dropoff[1])
            pygame.draw.line(self.screen, color, pickup_center, dropoff_center, 2)
            
            # Draw priority indicator
            if priority > 1:
                priority_text = self.font_small.render(str(priority), True, (255, 255, 255))
                text_pos = (pickup_center[0] - 5, pickup_center[1] - 5)
                self.screen.blit(priority_text, text_pos)
                
    def _draw_robots(self):
        """Draw robots with status indicators."""
        for robot_id, robot_data in self.robots.items():
            location = robot_data.get('location', [0, 0])
            status = robot_data.get('status', 'idle')
            battery = robot_data.get('battery', '100/100')
            
            # Choose color based on status
            color = self.colors.get(f'robot_{status}', self.colors['robot_idle'])
            
            # Draw robot as circle
            center = self._grid_to_screen(location[0], location[1])
            pygame.draw.circle(self.screen, color, center, self.cell_width // 3)
            pygame.draw.circle(self.screen, (0, 0, 0), center, self.cell_width // 3, 2)
            
            # Draw battery indicator if enabled
            if self.show_battery:
                battery_parts = battery.split('/')
                if len(battery_parts) == 2:
                    current = int(battery_parts[0])
                    max_battery = int(battery_parts[1])
                    battery_ratio = current / max_battery
                    
                    # Battery bar
                    bar_width = self.cell_width // 2
                    bar_height = 4
                    bar_x = center[0] - bar_width // 2
                    bar_y = center[1] + self.cell_height // 3
                    
                    # Background
                    pygame.draw.rect(self.screen, (100, 100, 100), 
                                   (bar_x, bar_y, bar_width, bar_height))
                    
                    # Battery level
                    battery_color = (0, 255, 0) if battery_ratio > 0.5 else (255, 255, 0) if battery_ratio > 0.2 else (255, 0, 0)
                    pygame.draw.rect(self.screen, battery_color,
                                   (bar_x, bar_y, int(bar_width * battery_ratio), bar_height))
                                   
    def _draw_paths(self):
        """Draw robot paths if available."""
        # This would require path data from server
        # For now, skip path drawing
        pass
        
    def _draw_sidebar(self):
        """Draw the information sidebar."""
        sidebar_x = self.grid_offset_x + self.grid_width * self.cell_width + 20
        sidebar_width = self.width - sidebar_x - 10
        
        # Sidebar background
        sidebar_rect = pygame.Rect(sidebar_x, self.grid_offset_y, sidebar_width, self.height - self.grid_offset_y - 10)
        pygame.draw.rect(self.screen, self.colors['sidebar'], sidebar_rect)
        pygame.draw.rect(self.screen, (200, 200, 200), sidebar_rect, 2)
        
        y_offset = self.grid_offset_y + 10
        
        # System statistics
        if self.system_stats:
            y_offset = self._draw_system_stats(sidebar_x + 10, y_offset, sidebar_width - 20)
            
        # Selected robot details
        if self.selected_robot and self.selected_robot in self.robots:
            y_offset = self._draw_robot_details(sidebar_x + 10, y_offset, sidebar_width - 20)
            
        # Selected task details
        if self.selected_task and self.selected_task in self.tasks:
            y_offset = self._draw_task_details(sidebar_x + 10, y_offset, sidebar_width - 20)
            
        # Controls help
        self._draw_controls_help(sidebar_x + 10, self.height - 150, sidebar_width - 20)
        
    def _draw_system_stats(self, x: int, y: int, width: int) -> int:
        """Draw system statistics section."""
        title = self.font_medium.render("System Statistics", True, self.colors['text'])
        self.screen.blit(title, (x, y))
        y += 25
        
        if 'grid_statistics' in self.system_stats:
            stats = self.system_stats['grid_statistics']
            
            stats_to_show = [
                ("Total Robots", stats.get('total_robots', 0)),
                ("Active Robots", stats.get('active_robots', 0)),
                ("Total Tasks", stats.get('total_tasks', 0)),
                ("Pending Tasks", stats.get('pending_tasks', 0)),
                ("Completed Tasks", stats.get('completed_tasks', 0)),
                ("Avg Battery", f"{stats.get('average_battery', 0):.1f}%"),
                ("System Efficiency", f"{stats.get('system_efficiency', 0):.3f}"),
            ]
            
            for label, value in stats_to_show:
                text = self.font_small.render(f"{label}: {value}", True, self.colors['text'])
                self.screen.blit(text, (x, y))
                y += 18
                
        return y + 10
        
    def _draw_robot_details(self, x: int, y: int, width: int) -> int:
        """Draw selected robot details."""
        title = self.font_medium.render(f"Robot {self.selected_robot[:8]}", True, self.colors['text'])
        self.screen.blit(title, (x, y))
        y += 25
        
        robot_data = self.robots[self.selected_robot]
        
        details = [
            ("Location", tuple(robot_data.get('location', [0, 0]))),
            ("Battery", robot_data.get('battery', 'Unknown')),
            ("Status", robot_data.get('status', 'Unknown')),
            ("Tasks Done", robot_data.get('tasks_completed', 0)),
            ("Distance", f"{robot_data.get('total_distance', 0):.1f}"),
            ("Efficiency", robot_data.get('efficiency', 0))
        ]
        
        for label, value in details:
            text = self.font_small.render(f"{label}: {value}", True, self.colors['text'])
            self.screen.blit(text, (x, y))
            y += 18
            
        return y + 10
        
    def _draw_task_details(self, x: int, y: int, width: int) -> int:
        """Draw selected task details."""
        title = self.font_medium.render(f"Task {self.selected_task[:8]}", True, self.colors['text'])
        self.screen.blit(title, (x, y))
        y += 25
        
        task_data = self.tasks[self.selected_task]
        
        details = [
            ("Pickup", tuple(task_data.get('pickup', [0, 0]))),
            ("Dropoff", tuple(task_data.get('dropoff', [0, 0]))),
            ("Priority", task_data.get('priority', 1)),
            ("Status", task_data.get('status', 'Unknown'))
        ]
        
        for label, value in details:
            text = self.font_small.render(f"{label}: {value}", True, self.colors['text'])
            self.screen.blit(text, (x, y))
            y += 18
            
        return y + 10
        
    def _draw_controls_help(self, x: int, y: int, width: int):
        """Draw controls help section."""
        title = self.font_medium.render("Controls", True, self.colors['text'])
        self.screen.blit(title, (x, y))
        y += 25
        
        controls = [
            "Click: Select robot/task",
            "P: Toggle paths",
            "B: Toggle battery indicators",
            "T: Toggle task details",
            "R: Refresh data",
            "ESC: Exit"
        ]
        
        for control in controls:
            text = self.font_small.render(control, True, self.colors['text'])
            self.screen.blit(text, (x, y))
            y += 16
            
    def _draw_selection_highlights(self):
        """Draw highlights for selected robot/task."""
        if self.selected_robot and self.selected_robot in self.robots:
            location = self.robots[self.selected_robot].get('location', [0, 0])
            center = self._grid_to_screen(location[0], location[1])
            pygame.draw.circle(self.screen, (255, 255, 0), center, self.cell_width // 2, 3)
            
        if self.selected_task and self.selected_task in self.tasks:
            task_data = self.tasks[self.selected_task]
            pickup = task_data.get('pickup', [0, 0])
            dropoff = task_data.get('dropoff', [0, 0])
            
            pickup_center = self._grid_to_screen(pickup[0], pickup[1])
            dropoff_center = self._grid_to_screen(dropoff[0], dropoff[1])
            
            pygame.draw.circle(self.screen, (255, 255, 0), pickup_center, self.cell_width // 2, 3)
            pygame.draw.circle(self.screen, (255, 255, 0), dropoff_center, self.cell_width // 2, 3)
            
    def _draw_cell(self, grid_x: int, grid_y: int, color: Tuple[int, int, int], shape: str = 'circle'):
        """Draw a cell at grid coordinates with specified color and shape."""
        center = self._grid_to_screen(grid_x, grid_y)
        size = min(self.cell_width, self.cell_height) // 4
        
        if shape == 'circle':
            pygame.draw.circle(self.screen, color, center, size)
        elif shape == 'square':
            rect = pygame.Rect(center[0] - size, center[1] - size, size * 2, size * 2)
            pygame.draw.rect(self.screen, color, rect)
        elif shape == 'diamond':
            points = [
                (center[0], center[1] - size),
                (center[0] + size, center[1]),
                (center[0], center[1] + size),
                (center[0] - size, center[1])
            ]
            pygame.draw.polygon(self.screen, color, points)
            
    def _grid_to_screen(self, grid_x: int, grid_y: int) -> Tuple[int, int]:
        """Convert grid coordinates to screen coordinates."""
        screen_x = self.grid_offset_x + grid_x * self.cell_width + self.cell_width // 2
        screen_y = self.grid_offset_y + grid_y * self.cell_height + self.cell_height // 2
        return (screen_x, screen_y)

def create_task_via_gui(server_url: str, pickup: Tuple[int, int], dropoff: Tuple[int, int], priority: int = 1):
    """Helper function to create tasks via GUI interaction."""
    try:
        payload = {
            "pickup_x": pickup[0],
            "pickup_y": pickup[1],
            "dropoff_x": dropoff[0],
            "dropoff_y": dropoff[1],
            "priority": priority
        }
        
        response = requests.post(f"{server_url}/api/tasks", json=payload, timeout=5.0)
        return response.status_code == 201
    except Exception as e:
        print(f"Failed to create task: {e}")
        return False