import requests
import threading
import time
import json
from typing import Tuple, Optional, List, Dict, Any
from datetime import datetime

from ..models.robot import Robot
from ..models.task import Task

class RobotClient:
    """
    Robot Client for distributed SwarmDash system.
    
    Demonstrates:
    - Client-server architecture in distributed systems
    - RESTful API communication
    - Autonomous agent behavior
    - Real-time status reporting
    """
    
    def __init__(self, server_url: str, initial_location: Tuple[int, int], 
                 robot_type: str = "drone", max_battery: int = 100, speed: float = 1.0):
        self.server_url = server_url.rstrip('/')
        self.robot = Robot(initial_location, max_battery, speed, robot_type)
        self.robot_id = None  # Will be set after registration
        self.is_running = False
        self.worker_thread = None
        self.current_path: List[Tuple[int, int]] = []
        self.path_index = 0
        
        # Communication settings
        self.update_interval = 2.0  # seconds
        self.timeout = 5.0  # seconds for HTTP requests
        
        # Performance tracking
        self.messages_sent = 0
        self.messages_failed = 0
        self.tasks_completed = 0
        self.last_communication = None
        
    def register_with_server(self) -> bool:
        """Register this robot with the central task assigner service."""
        try:
            payload = {
                "x": self.robot.location[0],
                "y": self.robot.location[1],
                "type": self.robot.robot_type,
                "max_battery": self.robot.max_battery,
                "speed": self.robot.speed
            }
            
            response = requests.post(
                f"{self.server_url}/api/robots",
                json=payload,
                timeout=self.timeout
            )
            
            if response.status_code == 201:
                data = response.json()
                self.robot_id = data["robot_id"]
                print(f"Robot {self.robot_id[:8]} registered successfully at {self.robot.location}")
                return True
            else:
                print(f"Registration failed: {response.text}")
                return False
                
        except requests.RequestException as e:
            print(f"Registration error: {e}")
            return False
            
    def start_autonomous_operation(self):
        """Start autonomous robot operation in a separate thread."""
        if self.robot_id is None:
            if not self.register_with_server():
                print("Cannot start - registration failed")
                return False
                
        self.is_running = True
        self.worker_thread = threading.Thread(target=self._autonomous_loop, daemon=True)
        self.worker_thread.start()
        print(f"Robot {self.robot_id[:8]} started autonomous operation")
        return True
        
    def stop_autonomous_operation(self):
        """Stop autonomous robot operation."""
        self.is_running = False
        if self.worker_thread:
            self.worker_thread.join(timeout=5.0)
        print(f"Robot {self.robot_id[:8]} stopped autonomous operation")
        
    def _autonomous_loop(self):
        """Main autonomous operation loop."""
        while self.is_running:
            try:
                # Check for new task assignment
                self._check_for_task_assignment()
                
                # Execute current task if any
                if self.robot.current_task:
                    self._execute_current_task()
                elif self.robot.needs_charging():
                    self._return_to_charging_station()
                    
                # Send status update to server
                self._send_status_update()
                
                # Rest between operations
                time.sleep(self.update_interval)
                
            except Exception as e:
                print(f"Robot {self.robot_id[:8]} error in autonomous loop: {e}")
                time.sleep(self.update_interval)
                
    def _check_for_task_assignment(self):
        """Check if robot has been assigned a new task."""
        try:
            response = requests.get(
                f"{self.server_url}/api/robots/{self.robot_id}",
                timeout=self.timeout
            )
            
            if response.status_code == 200:
                data = response.json()
                
                # If we have a current task ID but no local task object, fetch task details
                current_task_id = data.get("current_task_id")
                if current_task_id and not self.robot.current_task:
                    self._fetch_and_assign_task(current_task_id)
                    
        except requests.RequestException as e:
            print(f"Robot {self.robot_id[:8]} error checking assignment: {e}")
            self.messages_failed += 1
            
    def _fetch_and_assign_task(self, task_id: str):
        """Fetch task details and assign to robot."""
        try:
            response = requests.get(
                f"{self.server_url}/api/tasks/{task_id}",
                timeout=self.timeout
            )
            
            if response.status_code == 200:
                task_data = response.json()
                
                # Create task object
                task = Task(
                    pickup_location=tuple(task_data["pickup_location"]),
                    dropoff_location=tuple(task_data["dropoff_location"]),
                    priority=task_data["priority"]
                )
                task.task_id = task_data["task_id"]
                task.status = task_data["status"]
                
                # Assign to robot
                self.robot.current_task = task
                self.robot.status = "en_route"
                
                # Get optimal path from server
                self._fetch_optimal_path()
                
                print(f"Robot {self.robot_id[:8]} assigned task {task_id[:8]}: "
                      f"{task.pickup_location} -> {task.dropoff_location}")
                
        except requests.RequestException as e:
            print(f"Robot {self.robot_id[:8]} error fetching task: {e}")
            self.messages_failed += 1
            
    def _fetch_optimal_path(self):
        """Fetch optimal path from the server."""
        try:
            response = requests.get(
                f"{self.server_url}/api/robots/{self.robot_id}/path",
                timeout=self.timeout
            )
            
            if response.status_code == 200:
                data = response.json()
                self.current_path = data["path"]
                self.path_index = 0
                print(f"Robot {self.robot_id[:8]} received path with {len(self.current_path)} waypoints")
                
        except requests.RequestException as e:
            print(f"Robot {self.robot_id[:8]} error fetching path: {e}")
            self.messages_failed += 1
            
    def _execute_current_task(self):
        """Execute the current assigned task."""
        if not self.robot.current_task:
            return
            
        task = self.robot.current_task
        
        # Follow the path if we have one
        if self.current_path and self.path_index < len(self.current_path):
            target = self.current_path[self.path_index]
            
            if self.robot.move_to(target):
                self.path_index += 1
                
                # Check if we've reached pickup location
                if target == task.pickup_location and task.status != "in_progress":
                    task.start_task()
                    self.robot.status = "delivering"
                    print(f"Robot {self.robot_id[:8]} picked up package at {target}")
                    
                # Check if we've reached dropoff location
                elif target == task.dropoff_location and task.status == "in_progress":
                    self._complete_current_task()
                    
        else:
            # Fallback: simple movement towards target
            if task.status == "assigned":
                target = task.pickup_location
            elif task.status == "in_progress":
                target = task.dropoff_location
            else:
                return
                
            if self.robot.move_to(target):
                if target == task.pickup_location:
                    task.start_task()
                    self.robot.status = "delivering"
                elif target == task.dropoff_location:
                    self._complete_current_task()
                    
    def _complete_current_task(self):
        """Complete the current task and notify server."""
        if not self.robot.current_task:
            return
            
        task = self.robot.current_task
        task.complete_task()
        
        try:
            # Notify server of task completion
            response = requests.put(
                f"{self.server_url}/api/tasks/{task.task_id}/complete",
                timeout=self.timeout
            )
            
            if response.status_code == 200:
                print(f"Robot {self.robot_id[:8]} completed task {task.task_id[:8]}")
                self.tasks_completed += 1
            else:
                print(f"Robot {self.robot_id[:8]} failed to report task completion: {response.text}")
                
        except requests.RequestException as e:
            print(f"Robot {self.robot_id[:8]} error reporting completion: {e}")
            self.messages_failed += 1
            
        # Reset robot state
        self.robot.complete_current_task()
        self.current_path = []
        self.path_index = 0
        
    def _return_to_charging_station(self):
        """Return to charging station when battery is low."""
        if self.robot.location != self.robot.charging_station:
            self.robot.status = "charging"
            target_reached = self.robot.move_to(self.robot.charging_station)
            
            if target_reached:
                print(f"Robot {self.robot_id[:8]} reached charging station")
        else:
            # Charge the robot
            self.robot.charge(amount=15)  # Fast charging
            if self.robot.current_battery >= self.robot.max_battery * 0.8:
                print(f"Robot {self.robot_id[:8]} finished charging ({self.robot.current_battery}%)")
                
    def _send_status_update(self):
        """Send current status to the server."""
        try:
            payload = {
                "x": self.robot.location[0],
                "y": self.robot.location[1],
                "battery": self.robot.current_battery,
                "status": self.robot.status
            }
            
            response = requests.put(
                f"{self.server_url}/api/robots/{self.robot_id}/location",
                json=payload,
                timeout=self.timeout
            )
            
            if response.status_code == 200:
                self.messages_sent += 1
                self.last_communication = datetime.now()
            else:
                self.messages_failed += 1
                
        except requests.RequestException as e:
            self.messages_failed += 1
            # Don't print every communication error to avoid spam
            
    def get_performance_report(self) -> Dict[str, Any]:
        """Get performance statistics for this robot client."""
        total_messages = self.messages_sent + self.messages_failed
        success_rate = (self.messages_sent / max(total_messages, 1)) * 100
        
        return {
            "robot_id": self.robot_id,
            "robot_type": self.robot.robot_type,
            "current_location": self.robot.location,
            "battery_level": self.robot.current_battery,
            "status": self.robot.status,
            "tasks_completed": self.tasks_completed,
            "communication": {
                "messages_sent": self.messages_sent,
                "messages_failed": self.messages_failed,
                "success_rate": round(success_rate, 2),
                "last_communication": self.last_communication.isoformat() if self.last_communication else None
            },
            "current_task": {
                "task_id": self.robot.current_task.task_id if self.robot.current_task else None,
                "pickup": self.robot.current_task.pickup_location if self.robot.current_task else None,
                "dropoff": self.robot.current_task.dropoff_location if self.robot.current_task else None,
                "status": self.robot.current_task.status if self.robot.current_task else None
            } if self.robot.current_task else None,
            "path_progress": f"{self.path_index}/{len(self.current_path)}" if self.current_path else "No path"
        }
        
    def simulate_failure(self, failure_type: str):
        """Simulate different types of robot failures for testing."""
        if failure_type == "battery_drain":
            self.robot.current_battery = 0
            self.robot.status = "failed"
        elif failure_type == "communication_loss":
            self.timeout = 0.001  # Very short timeout to simulate network issues
        elif failure_type == "position_error":
            # Simulate GPS/localization error
            x, y = self.robot.location
            self.robot.location = (x + 5, y + 5)  # Offset position
        elif failure_type == "task_abandonment":
            if self.robot.current_task:
                self.robot.current_task.fail_task()
                self.robot.current_task = None
                self.robot.status = "idle"
        
        print(f"Robot {self.robot_id[:8]} simulated failure: {failure_type}")

class RobotFleetManager:
    """
    Manages multiple robot clients for testing distributed system scalability.
    """
    
    def __init__(self, server_url: str):
        self.server_url = server_url
        self.robots: List[RobotClient] = []
        
    def create_robot_fleet(self, count: int, grid_size: Tuple[int, int] = (50, 50)) -> List[RobotClient]:
        """Create a fleet of robots at random locations."""
        import random
        
        for i in range(count):
            location = (random.randint(0, grid_size[0]-1), random.randint(0, grid_size[1]-1))
            robot_type = random.choice(["drone", "ground_robot"])
            max_battery = random.randint(80, 120)
            speed = random.uniform(0.8, 1.5)
            
            robot_client = RobotClient(
                self.server_url, location, robot_type, max_battery, speed
            )
            
            self.robots.append(robot_client)
            
        return self.robots
        
    def start_all_robots(self):
        """Start autonomous operation for all robots."""
        for robot in self.robots:
            robot.start_autonomous_operation()
            time.sleep(0.1)  # Stagger startup to avoid overwhelming server
            
    def stop_all_robots(self):
        """Stop all robots."""
        for robot in self.robots:
            robot.stop_autonomous_operation()
            
    def get_fleet_statistics(self) -> Dict[str, Any]:
        """Get comprehensive fleet performance statistics."""
        if not self.robots:
            return {}
            
        total_tasks = sum(robot.tasks_completed for robot in self.robots)
        total_messages = sum(robot.messages_sent for robot in self.robots)
        total_failures = sum(robot.messages_failed for robot in self.robots)
        
        battery_levels = [robot.robot.current_battery for robot in self.robots]
        avg_battery = sum(battery_levels) / len(battery_levels)
        
        status_counts = {}
        for robot in self.robots:
            status = robot.robot.status
            status_counts[status] = status_counts.get(status, 0) + 1
            
        return {
            "fleet_size": len(self.robots),
            "total_tasks_completed": total_tasks,
            "average_battery_level": round(avg_battery, 1),
            "status_distribution": status_counts,
            "communication": {
                "total_messages": total_messages,
                "total_failures": total_failures,
                "success_rate": round((total_messages / max(total_messages + total_failures, 1)) * 100, 2)
            }
        }