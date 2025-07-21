from typing import Tuple, List, Optional
import uuid
import time
import threading
from datetime import datetime
from .task import Task

class Robot:
    """
    Represents an autonomous delivery robot/drone.
    Demonstrates object-oriented design with encapsulation and state management.
    """
    
    def __init__(self, start_location: Tuple[int, int], max_battery: int = 100, 
                 speed: float = 1.0, robot_type: str = "drone"):
        self.robot_id = str(uuid.uuid4())
        self.location = start_location
        self.max_battery = max_battery
        self.current_battery = max_battery
        self.speed = speed  # units per second
        self.robot_type = robot_type
        self.status = "idle"  # idle, en_route, charging, delivering, failed
        self.current_task: Optional[Task] = None
        self.path: List[Tuple[int, int]] = []
        self.lock = threading.Lock()  # For thread safety
        self.charging_station = start_location  # Where robot returns to charge
        self.tasks_completed = 0
        self.total_distance_traveled = 0.0
        self.creation_time = datetime.now()
        
    def assign_task(self, task: Task) -> bool:
        """
        Assign a task to this robot if it has sufficient battery.
        Returns True if task was assigned successfully.
        """
        with self.lock:
            if self.status != "idle":
                return False
                
            estimated_distance = self._calculate_task_distance(task)
            required_battery = estimated_distance * 2  # Round trip + safety margin
            
            if self.current_battery < required_battery:
                return False
                
            self.current_task = task
            task.assign_to_robot(self.robot_id)
            self.status = "en_route"
            return True
            
    def _calculate_task_distance(self, task: Task) -> float:
        """Calculate total distance for a task including return to base."""
        # Distance from current location to pickup
        pickup_dist = abs(self.location[0] - task.pickup_location[0]) + \
                     abs(self.location[1] - task.pickup_location[1])
        
        # Distance from pickup to dropoff
        delivery_dist = task.get_total_distance()
        
        # Distance from dropoff back to charging station
        return_dist = abs(task.dropoff_location[0] - self.charging_station[0]) + \
                     abs(task.dropoff_location[1] - self.charging_station[1])
        
        return pickup_dist + delivery_dist + return_dist
        
    def move_to(self, target: Tuple[int, int]) -> bool:
        """
        Move robot one step towards target location.
        Returns True if robot reached target.
        """
        with self.lock:
            if self.current_battery <= 0:
                self.status = "failed"
                return False
                
            current_x, current_y = self.location
            target_x, target_y = target
            
            # Simple movement: one step at a time
            if current_x < target_x:
                current_x += 1
            elif current_x > target_x:
                current_x -= 1
            elif current_y < target_y:
                current_y += 1
            elif current_y > target_y:
                current_y -= 1
                
            self.location = (current_x, current_y)
            self.current_battery -= 1  # Battery consumption per move
            self.total_distance_traveled += 1
            
            return self.location == target
            
    def charge(self, amount: int = 10):
        """Charge the robot's battery."""
        with self.lock:
            self.current_battery = min(self.max_battery, self.current_battery + amount)
            if self.current_battery >= self.max_battery * 0.8:  # 80% charge threshold
                self.status = "idle"
                
    def needs_charging(self) -> bool:
        """Check if robot needs to return to charging station."""
        return self.current_battery < self.max_battery * 0.3  # 30% threshold
        
    def complete_current_task(self):
        """Mark current task as completed and reset robot state."""
        with self.lock:
            if self.current_task:
                self.current_task.complete_task()
                self.tasks_completed += 1
                self.current_task = None
            self.status = "idle"
            
    def get_efficiency_score(self) -> float:
        """Calculate robot efficiency based on tasks completed vs energy used."""
        if self.total_distance_traveled == 0:
            return 0.0
        return self.tasks_completed / self.total_distance_traveled
        
    def get_status_report(self) -> dict:
        """Return comprehensive status report for this robot."""
        with self.lock:
            return {
                "robot_id": self.robot_id,
                "location": self.location,
                "battery": f"{self.current_battery}/{self.max_battery}",
                "status": self.status,
                "current_task_id": self.current_task.task_id if self.current_task else None,
                "tasks_completed": self.tasks_completed,
                "total_distance": self.total_distance_traveled,
                "efficiency": round(self.get_efficiency_score(), 3),
                "uptime": str(datetime.now() - self.creation_time)
            }
            
    def __repr__(self):
        return f"Robot({self.robot_id[:8]}, {self.location}, battery={self.current_battery}, status={self.status})"