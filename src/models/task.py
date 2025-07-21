from dataclasses import dataclass
from typing import Tuple
import uuid
from datetime import datetime

@dataclass
class Task:
    """
    Represents a delivery task with pickup and dropoff locations.
    Demonstrates object-oriented design principles.
    """
    
    def __init__(self, pickup_location: Tuple[int, int], dropoff_location: Tuple[int, int], 
                 priority: int = 1, time_created: datetime = None):
        self.task_id = str(uuid.uuid4())
        self.pickup_location = pickup_location
        self.dropoff_location = dropoff_location
        self.priority = priority  # Higher number = higher priority
        self.time_created = time_created or datetime.now()
        self.assigned_robot_id = None
        self.status = "pending"  # pending, assigned, in_progress, completed, failed
        self.estimated_completion_time = None
        
    def assign_to_robot(self, robot_id: str):
        """Assign this task to a specific robot."""
        self.assigned_robot_id = robot_id
        self.status = "assigned"
        
    def start_task(self):
        """Mark task as in progress."""
        self.status = "in_progress"
        
    def complete_task(self):
        """Mark task as completed."""
        self.status = "completed"
        
    def fail_task(self):
        """Mark task as failed."""
        self.status = "failed"
        
    def get_total_distance(self) -> float:
        """Calculate Manhattan distance for this task."""
        pickup_x, pickup_y = self.pickup_location
        dropoff_x, dropoff_y = self.dropoff_location
        return abs(dropoff_x - pickup_x) + abs(dropoff_y - pickup_y)
        
    def __lt__(self, other):
        """Enable priority queue ordering by priority and creation time."""
        if self.priority != other.priority:
            return self.priority > other.priority  # Higher priority first
        return self.time_created < other.time_created  # Earlier tasks first for same priority
        
    def __repr__(self):
        return f"Task({self.task_id[:8]}, {self.pickup_location}->{self.dropoff_location}, priority={self.priority}, status={self.status})"