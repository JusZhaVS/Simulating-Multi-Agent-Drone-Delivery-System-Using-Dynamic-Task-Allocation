from typing import List, Tuple, Set, Dict, Optional
import random
import threading
from .robot import Robot
from .task import Task

class CityGrid:
    """
    Represents the urban environment for robot navigation.
    Manages obstacles, charging stations, and provides pathfinding support.
    """
    
    def __init__(self, width: int, height: int, obstacle_density: float = 0.1):
        self.width = width
        self.height = height
        self.obstacle_density = obstacle_density
        self.obstacles: Set[Tuple[int, int]] = set()
        self.charging_stations: List[Tuple[int, int]] = []
        self.robots: Dict[str, Robot] = {}
        self.active_tasks: Dict[str, Task] = {}
        self.lock = threading.Lock()
        
        self._generate_obstacles()
        self._place_charging_stations()
        
    def _generate_obstacles(self):
        """Generate random obstacles in the grid."""
        total_cells = self.width * self.height
        num_obstacles = int(total_cells * self.obstacle_density)
        
        for _ in range(num_obstacles):
            x = random.randint(0, self.width - 1)
            y = random.randint(0, self.height - 1)
            self.obstacles.add((x, y))
            
    def _place_charging_stations(self):
        """Place charging stations strategically across the grid."""
        # Place stations in corners and center for good coverage
        stations = [
            (0, 0),
            (self.width - 1, 0),
            (0, self.height - 1),
            (self.width - 1, self.height - 1),
            (self.width // 2, self.height // 2)
        ]
        
        for station in stations:
            if station not in self.obstacles:
                self.charging_stations.append(station)
                
    def is_valid_location(self, location: Tuple[int, int]) -> bool:
        """Check if a location is valid (within bounds and not an obstacle)."""
        x, y = location
        return (0 <= x < self.width and 
                0 <= y < self.height and 
                location not in self.obstacles)
                
    def get_neighbors(self, location: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighboring cells for pathfinding."""
        x, y = location
        neighbors = [
            (x + 1, y), (x - 1, y),
            (x, y + 1), (x, y - 1)
        ]
        return [loc for loc in neighbors if self.is_valid_location(loc)]
        
    def add_robot(self, robot: Robot):
        """Add a robot to the grid."""
        with self.lock:
            self.robots[robot.robot_id] = robot
            
    def remove_robot(self, robot_id: str):
        """Remove a robot from the grid."""
        with self.lock:
            if robot_id in self.robots:
                del self.robots[robot_id]
                
    def add_task(self, task: Task):
        """Add a new delivery task."""
        with self.lock:
            self.active_tasks[task.task_id] = task
            
    def complete_task(self, task_id: str):
        """Remove a completed task."""
        with self.lock:
            if task_id in self.active_tasks:
                del self.active_tasks[task_id]
                
    def get_nearest_charging_station(self, location: Tuple[int, int]) -> Tuple[int, int]:
        """Find the closest charging station to a given location."""
        if not self.charging_stations:
            return (0, 0)  # Fallback
            
        min_distance = float('inf')
        nearest_station = self.charging_stations[0]
        
        for station in self.charging_stations:
            distance = abs(location[0] - station[0]) + abs(location[1] - station[1])
            if distance < min_distance:
                min_distance = distance
                nearest_station = station
                
        return nearest_station
        
    def get_random_valid_location(self) -> Tuple[int, int]:
        """Generate a random valid location on the grid."""
        while True:
            x = random.randint(0, self.width - 1)
            y = random.randint(0, self.height - 1)
            location = (x, y)
            if self.is_valid_location(location):
                return location
                
    def generate_random_task(self) -> Task:
        """Generate a random delivery task with valid pickup/dropoff locations."""
        pickup = self.get_random_valid_location()
        dropoff = self.get_random_valid_location()
        
        # Ensure pickup and dropoff are different
        while dropoff == pickup:
            dropoff = self.get_random_valid_location()
            
        priority = random.randint(1, 5)  # Random priority 1-5
        return Task(pickup, dropoff, priority)
        
    def get_grid_state(self) -> Dict:
        """Get current state of the entire grid for visualization."""
        with self.lock:
            return {
                "dimensions": (self.width, self.height),
                "obstacles": list(self.obstacles),
                "charging_stations": self.charging_stations,
                "robots": {robot_id: robot.get_status_report() 
                          for robot_id, robot in self.robots.items()},
                "active_tasks": {task_id: {
                    "pickup": task.pickup_location,
                    "dropoff": task.dropoff_location,
                    "priority": task.priority,
                    "status": task.status
                } for task_id, task in self.active_tasks.items()}
            }
            
    def calculate_manhattan_distance(self, loc1: Tuple[int, int], loc2: Tuple[int, int]) -> int:
        """Calculate Manhattan distance between two locations."""
        return abs(loc1[0] - loc2[0]) + abs(loc1[1] - loc2[1])
        
    def get_system_statistics(self) -> Dict:
        """Get comprehensive system performance statistics."""
        with self.lock:
            total_robots = len(self.robots)
            active_robots = sum(1 for robot in self.robots.values() if robot.status != "idle")
            total_tasks = len(self.active_tasks)
            pending_tasks = sum(1 for task in self.active_tasks.values() if task.status == "pending")
            
            if total_robots > 0:
                avg_battery = sum(robot.current_battery for robot in self.robots.values()) / total_robots
                total_completed = sum(robot.tasks_completed for robot in self.robots.values())
                avg_efficiency = sum(robot.get_efficiency_score() for robot in self.robots.values()) / total_robots
            else:
                avg_battery = 0
                total_completed = 0
                avg_efficiency = 0
                
            return {
                "total_robots": total_robots,
                "active_robots": active_robots,
                "total_tasks": total_tasks,
                "pending_tasks": pending_tasks,
                "completed_tasks": total_completed,
                "average_battery": round(avg_battery, 1),
                "system_efficiency": round(avg_efficiency, 3),
                "grid_utilization": round((total_robots + total_tasks) / (self.width * self.height), 3)
            }