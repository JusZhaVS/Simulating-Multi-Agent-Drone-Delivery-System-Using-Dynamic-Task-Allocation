import threading
import time
import queue
import random
from typing import List, Dict, Any, Optional
from datetime import datetime, timedelta
from concurrent.futures import ThreadPoolExecutor, as_completed
import json

from ..models.robot import Robot
from ..models.task import Task
from ..models.city_grid import CityGrid
from ..algorithms.optimization import HybridOptimizer
from ..algorithms.pathfinding import AStarPathfinder
from ..database.database_manager import DatabaseManager

class SimulationManager:
    """
    Multi-threaded simulation manager for SwarmDash system.
    
    Demonstrates:
    - Operating systems concepts (concurrency, threading, synchronization)
    - Resource management (shared resources, locks, queues)
    - Multi-threaded robot operations
    - Producer-consumer patterns for task generation
    - Thread pools for scalability
    """
    
    def __init__(self, grid_width: int = 50, grid_height: int = 50, 
                 max_robots: int = 20, max_worker_threads: int = 10):
        # Core simulation components
        self.city_grid = CityGrid(grid_width, grid_height)
        self.optimizer = HybridOptimizer(self.city_grid)
        self.pathfinder = AStarPathfinder(self.city_grid)
        self.db_manager = DatabaseManager()
        
        # Thread management
        self.max_worker_threads = max_worker_threads
        self.thread_pool = ThreadPoolExecutor(max_workers=max_worker_threads)
        self.robot_threads: Dict[str, threading.Thread] = {}
        self.simulation_lock = threading.RLock()  # Reentrant lock for nested operations
        
        # Simulation state
        self.is_running = False
        self.robots: Dict[str, Robot] = {}
        self.tasks: Dict[str, Task] = {}
        self.completed_tasks: List[Task] = []
        
        # Task generation
        self.task_queue = queue.Queue(maxsize=100)  # Bounded queue for task management
        self.task_generator_thread = None
        self.task_assignment_thread = None
        
        # Performance monitoring
        self.simulation_start_time = None
        self.performance_metrics = {
            "tasks_generated": 0,
            "tasks_completed": 0,
            "tasks_failed": 0,
            "robot_operations": 0,
            "optimization_runs": 0,
            "thread_pool_tasks": 0
        }
        
        # Synchronization primitives
        self.robot_assignment_lock = threading.Lock()
        self.task_completion_event = threading.Event()
        self.shutdown_event = threading.Event()
        
        # Resource management
        self.charging_stations_semaphore = threading.Semaphore(len(self.city_grid.charging_stations))
        
    def add_robot(self, location: tuple, robot_type: str = "drone", 
                  max_battery: int = 100, speed: float = 1.0) -> str:
        """
        Add a robot to the simulation (thread-safe).
        Each robot will operate in its own thread.
        """
        with self.simulation_lock:
            robot = Robot(location, max_battery, speed, robot_type)
            self.robots[robot.robot_id] = robot
            self.city_grid.add_robot(robot)
            
            # Log robot creation
            self.db_manager.insert_robot({
                'robot_id': robot.robot_id,
                'robot_type': robot.robot_type,
                'max_battery': robot.max_battery,
                'speed': robot.speed,
                'charging_station': robot.charging_station
            })
            
            # Start robot thread if simulation is running
            if self.is_running:
                self._start_robot_thread(robot)
                
            return robot.robot_id
            
    def _start_robot_thread(self, robot: Robot):
        """Start a dedicated thread for robot operations."""
        thread = threading.Thread(
            target=self._robot_operation_loop,
            args=(robot,),
            name=f"Robot-{robot.robot_id[:8]}",
            daemon=True
        )
        thread.start()
        self.robot_threads[robot.robot_id] = thread
        
    def _robot_operation_loop(self, robot: Robot):
        """
        Main operation loop for an individual robot (runs in separate thread).
        Demonstrates concurrent robot operations with proper synchronization.
        """
        print(f"Robot {robot.robot_id[:8]} started operation thread")
        
        while self.is_running and not self.shutdown_event.is_set():
            try:
                with robot.lock:  # Robot-specific lock for thread safety
                    # Check if robot needs charging
                    if robot.needs_charging() and robot.status != "charging":
                        self._handle_robot_charging(robot)
                        continue
                        
                    # Execute current task if any
                    if robot.current_task and robot.status in ["en_route", "delivering"]:
                        self._execute_robot_task(robot)
                        
                    # Update performance metrics
                    with self.simulation_lock:
                        self.performance_metrics["robot_operations"] += 1
                        
                    # Log robot movement
                    self.db_manager.log_robot_movement(
                        robot.robot_id, robot.location, robot.current_battery,
                        robot.status, robot.current_task.task_id if robot.current_task else None
                    )
                    
                # Sleep between operations (release locks during sleep)
                time.sleep(0.5 + random.uniform(0, 0.5))  # Staggered timing
                
            except Exception as e:
                print(f"Robot {robot.robot_id[:8]} error in operation loop: {e}")
                time.sleep(1.0)
                
        print(f"Robot {robot.robot_id[:8]} stopped operation thread")
        
    def _handle_robot_charging(self, robot: Robot):
        """
        Handle robot charging with semaphore-based resource management.
        Demonstrates resource sharing and synchronization.
        """
        if robot.location != robot.charging_station:
            # Move towards charging station
            target_reached = robot.move_to(robot.charging_station)
            robot.status = "charging"
            
            if target_reached:
                print(f"Robot {robot.robot_id[:8]} reached charging station")
        else:
            # Acquire charging station (limited resource)
            try:
                if self.charging_stations_semaphore.acquire(blocking=False):
                    try:
                        # Charge the robot
                        robot.charge(amount=20)
                        if robot.current_battery >= robot.max_battery * 0.8:
                            robot.status = "idle"
                            print(f"Robot {robot.robot_id[:8]} finished charging")
                    finally:
                        self.charging_stations_semaphore.release()
                else:
                    # Charging station busy, wait
                    time.sleep(1.0)
            except Exception as e:
                print(f"Robot {robot.robot_id[:8]} charging error: {e}")
                
    def _execute_robot_task(self, robot: Robot):
        """Execute robot task with pathfinding and task progression."""
        task = robot.current_task
        
        # Determine next target based on task status
        if task.status == "assigned":
            target = task.pickup_location
            next_status = "in_progress"
            status_message = "picked up package"
        elif task.status == "in_progress":
            target = task.dropoff_location
            next_status = "completed"
            status_message = "delivered package"
        else:
            return
            
        # Move towards target
        if robot.move_to(target):
            task.status = next_status
            if next_status == "completed":
                self._complete_task(robot, task)
            else:
                task.start_task()
                robot.status = "delivering"
            print(f"Robot {robot.robot_id[:8]} {status_message} at {target}")
            
    def _complete_task(self, robot: Robot, task: Task):
        """Complete a task and update all relevant data structures."""
        with self.simulation_lock:
            task.complete_task()
            robot.complete_current_task()
            
            # Move task to completed list
            if task.task_id in self.tasks:
                del self.tasks[task.task_id]
            self.completed_tasks.append(task)
            self.city_grid.complete_task(task.task_id)
            
            # Update performance metrics
            self.performance_metrics["tasks_completed"] += 1
            
            # Update database
            self.db_manager.update_task_status(task.task_id, 'completed')
            
            # Signal that a task was completed (for optimization triggering)
            self.task_completion_event.set()
            
    def start_task_generator(self, generation_rate: float = 2.0):
        """
        Start task generation thread (producer in producer-consumer pattern).
        """
        def task_generator():
            print("Task generator thread started")
            
            while self.is_running and not self.shutdown_event.is_set():
                try:
                    # Generate random task
                    task = self.city_grid.generate_random_task()
                    
                    # Add to queue (blocks if queue is full)
                    self.task_queue.put(task, timeout=1.0)
                    
                    with self.simulation_lock:
                        self.performance_metrics["tasks_generated"] += 1
                        
                    print(f"Generated task {task.task_id[:8]}: {task.pickup_location} -> {task.dropoff_location}")
                    
                    # Wait before generating next task
                    time.sleep(1.0 / generation_rate)
                    
                except queue.Full:
                    print("Task queue full, skipping task generation")
                    time.sleep(1.0)
                except Exception as e:
                    print(f"Task generator error: {e}")
                    time.sleep(1.0)
                    
            print("Task generator thread stopped")
            
        self.task_generator_thread = threading.Thread(
            target=task_generator,
            name="TaskGenerator",
            daemon=True
        )
        self.task_generator_thread.start()
        
    def start_task_assignment(self, assignment_interval: float = 3.0):
        """
        Start task assignment thread (consumer in producer-consumer pattern).
        Uses thread pool for optimization operations.
        """
        def task_assigner():
            print("Task assignment thread started")
            
            while self.is_running and not self.shutdown_event.is_set():
                try:
                    # Collect pending tasks from queue
                    pending_tasks = []
                    
                    # Non-blocking collection of tasks
                    while len(pending_tasks) < 10:  # Batch size limit
                        try:
                            task = self.task_queue.get(timeout=0.1)
                            pending_tasks.append(task)
                            
                            # Add to simulation state
                            with self.simulation_lock:
                                self.tasks[task.task_id] = task
                                self.city_grid.add_task(task)
                                
                            # Log to database
                            self.db_manager.insert_task({
                                'task_id': task.task_id,
                                'pickup_location': task.pickup_location,
                                'dropoff_location': task.dropoff_location,
                                'priority': task.priority,
                                'status': task.status,
                                'time_created': task.time_created,
                                'estimated_distance': task.get_total_distance()
                            })
                            
                        except queue.Empty:
                            break
                            
                    # Run optimization if we have tasks
                    if pending_tasks:
                        self._submit_optimization_task(pending_tasks)
                        
                    # Wait or check for task completion events
                    self.task_completion_event.wait(timeout=assignment_interval)
                    self.task_completion_event.clear()
                    
                except Exception as e:
                    print(f"Task assignment error: {e}")
                    time.sleep(1.0)
                    
            print("Task assignment thread stopped")
            
        self.task_assignment_thread = threading.Thread(
            target=task_assigner,
            name="TaskAssigner",
            daemon=True
        )
        self.task_assignment_thread.start()
        
    def _submit_optimization_task(self, pending_tasks: List[Task]):
        """
        Submit optimization task to thread pool for parallel processing.
        """
        def run_optimization():
            with self.robot_assignment_lock:  # Prevent concurrent assignments
                # Get available robots
                available_robots = []
                with self.simulation_lock:
                    available_robots = [robot for robot in self.robots.values() 
                                      if robot.status == "idle" and not robot.needs_charging()]
                
                if not available_robots:
                    return {"assignments": {}, "message": "No available robots"}
                    
                # Run optimization
                result = self.optimizer.optimize_allocation(available_robots, pending_tasks)
                
                # Apply assignments
                assignments_made = 0
                for robot_id, task_id in result["assignments"].items():
                    with self.simulation_lock:
                        if robot_id in self.robots and task_id in self.tasks:
                            robot = self.robots[robot_id]
                            task = self.tasks[task_id]
                            
                            if robot.assign_task(task):
                                assignments_made += 1
                                # Update database
                                self.db_manager.update_task_assignment(task_id, robot_id)
                                
                print(f"Optimization completed: {assignments_made} assignments made")
                
                # Update performance metrics
                with self.simulation_lock:
                    self.performance_metrics["optimization_runs"] += 1
                    self.performance_metrics["thread_pool_tasks"] += 1
                    
                # Log optimization performance
                result["assignments_made"] = assignments_made
                self.db_manager.log_optimization_run(result)
                
                return result
                
        # Submit to thread pool
        future = self.thread_pool.submit(run_optimization)
        
    def start_simulation(self, num_robots: int = 10, task_generation_rate: float = 2.0):
        """
        Start the complete multi-threaded simulation.
        """
        print("Starting SwarmDash multi-threaded simulation...")
        self.simulation_start_time = datetime.now()
        self.is_running = True
        self.shutdown_event.clear()
        
        # Create initial robot fleet
        for i in range(num_robots):
            location = self.city_grid.get_random_valid_location()
            robot_type = random.choice(["drone", "ground_robot"])
            max_battery = random.randint(80, 120)
            speed = random.uniform(0.8, 1.5)
            
            robot_id = self.add_robot(location, robot_type, max_battery, speed)
            print(f"Created robot {robot_id[:8]} at {location}")
            
        # Start all robot threads
        for robot in self.robots.values():
            self._start_robot_thread(robot)
            
        # Start task generation and assignment
        self.start_task_generator(task_generation_rate)
        self.start_task_assignment()
        
        # Start performance monitoring thread
        self._start_performance_monitor()
        
        print(f"Simulation started with {num_robots} robots and {self.max_worker_threads} worker threads")
        
    def _start_performance_monitor(self):
        """Start performance monitoring thread."""
        def monitor_performance():
            while self.is_running and not self.shutdown_event.is_set():
                try:
                    # Log system metrics every 30 seconds
                    with self.simulation_lock:
                        system_stats = self.city_grid.get_system_statistics()
                    self.db_manager.log_system_metrics(system_stats)
                    
                    time.sleep(30.0)
                except Exception as e:
                    print(f"Performance monitor error: {e}")
                    time.sleep(30.0)
                    
        monitor_thread = threading.Thread(
            target=monitor_performance,
            name="PerformanceMonitor",
            daemon=True
        )
        monitor_thread.start()
        
    def stop_simulation(self):
        """
        Gracefully stop the simulation and all threads.
        """
        print("Stopping simulation...")
        
        # Signal shutdown
        self.is_running = False
        self.shutdown_event.set()
        
        # Stop task generation
        if self.task_generator_thread:
            self.task_generator_thread.join(timeout=5.0)
            
        # Stop task assignment
        if self.task_assignment_thread:
            self.task_assignment_thread.join(timeout=5.0)
            
        # Stop robot threads
        for robot_id, thread in self.robot_threads.items():
            thread.join(timeout=2.0)
            
        # Shutdown thread pool
        self.thread_pool.shutdown(wait=True, timeout=10.0)
        
        print("Simulation stopped")
        
    def get_simulation_statistics(self) -> Dict[str, Any]:
        """Get comprehensive simulation statistics."""
        uptime = datetime.now() - self.simulation_start_time if self.simulation_start_time else timedelta(0)
        
        with self.simulation_lock:
            active_threads = sum(1 for thread in self.robot_threads.values() if thread.is_alive())
            
            return {
                "simulation_uptime": str(uptime),
                "robots": {
                    "total": len(self.robots),
                    "active_threads": active_threads,
                    "status_distribution": self._get_robot_status_distribution()
                },
                "tasks": {
                    "pending": len(self.tasks),
                    "completed": len(self.completed_tasks),
                    "queue_size": self.task_queue.qsize(),
                    "queue_max_size": self.task_queue.maxsize
                },
                "performance": self.performance_metrics.copy(),
                "threading": {
                    "worker_threads": self.max_worker_threads,
                    "active_robot_threads": active_threads,
                    "thread_pool_active": not self.thread_pool._shutdown
                },
                "system_statistics": self.city_grid.get_system_statistics()
            }
            
    def _get_robot_status_distribution(self) -> Dict[str, int]:
        """Get distribution of robot statuses."""
        status_counts = {}
        for robot in self.robots.values():
            status = robot.status
            status_counts[status] = status_counts.get(status, 0) + 1
        return status_counts