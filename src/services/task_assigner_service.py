from flask import Flask, request, jsonify
import threading
import time
from typing import Dict, List, Any
from datetime import datetime, timedelta
import json

from ..models.robot import Robot
from ..models.task import Task
from ..models.city_grid import CityGrid
from ..algorithms.optimization import HybridOptimizer
from ..algorithms.pathfinding import AStarPathfinder
from ..database.database_manager import DatabaseManager

class TaskAssignerService:
    """
    Central Task Assigner Service for the SwarmDash distributed system.
    
    This demonstrates:
    - Distributed system architecture (REST API)
    - Multi-tiered system design (presentation, logic, data tiers)
    - Concurrent request handling
    - Real-time task allocation and robot management
    """
    
    def __init__(self, grid_width: int = 50, grid_height: int = 50, port: int = 5000):
        self.app = Flask(__name__)
        self.port = port
        
        # Core components
        self.city_grid = CityGrid(grid_width, grid_height)
        self.optimizer = HybridOptimizer(self.city_grid)
        self.pathfinder = AStarPathfinder(self.city_grid)
        self.db_manager = DatabaseManager()
        
        # Service state
        self.robots: Dict[str, Robot] = {}
        self.tasks: Dict[str, Task] = {}
        self.service_lock = threading.Lock()
        self.is_running = False
        
        # Performance metrics
        self.allocation_count = 0
        self.total_allocation_time = 0.0
        self.service_start_time = datetime.now()
        
        self._setup_routes()
        
    def _setup_routes(self):
        """Set up REST API endpoints."""
        
        @self.app.route('/api/health', methods=['GET'])
        def health_check():
            """Health check endpoint."""
            return jsonify({
                "status": "healthy",
                "service": "TaskAssignerService",
                "uptime": str(datetime.now() - self.service_start_time),
                "robots_count": len(self.robots),
                "tasks_count": len(self.tasks),
                "timestamp": datetime.now().isoformat()
            })
            
        @self.app.route('/api/robots', methods=['POST'])
        def register_robot():
            """Register a new robot with the service."""
            try:
                data = request.get_json()
                
                robot = Robot(
                    start_location=(data['x'], data['y']),
                    max_battery=data.get('max_battery', 100),
                    speed=data.get('speed', 1.0),
                    robot_type=data.get('type', 'drone')
                )
                
                with self.service_lock:
                    self.robots[robot.robot_id] = robot
                    self.city_grid.add_robot(robot)
                    
                # Log to database
                self.db_manager.insert_robot({
                    'robot_id': robot.robot_id,
                    'robot_type': robot.robot_type,
                    'max_battery': robot.max_battery,
                    'speed': robot.speed,
                    'charging_station': robot.charging_station
                })
                
                return jsonify({
                    "robot_id": robot.robot_id,
                    "status": "registered",
                    "location": robot.location
                }), 201
                
            except Exception as e:
                return jsonify({"error": str(e)}), 400
                
        @self.app.route('/api/robots/<robot_id>', methods=['GET'])
        def get_robot_status(robot_id):
            """Get detailed status of a specific robot."""
            with self.service_lock:
                if robot_id not in self.robots:
                    return jsonify({"error": "Robot not found"}), 404
                    
                robot = self.robots[robot_id]
                return jsonify(robot.get_status_report())
                
        @self.app.route('/api/robots/<robot_id>/location', methods=['PUT'])
        def update_robot_location(robot_id):
            """Update robot location (called by robot clients)."""
            try:
                data = request.get_json()
                
                with self.service_lock:
                    if robot_id not in self.robots:
                        return jsonify({"error": "Robot not found"}), 404
                        
                    robot = self.robots[robot_id]
                    robot.location = (data['x'], data['y'])
                    robot.current_battery = data.get('battery', robot.current_battery)
                    robot.status = data.get('status', robot.status)
                    
                # Log movement to database
                self.db_manager.log_robot_movement(
                    robot_id, robot.location, robot.current_battery, 
                    robot.status, robot.current_task.task_id if robot.current_task else None
                )
                
                return jsonify({"status": "updated"})
                
            except Exception as e:
                return jsonify({"error": str(e)}), 400
                
        @self.app.route('/api/tasks', methods=['POST'])
        def create_task():
            """Create a new delivery task."""
            try:
                data = request.get_json()
                
                task = Task(
                    pickup_location=(data['pickup_x'], data['pickup_y']),
                    dropoff_location=(data['dropoff_x'], data['dropoff_y']),
                    priority=data.get('priority', 1)
                )
                
                with self.service_lock:
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
                
                # Trigger task allocation
                self._trigger_task_allocation()
                
                return jsonify({
                    "task_id": task.task_id,
                    "status": "created",
                    "estimated_distance": task.get_total_distance()
                }), 201
                
            except Exception as e:
                return jsonify({"error": str(e)}), 400
                
        @self.app.route('/api/tasks/<task_id>', methods=['GET'])
        def get_task_status(task_id):
            """Get status of a specific task."""
            with self.service_lock:
                if task_id not in self.tasks:
                    return jsonify({"error": "Task not found"}), 404
                    
                task = self.tasks[task_id]
                return jsonify({
                    "task_id": task.task_id,
                    "pickup_location": task.pickup_location,
                    "dropoff_location": task.dropoff_location,
                    "priority": task.priority,
                    "status": task.status,
                    "assigned_robot_id": task.assigned_robot_id,
                    "time_created": task.time_created.isoformat(),
                    "estimated_distance": task.get_total_distance()
                })
                
        @self.app.route('/api/tasks/<task_id>/complete', methods=['PUT'])
        def complete_task(task_id):
            """Mark a task as completed."""
            try:
                with self.service_lock:
                    if task_id not in self.tasks:
                        return jsonify({"error": "Task not found"}), 404
                        
                    task = self.tasks[task_id]
                    task.complete_task()
                    
                    # Update robot status
                    if task.assigned_robot_id and task.assigned_robot_id in self.robots:
                        robot = self.robots[task.assigned_robot_id]
                        robot.complete_current_task()
                        
                    # Remove from active tasks
                    del self.tasks[task_id]
                    self.city_grid.complete_task(task_id)
                    
                # Update database
                self.db_manager.update_task_status(task_id, 'completed')
                
                # Trigger new task allocation
                self._trigger_task_allocation()
                
                return jsonify({"status": "completed"})
                
            except Exception as e:
                return jsonify({"error": str(e)}), 400
                
        @self.app.route('/api/allocation/trigger', methods=['POST'])
        def trigger_allocation():
            """Manually trigger task allocation optimization."""
            result = self._trigger_task_allocation()
            return jsonify(result)
            
        @self.app.route('/api/system/status', methods=['GET'])
        def get_system_status():
            """Get comprehensive system status."""
            with self.service_lock:
                grid_stats = self.city_grid.get_system_statistics()
                
            return jsonify({
                "service_uptime": str(datetime.now() - self.service_start_time),
                "total_allocations": self.allocation_count,
                "avg_allocation_time_ms": (self.total_allocation_time / max(self.allocation_count, 1)) * 1000,
                "grid_statistics": grid_stats,
                "database_stats": self.db_manager.get_database_stats()
            })
            
        @self.app.route('/api/system/metrics', methods=['GET'])
        def get_system_metrics():
            """Get historical system metrics."""
            days = request.args.get('days', 7, type=int)
            
            return jsonify({
                "robot_performance": self.db_manager.get_robot_performance(),
                "task_completion_stats": self.db_manager.get_task_completion_stats(days),
                "system_performance": self.db_manager.get_system_performance_summary(days),
                "optimization_performance": self.db_manager.get_optimization_performance(100)
            })
            
        @self.app.route('/api/robots/<robot_id>/path', methods=['GET'])
        def get_robot_path(robot_id):
            """Get optimal path for robot to complete its current task."""
            try:
                with self.service_lock:
                    if robot_id not in self.robots:
                        return jsonify({"error": "Robot not found"}), 404
                        
                    robot = self.robots[robot_id]
                    if not robot.current_task:
                        return jsonify({"path": [], "message": "No active task"})
                        
                task = robot.current_task
                
                # Calculate path through waypoints: current -> pickup -> dropoff -> charging
                waypoints = [task.pickup_location, task.dropoff_location, robot.charging_station]
                
                start_time = time.time()
                path = self.pathfinder.find_path_with_waypoints(robot.location, waypoints)
                computation_time = (time.time() - start_time) * 1000
                
                # Log pathfinding performance
                if path:
                    self.db_manager.log_pathfinding_stats({
                        'robot_id': robot_id,
                        'task_id': task.task_id,
                        'start': robot.location,
                        'goal': robot.charging_station,
                        'algorithm_used': 'A*',
                        'path_length': len(path),
                        'computation_time_ms': computation_time,
                        'nodes_explored': len(path) * 4,  # Estimated
                        'path_optimal': True
                    })
                
                return jsonify({
                    "path": path,
                    "path_length": len(path) if path else 0,
                    "computation_time_ms": computation_time,
                    "waypoints": waypoints
                })
                
            except Exception as e:
                return jsonify({"error": str(e)}), 400
                
    def _trigger_task_allocation(self) -> Dict[str, Any]:
        """Trigger optimization-based task allocation."""
        start_time = time.time()
        
        with self.service_lock:
            # Get available robots and pending tasks
            available_robots = [robot for robot in self.robots.values() 
                              if robot.status == "idle" and not robot.needs_charging()]
            pending_tasks = [task for task in self.tasks.values() 
                           if task.status == "pending"]
            
        if not available_robots or not pending_tasks:
            return {"message": "No allocation needed", "assignments": {}}
            
        # Run optimization
        optimization_result = self.optimizer.optimize_allocation(available_robots, pending_tasks)
        
        # Apply assignments
        assignments_made = 0
        for robot_id, task_id in optimization_result["assignments"].items():
            with self.service_lock:
                if robot_id in self.robots and task_id in self.tasks:
                    robot = self.robots[robot_id]
                    task = self.tasks[task_id]
                    
                    if robot.assign_task(task):
                        assignments_made += 1
                        # Update database
                        self.db_manager.update_task_assignment(task_id, robot_id)
        
        allocation_time = time.time() - start_time
        self.allocation_count += 1
        self.total_allocation_time += allocation_time
        
        # Log optimization performance
        optimization_result["assignments_made"] = assignments_made
        self.db_manager.log_optimization_run(optimization_result)
        
        # Log system metrics
        with self.service_lock:
            system_stats = self.city_grid.get_system_statistics()
        self.db_manager.log_system_metrics(system_stats)
        
        return optimization_result
        
    def start_service(self, debug: bool = False, threaded: bool = True):
        """Start the Flask web service."""
        self.is_running = True
        print(f"Starting TaskAssignerService on port {self.port}")
        print(f"Grid size: {self.city_grid.width}x{self.city_grid.height}")
        print(f"API endpoints available at http://localhost:{self.port}/api/")
        
        self.app.run(
            host='0.0.0.0',
            port=self.port,
            debug=debug,
            threaded=threaded,
            use_reloader=False  # Prevent double initialization
        )
        
    def stop_service(self):
        """Stop the service gracefully."""
        self.is_running = False
        
    def get_api_documentation(self) -> Dict[str, Any]:
        """Return API documentation for the service."""
        return {
            "service": "SwarmDash TaskAssignerService",
            "version": "1.0",
            "endpoints": {
                "GET /api/health": "Service health check",
                "POST /api/robots": "Register new robot",
                "GET /api/robots/<id>": "Get robot status",
                "PUT /api/robots/<id>/location": "Update robot location",
                "POST /api/tasks": "Create new delivery task",
                "GET /api/tasks/<id>": "Get task status",
                "PUT /api/tasks/<id>/complete": "Mark task complete",
                "POST /api/allocation/trigger": "Trigger task allocation",
                "GET /api/system/status": "Get system status",
                "GET /api/system/metrics": "Get historical metrics",
                "GET /api/robots/<id>/path": "Get optimal path for robot"
            },
            "distributed_architecture": {
                "tier": "Logic Tier (Central Service)",
                "data_tier": "SQLite Database with analytics views",
                "presentation_tier": "REST API + Future GUI client",
                "scalability": "Horizontal scaling via load balancer"
            }
        }