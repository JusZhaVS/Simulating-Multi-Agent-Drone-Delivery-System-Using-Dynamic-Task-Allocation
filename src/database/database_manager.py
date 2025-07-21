import sqlite3
import os
import threading
from typing import List, Dict, Any, Optional, Tuple
from datetime import datetime
from contextlib import contextmanager

class DatabaseManager:
    """
    Database manager for SwarmDash system.
    Demonstrates relational database operations and SQL knowledge.
    Provides thread-safe database operations for multi-threaded robot simulation.
    """
    
    def __init__(self, db_path: str = "swarm_dash.db"):
        self.db_path = db_path
        self.lock = threading.Lock()
        self._initialize_database()
        
    def _initialize_database(self):
        """Initialize database with schema if it doesn't exist."""
        schema_path = os.path.join(os.path.dirname(__file__), "schema.sql")
        
        with self.get_connection() as conn:
            with open(schema_path, 'r') as schema_file:
                schema_sql = schema_file.read()
                conn.executescript(schema_sql)
                conn.commit()
                
    @contextmanager
    def get_connection(self):
        """Thread-safe database connection context manager."""
        with self.lock:
            conn = sqlite3.connect(self.db_path)
            conn.row_factory = sqlite3.Row  # Enable dict-like row access
            try:
                yield conn
            finally:
                conn.close()
                
    def insert_robot(self, robot_data: Dict[str, Any]) -> bool:
        """Insert a new robot into the database."""
        try:
            with self.get_connection() as conn:
                conn.execute("""
                    INSERT INTO robots (
                        robot_id, robot_type, max_battery, speed,
                        charging_station_x, charging_station_y, creation_time
                    ) VALUES (?, ?, ?, ?, ?, ?, ?)
                """, (
                    robot_data['robot_id'],
                    robot_data['robot_type'],
                    robot_data['max_battery'],
                    robot_data['speed'],
                    robot_data['charging_station'][0],
                    robot_data['charging_station'][1],
                    datetime.now()
                ))
                conn.commit()
                return True
        except sqlite3.Error as e:
            print(f"Error inserting robot: {e}")
            return False
            
    def insert_task(self, task_data: Dict[str, Any]) -> bool:
        """Insert a new task into the database."""
        try:
            with self.get_connection() as conn:
                conn.execute("""
                    INSERT INTO tasks (
                        task_id, pickup_x, pickup_y, dropoff_x, dropoff_y,
                        priority, status, creation_time, estimated_distance
                    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                """, (
                    task_data['task_id'],
                    task_data['pickup_location'][0],
                    task_data['pickup_location'][1],
                    task_data['dropoff_location'][0],
                    task_data['dropoff_location'][1],
                    task_data['priority'],
                    task_data['status'],
                    task_data['time_created'],
                    task_data.get('estimated_distance', 0)
                ))
                conn.commit()
                return True
        except sqlite3.Error as e:
            print(f"Error inserting task: {e}")
            return False
            
    def update_task_assignment(self, task_id: str, robot_id: str) -> bool:
        """Update task with robot assignment."""
        try:
            with self.get_connection() as conn:
                conn.execute("""
                    UPDATE tasks 
                    SET assigned_robot_id = ?, assignment_time = ?, status = 'assigned'
                    WHERE task_id = ?
                """, (robot_id, datetime.now(), task_id))
                conn.commit()
                return True
        except sqlite3.Error as e:
            print(f"Error updating task assignment: {e}")
            return False
            
    def update_task_status(self, task_id: str, status: str, completion_time: datetime = None) -> bool:
        """Update task status and completion time."""
        try:
            with self.get_connection() as conn:
                if status == 'in_progress':
                    conn.execute("""
                        UPDATE tasks 
                        SET status = ?, start_time = ?
                        WHERE task_id = ?
                    """, (status, datetime.now(), task_id))
                elif status == 'completed':
                    conn.execute("""
                        UPDATE tasks 
                        SET status = ?, completion_time = ?
                        WHERE task_id = ?
                    """, (status, completion_time or datetime.now(), task_id))
                else:
                    conn.execute("""
                        UPDATE tasks 
                        SET status = ?
                        WHERE task_id = ?
                    """, (status, task_id))
                conn.commit()
                return True
        except sqlite3.Error as e:
            print(f"Error updating task status: {e}")
            return False
            
    def log_robot_movement(self, robot_id: str, position: Tuple[int, int], 
                          battery_level: int, status: str, current_task_id: str = None) -> bool:
        """Log robot movement and status."""
        try:
            with self.get_connection() as conn:
                conn.execute("""
                    INSERT INTO robot_movements (
                        robot_id, position_x, position_y, battery_level, 
                        status, current_task_id
                    ) VALUES (?, ?, ?, ?, ?, ?)
                """, (
                    robot_id, position[0], position[1], 
                    battery_level, status, current_task_id
                ))
                conn.commit()
                return True
        except sqlite3.Error as e:
            print(f"Error logging robot movement: {e}")
            return False
            
    def log_system_metrics(self, metrics_data: Dict[str, Any]) -> bool:
        """Log system-wide performance metrics."""
        try:
            with self.get_connection() as conn:
                conn.execute("""
                    INSERT INTO system_metrics (
                        total_robots, active_robots, total_tasks, pending_tasks,
                        completed_tasks, average_battery_level, system_efficiency,
                        grid_utilization
                    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                """, (
                    metrics_data['total_robots'],
                    metrics_data['active_robots'],
                    metrics_data['total_tasks'],
                    metrics_data['pending_tasks'],
                    metrics_data['completed_tasks'],
                    metrics_data['average_battery'],
                    metrics_data['system_efficiency'],
                    metrics_data['grid_utilization']
                ))
                conn.commit()
                return True
        except sqlite3.Error as e:
            print(f"Error logging system metrics: {e}")
            return False
            
    def log_optimization_run(self, optimization_data: Dict[str, Any]) -> bool:
        """Log optimization algorithm performance."""
        try:
            with self.get_connection() as conn:
                conn.execute("""
                    INSERT INTO optimization_runs (
                        algorithm_type, execution_time_ms, tasks_assigned,
                        robots_utilized, optimal_solution, objective_value,
                        constraints_satisfied, total_constraints
                    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                """, (
                    optimization_data['algorithm_type'],
                    optimization_data['execution_time_ms'],
                    optimization_data['tasks_assigned'],
                    optimization_data['robots_utilized'],
                    optimization_data['optimal_solution'],
                    optimization_data.get('objective_value'),
                    optimization_data.get('constraints_satisfied', 0),
                    optimization_data.get('total_constraints', 0)
                ))
                conn.commit()
                return True
        except sqlite3.Error as e:
            print(f"Error logging optimization run: {e}")
            return False
            
    def log_pathfinding_stats(self, pathfinding_data: Dict[str, Any]) -> bool:
        """Log pathfinding algorithm performance."""
        try:
            with self.get_connection() as conn:
                conn.execute("""
                    INSERT INTO pathfinding_stats (
                        robot_id, task_id, start_x, start_y, goal_x, goal_y,
                        algorithm_used, path_length, computation_time_ms,
                        nodes_explored, path_optimal
                    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """, (
                    pathfinding_data['robot_id'],
                    pathfinding_data.get('task_id'),
                    pathfinding_data['start'][0],
                    pathfinding_data['start'][1],
                    pathfinding_data['goal'][0],
                    pathfinding_data['goal'][1],
                    pathfinding_data['algorithm_used'],
                    pathfinding_data['path_length'],
                    pathfinding_data['computation_time_ms'],
                    pathfinding_data['nodes_explored'],
                    pathfinding_data['path_optimal']
                ))
                conn.commit()
                return True
        except sqlite3.Error as e:
            print(f"Error logging pathfinding stats: {e}")
            return False
            
    def get_robot_performance(self) -> List[Dict[str, Any]]:
        """Get robot performance analytics."""
        try:
            with self.get_connection() as conn:
                cursor = conn.execute("SELECT * FROM robot_performance")
                return [dict(row) for row in cursor.fetchall()]
        except sqlite3.Error as e:
            print(f"Error fetching robot performance: {e}")
            return []
            
    def get_task_completion_stats(self, days: int = 7) -> List[Dict[str, Any]]:
        """Get task completion statistics for the last N days."""
        try:
            with self.get_connection() as conn:
                cursor = conn.execute("""
                    SELECT * FROM task_completion_stats 
                    WHERE date >= date('now', '-{} days')
                    ORDER BY date DESC
                """.format(days))
                return [dict(row) for row in cursor.fetchall()]
        except sqlite3.Error as e:
            print(f"Error fetching task completion stats: {e}")
            return []
            
    def get_system_performance_summary(self, days: int = 7) -> List[Dict[str, Any]]:
        """Get system performance summary for the last N days."""
        try:
            with self.get_connection() as conn:
                cursor = conn.execute("""
                    SELECT * FROM system_performance_summary 
                    WHERE date >= date('now', '-{} days')
                    ORDER BY date DESC
                """.format(days))
                return [dict(row) for row in cursor.fetchall()]
        except sqlite3.Error as e:
            print(f"Error fetching system performance summary: {e}")
            return []
            
    def get_optimization_performance(self, limit: int = 100) -> List[Dict[str, Any]]:
        """Get recent optimization algorithm performance."""
        try:
            with self.get_connection() as conn:
                cursor = conn.execute("""
                    SELECT * FROM optimization_runs 
                    ORDER BY timestamp DESC 
                    LIMIT ?
                """, (limit,))
                return [dict(row) for row in cursor.fetchall()]
        except sqlite3.Error as e:
            print(f"Error fetching optimization performance: {e}")
            return []
            
    def cleanup_old_data(self, days_to_keep: int = 30) -> bool:
        """Clean up old movement and metrics data to manage database size."""
        try:
            with self.get_connection() as conn:
                # Clean old robot movements
                conn.execute("""
                    DELETE FROM robot_movements 
                    WHERE timestamp < date('now', '-{} days')
                """.format(days_to_keep))
                
                # Clean old system metrics (keep daily aggregates)
                conn.execute("""
                    DELETE FROM system_metrics 
                    WHERE timestamp < date('now', '-{} days')
                """.format(days_to_keep))
                
                conn.commit()
                return True
        except sqlite3.Error as e:
            print(f"Error cleaning up old data: {e}")
            return False
            
    def get_database_stats(self) -> Dict[str, Any]:
        """Get database statistics and health metrics."""
        try:
            with self.get_connection() as conn:
                stats = {}
                
                # Table counts
                for table in ['robots', 'tasks', 'robot_movements', 'system_metrics', 
                             'optimization_runs', 'pathfinding_stats']:
                    cursor = conn.execute(f"SELECT COUNT(*) FROM {table}")
                    stats[f"{table}_count"] = cursor.fetchone()[0]
                
                # Database size
                cursor = conn.execute("PRAGMA page_count")
                page_count = cursor.fetchone()[0]
                cursor = conn.execute("PRAGMA page_size")
                page_size = cursor.fetchone()[0]
                stats['database_size_mb'] = round((page_count * page_size) / (1024 * 1024), 2)
                
                return stats
        except sqlite3.Error as e:
            print(f"Error getting database stats: {e}")
            return {}