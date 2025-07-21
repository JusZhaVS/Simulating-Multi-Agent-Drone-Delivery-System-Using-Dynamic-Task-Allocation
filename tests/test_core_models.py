import unittest
import sys
import os
from datetime import datetime, timedelta

# Add src to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from models.robot import Robot
from models.task import Task
from models.city_grid import CityGrid

class TestTask(unittest.TestCase):
    """
    Unit tests for the Task class.
    Tests object-oriented design and core functionality.
    """
    
    def setUp(self):
        """Set up test fixtures."""
        self.pickup = (5, 10)
        self.dropoff = (15, 20)
        self.priority = 3
        self.task = Task(self.pickup, self.dropoff, self.priority)
        
    def test_task_initialization(self):
        """Test task is properly initialized."""
        self.assertEqual(self.task.pickup_location, self.pickup)
        self.assertEqual(self.task.dropoff_location, self.dropoff)
        self.assertEqual(self.task.priority, self.priority)
        self.assertEqual(self.task.status, "pending")
        self.assertIsInstance(self.task.task_id, str)
        self.assertIsInstance(self.task.time_created, datetime)
        
    def test_task_assignment(self):
        """Test task assignment to robot."""
        robot_id = "test-robot-123"
        self.task.assign_to_robot(robot_id)
        
        self.assertEqual(self.task.assigned_robot_id, robot_id)
        self.assertEqual(self.task.status, "assigned")
        
    def test_task_progression(self):
        """Test task status progression."""
        # Start task
        self.task.start_task()
        self.assertEqual(self.task.status, "in_progress")
        
        # Complete task
        self.task.complete_task()
        self.assertEqual(self.task.status, "completed")
        
        # Test failure
        task2 = Task((0, 0), (1, 1), 1)
        task2.fail_task()
        self.assertEqual(task2.status, "failed")
        
    def test_distance_calculation(self):
        """Test Manhattan distance calculation."""
        expected_distance = abs(15 - 5) + abs(20 - 10)
        self.assertEqual(self.task.get_total_distance(), expected_distance)
        
    def test_task_comparison(self):
        """Test task priority comparison for sorting."""
        high_priority_task = Task((0, 0), (1, 1), 5)
        low_priority_task = Task((2, 2), (3, 3), 1)
        
        self.assertLess(high_priority_task, low_priority_task)
        
    def test_task_representation(self):
        """Test string representation."""
        repr_str = repr(self.task)
        self.assertIn("Task", repr_str)
        self.assertIn(self.task.task_id[:8], repr_str)

class TestRobot(unittest.TestCase):
    """
    Unit tests for the Robot class.
    Tests robot behavior, battery management, and task execution.
    """
    
    def setUp(self):
        """Set up test fixtures."""
        self.start_location = (10, 10)
        self.robot = Robot(self.start_location, max_battery=100, speed=1.0)
        
    def test_robot_initialization(self):
        """Test robot is properly initialized."""
        self.assertEqual(self.robot.location, self.start_location)
        self.assertEqual(self.robot.max_battery, 100)
        self.assertEqual(self.robot.current_battery, 100)
        self.assertEqual(self.robot.status, "idle")
        self.assertEqual(self.robot.charging_station, self.start_location)
        self.assertIsInstance(self.robot.robot_id, str)
        
    def test_task_assignment(self):
        """Test task assignment to robot."""
        task = Task((5, 5), (15, 15), 1)
        
        # Should succeed with full battery
        result = self.robot.assign_task(task)
        self.assertTrue(result)
        self.assertEqual(self.robot.current_task, task)
        self.assertEqual(self.robot.status, "en_route")
        
    def test_task_assignment_insufficient_battery(self):
        """Test task assignment fails with insufficient battery."""
        # Drain battery
        self.robot.current_battery = 5
        
        # Create long-distance task
        task = Task((0, 0), (50, 50), 1)
        
        result = self.robot.assign_task(task)
        self.assertFalse(result)
        self.assertIsNone(self.robot.current_task)
        
    def test_robot_movement(self):
        """Test robot movement mechanics."""
        target = (11, 10)
        initial_battery = self.robot.current_battery
        
        # Move towards target
        reached = self.robot.move_to(target)
        
        self.assertTrue(reached)
        self.assertEqual(self.robot.location, target)
        self.assertEqual(self.robot.current_battery, initial_battery - 1)
        self.assertEqual(self.robot.total_distance_traveled, 1)
        
    def test_robot_movement_no_battery(self):
        """Test robot cannot move with no battery."""
        self.robot.current_battery = 0
        target = (11, 10)
        
        reached = self.robot.move_to(target)
        
        self.assertFalse(reached)
        self.assertEqual(self.robot.location, self.start_location)
        self.assertEqual(self.robot.status, "failed")
        
    def test_robot_charging(self):
        """Test robot charging mechanics."""
        # Drain battery
        self.robot.current_battery = 30
        self.robot.status = "charging"
        
        # Charge robot
        self.robot.charge(amount=60)
        
        self.assertEqual(self.robot.current_battery, 90)
        
        # Test overcharge protection
        self.robot.charge(amount=50)
        self.assertEqual(self.robot.current_battery, 100)  # Should not exceed max
        
    def test_needs_charging(self):
        """Test charging threshold detection."""
        # Full battery - no charging needed
        self.assertFalse(self.robot.needs_charging())
        
        # Low battery - charging needed
        self.robot.current_battery = 20
        self.assertTrue(self.robot.needs_charging())
        
    def test_task_completion(self):
        """Test task completion workflow."""
        task = Task((5, 5), (6, 6), 1)
        self.robot.assign_task(task)
        
        # Complete task
        self.robot.complete_current_task()
        
        self.assertEqual(task.status, "completed")
        self.assertEqual(self.robot.tasks_completed, 1)
        self.assertIsNone(self.robot.current_task)
        self.assertEqual(self.robot.status, "idle")
        
    def test_efficiency_calculation(self):
        """Test robot efficiency calculation."""
        # New robot has zero efficiency
        self.assertEqual(self.robot.get_efficiency_score(), 0.0)
        
        # Complete a task and move
        self.robot.tasks_completed = 2
        self.robot.total_distance_traveled = 10
        
        expected_efficiency = 2 / 10
        self.assertEqual(self.robot.get_efficiency_score(), expected_efficiency)
        
    def test_status_report(self):
        """Test comprehensive status report generation."""
        report = self.robot.get_status_report()
        
        required_fields = ['robot_id', 'location', 'battery', 'status', 
                          'tasks_completed', 'total_distance', 'efficiency', 'uptime']
        
        for field in required_fields:
            self.assertIn(field, report)
            
    def test_thread_safety(self):
        """Test robot operations are thread-safe."""
        import threading
        
        def move_robot():
            for _ in range(10):
                self.robot.move_to((self.robot.location[0] + 1, self.robot.location[1]))
                
        # Create multiple threads
        threads = [threading.Thread(target=move_robot) for _ in range(3)]
        
        # Start all threads
        for thread in threads:
            thread.start()
            
        # Wait for completion
        for thread in threads:
            thread.join()
            
        # Robot should still be in valid state
        self.assertGreaterEqual(self.robot.current_battery, 0)
        self.assertGreater(self.robot.total_distance_traveled, 0)

class TestCityGrid(unittest.TestCase):
    """
    Unit tests for the CityGrid class.
    Tests environment management and spatial operations.
    """
    
    def setUp(self):
        """Set up test fixtures."""
        self.width = 20
        self.height = 15
        self.grid = CityGrid(self.width, self.height, obstacle_density=0.1)
        
    def test_grid_initialization(self):
        """Test grid is properly initialized."""
        self.assertEqual(self.grid.width, self.width)
        self.assertEqual(self.grid.height, self.height)
        self.assertIsInstance(self.grid.obstacles, set)
        self.assertIsInstance(self.grid.charging_stations, list)
        self.assertGreater(len(self.grid.charging_stations), 0)
        
    def test_location_validation(self):
        """Test location validation."""
        # Valid locations
        self.assertTrue(self.grid.is_valid_location((0, 0)))
        self.assertTrue(self.grid.is_valid_location((self.width - 1, self.height - 1)))
        self.assertTrue(self.grid.is_valid_location((10, 5)))
        
        # Invalid locations (out of bounds)
        self.assertFalse(self.grid.is_valid_location((-1, 0)))
        self.assertFalse(self.grid.is_valid_location((0, -1)))
        self.assertFalse(self.grid.is_valid_location((self.width, 0)))
        self.assertFalse(self.grid.is_valid_location((0, self.height)))
        
    def test_neighbor_generation(self):
        """Test neighbor cell generation."""
        # Center location
        center = (10, 7)
        neighbors = self.grid.get_neighbors(center)
        
        expected_neighbors = [(11, 7), (9, 7), (10, 8), (10, 6)]
        
        # All expected neighbors should be valid
        for neighbor in expected_neighbors:
            if self.grid.is_valid_location(neighbor):
                self.assertIn(neighbor, neighbors)
                
        # All returned neighbors should be valid
        for neighbor in neighbors:
            self.assertTrue(self.grid.is_valid_location(neighbor))
            
    def test_robot_management(self):
        """Test robot addition and removal."""
        robot = Robot((5, 5))
        
        # Add robot
        self.grid.add_robot(robot)
        self.assertIn(robot.robot_id, self.grid.robots)
        
        # Remove robot
        self.grid.remove_robot(robot.robot_id)
        self.assertNotIn(robot.robot_id, self.grid.robots)
        
    def test_task_management(self):
        """Test task addition and completion."""
        task = Task((1, 1), (10, 10), 1)
        
        # Add task
        self.grid.add_task(task)
        self.assertIn(task.task_id, self.grid.active_tasks)
        
        # Complete task
        self.grid.complete_task(task.task_id)
        self.assertNotIn(task.task_id, self.grid.active_tasks)
        
    def test_charging_station_finder(self):
        """Test nearest charging station detection."""
        location = (10, 10)
        nearest = self.grid.get_nearest_charging_station(location)
        
        self.assertIsInstance(nearest, tuple)
        self.assertEqual(len(nearest), 2)
        self.assertTrue(self.grid.is_valid_location(nearest))
        
    def test_random_location_generation(self):
        """Test random valid location generation."""
        for _ in range(100):  # Test multiple random locations
            location = self.grid.get_random_valid_location()
            self.assertTrue(self.grid.is_valid_location(location))
            
    def test_random_task_generation(self):
        """Test random task generation."""
        task = self.grid.generate_random_task()
        
        self.assertIsInstance(task, Task)
        self.assertTrue(self.grid.is_valid_location(task.pickup_location))
        self.assertTrue(self.grid.is_valid_location(task.dropoff_location))
        self.assertNotEqual(task.pickup_location, task.dropoff_location)
        
    def test_distance_calculation(self):
        """Test Manhattan distance calculation."""
        loc1 = (0, 0)
        loc2 = (3, 4)
        
        distance = self.grid.calculate_manhattan_distance(loc1, loc2)
        expected = 3 + 4
        
        self.assertEqual(distance, expected)
        
    def test_grid_state_export(self):
        """Test grid state export for visualization."""
        # Add some robots and tasks
        robot = Robot((5, 5))
        task = Task((1, 1), (10, 10), 1)
        
        self.grid.add_robot(robot)
        self.grid.add_task(task)
        
        state = self.grid.get_grid_state()
        
        required_fields = ['dimensions', 'obstacles', 'charging_stations', 'robots', 'active_tasks']
        for field in required_fields:
            self.assertIn(field, state)
            
        self.assertEqual(state['dimensions'], (self.width, self.height))
        self.assertIn(robot.robot_id, state['robots'])
        self.assertIn(task.task_id, state['active_tasks'])
        
    def test_system_statistics(self):
        """Test system statistics generation."""
        # Add some robots and tasks
        robot1 = Robot((1, 1))
        robot2 = Robot((2, 2))
        robot1.tasks_completed = 5
        robot2.tasks_completed = 3
        
        self.grid.add_robot(robot1)
        self.grid.add_robot(robot2)
        
        task = Task((5, 5), (10, 10), 1)
        self.grid.add_task(task)
        
        stats = self.grid.get_system_statistics()
        
        self.assertEqual(stats['total_robots'], 2)
        self.assertEqual(stats['total_tasks'], 1)
        self.assertEqual(stats['completed_tasks'], 8)  # 5 + 3
        self.assertGreater(stats['average_battery'], 0)

if __name__ == '__main__':
    # Run all tests
    unittest.main(verbosity=2)