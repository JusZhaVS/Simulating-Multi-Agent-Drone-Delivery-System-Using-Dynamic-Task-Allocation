import unittest
import sys
import os
import time

# Add src to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from models.robot import Robot
from models.task import Task
from models.city_grid import CityGrid
from algorithms.pathfinding import AStarPathfinder, Node
from algorithms.optimization import TaskAllocationOptimizer, GreedyTaskAllocator, HybridOptimizer

class TestNode(unittest.TestCase):
    """
    Unit tests for the Node class used in A* pathfinding.
    """
    
    def test_node_creation(self):
        """Test node creation and initialization."""
        position = (5, 10)
        node = Node(position, g_cost=10, h_cost=15)
        
        self.assertEqual(node.position, position)
        self.assertEqual(node.g_cost, 10)
        self.assertEqual(node.h_cost, 15)
        self.assertEqual(node.f_cost, 25)  # g + h
        self.assertIsNone(node.parent)
        
    def test_node_comparison(self):
        """Test node comparison for priority queue ordering."""
        node1 = Node((0, 0), g_cost=10, h_cost=5)  # f_cost = 15
        node2 = Node((1, 1), g_cost=12, h_cost=8)  # f_cost = 20
        node3 = Node((2, 2), g_cost=8, h_cost=7)   # f_cost = 15, but h_cost = 7
        
        # node1 should be less than node2 (lower f_cost)
        self.assertLess(node1, node2)
        
        # For equal f_costs, compare h_cost
        self.assertLess(node3, node1)  # Same f_cost, but lower h_cost

class TestAStarPathfinder(unittest.TestCase):
    """
    Unit tests for the A* pathfinding algorithm.
    Tests algorithm correctness, optimality, and performance.
    """
    
    def setUp(self):
        """Set up test fixtures."""
        # Create a simple grid without obstacles
        self.grid = CityGrid(10, 10, obstacle_density=0.0)
        self.pathfinder = AStarPathfinder(self.grid)
        
    def test_manhattan_distance_heuristic(self):
        """Test Manhattan distance heuristic calculation."""
        pos1 = (0, 0)
        pos2 = (3, 4)
        
        distance = self.pathfinder.manhattan_distance(pos1, pos2)
        expected = 3 + 4
        
        self.assertEqual(distance, expected)
        
    def test_euclidean_distance_heuristic(self):
        """Test Euclidean distance heuristic calculation."""
        pos1 = (0, 0)
        pos2 = (3, 4)
        
        distance = self.pathfinder.euclidean_distance(pos1, pos2)
        expected = 5.0  # sqrt(3^2 + 4^2)
        
        self.assertAlmostEqual(distance, expected, places=5)
        
    def test_simple_path_finding(self):
        """Test pathfinding in simple scenarios."""
        start = (0, 0)
        goal = (3, 3)
        
        path = self.pathfinder.find_path(start, goal)
        
        self.assertIsNotNone(path)
        self.assertEqual(path[0], start)
        self.assertEqual(path[-1], goal)
        self.assertGreaterEqual(len(path), 7)  # Manhattan distance + 1
        
    def test_same_start_goal(self):
        """Test pathfinding when start equals goal."""
        position = (5, 5)
        path = self.pathfinder.find_path(position, position)
        
        self.assertEqual(path, [position])
        
    def test_invalid_locations(self):
        """Test pathfinding with invalid start or goal locations."""
        valid_pos = (5, 5)
        invalid_pos = (-1, -1)
        
        # Invalid start
        path1 = self.pathfinder.find_path(invalid_pos, valid_pos)
        self.assertIsNone(path1)
        
        # Invalid goal
        path2 = self.pathfinder.find_path(valid_pos, invalid_pos)
        self.assertIsNone(path2)
        
    def test_path_with_obstacles(self):
        """Test pathfinding around obstacles."""
        # Create grid with specific obstacles
        self.grid.obstacles = {(1, 1), (1, 2), (2, 1)}
        
        start = (0, 0)
        goal = (2, 2)
        
        path = self.pathfinder.find_path(start, goal)
        
        self.assertIsNotNone(path)
        self.assertEqual(path[0], start)
        self.assertEqual(path[-1], goal)
        
        # Path should avoid obstacles
        for position in path:
            self.assertNotIn(position, self.grid.obstacles)
            
    def test_no_path_exists(self):
        """Test pathfinding when no path exists."""
        # Create a wall of obstacles
        for x in range(1, 9):
            self.grid.obstacles.add((x, 5))
            
        start = (0, 0)
        goal = (9, 9)
        
        path = self.pathfinder.find_path(start, goal)
        
        # Should still find path around the wall
        self.assertIsNotNone(path)
        
        # Test completely blocked scenario
        # Surround goal with obstacles
        goal = (5, 5)
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx != 0 or dy != 0:
                    self.grid.obstacles.add((goal[0] + dx, goal[1] + dy))
                    
        path = self.pathfinder.find_path(start, goal)
        self.assertIsNone(path)
        
    def test_waypoint_pathfinding(self):
        """Test pathfinding through multiple waypoints."""
        start = (0, 0)
        waypoints = [(2, 2), (5, 2), (5, 5)]
        
        path = self.pathfinder.find_path_with_waypoints(start, waypoints)
        
        self.assertIsNotNone(path)
        self.assertEqual(path[0], start)
        self.assertEqual(path[-1], waypoints[-1])
        
        # All waypoints should be in the path
        for waypoint in waypoints:
            self.assertIn(waypoint, path)
            
    def test_path_cost_calculation(self):
        """Test path cost calculation."""
        path = [(0, 0), (1, 0), (2, 0), (2, 1)]
        cost = self.pathfinder.calculate_path_cost(path)
        
        expected_cost = 3.0  # 3 moves
        self.assertEqual(cost, expected_cost)
        
        # Empty path
        self.assertEqual(self.pathfinder.calculate_path_cost([]), 0.0)
        self.assertEqual(self.pathfinder.calculate_path_cost([(0, 0)]), 0.0)
        
    def test_algorithm_performance(self):
        """Test pathfinding performance on larger grids."""
        # Create larger grid
        large_grid = CityGrid(50, 50, obstacle_density=0.1)
        large_pathfinder = AStarPathfinder(large_grid)
        
        start_time = time.time()
        path = large_pathfinder.find_path((0, 0), (49, 49))
        end_time = time.time()
        
        self.assertIsNotNone(path)
        self.assertLess(end_time - start_time, 1.0)  # Should complete within 1 second
        
    def test_heuristic_comparison(self):
        """Test different heuristic functions."""
        start = (0, 0)
        goal = (5, 5)
        
        manhattan_path = self.pathfinder.find_path(start, goal, heuristic="manhattan")
        euclidean_path = self.pathfinder.find_path(start, goal, heuristic="euclidean")
        
        self.assertIsNotNone(manhattan_path)
        self.assertIsNotNone(euclidean_path)
        
        # Both should find valid paths
        self.assertEqual(manhattan_path[0], start)
        self.assertEqual(manhattan_path[-1], goal)
        self.assertEqual(euclidean_path[0], start)
        self.assertEqual(euclidean_path[-1], goal)

class TestTaskAllocationOptimizer(unittest.TestCase):
    """
    Unit tests for linear programming task allocation optimizer.
    Tests optimization correctness and constraint satisfaction.
    """
    
    def setUp(self):
        """Set up test fixtures."""
        self.grid = CityGrid(20, 20, obstacle_density=0.0)
        self.optimizer = TaskAllocationOptimizer(self.grid)
        
    def test_empty_inputs(self):
        """Test optimization with empty inputs."""
        result = self.optimizer.optimize_task_allocation([], [])
        
        self.assertEqual(result["assignments"], {})
        self.assertEqual(result["objective_value"], 0)
        self.assertEqual(result["status"], "optimal")
        
    def test_no_robots_available(self):
        """Test optimization with no available robots."""
        tasks = [Task((0, 0), (5, 5), 1)]
        result = self.optimizer.optimize_task_allocation([], tasks)
        
        self.assertEqual(result["assignments"], {})
        
    def test_no_tasks_available(self):
        """Test optimization with no tasks."""
        robots = [Robot((0, 0))]
        result = self.optimizer.optimize_task_allocation(robots, [])
        
        self.assertEqual(result["assignments"], {})
        
    def test_simple_assignment(self):
        """Test simple one-robot-one-task assignment."""
        robot = Robot((0, 0), max_battery=100)
        task = Task((1, 1), (2, 2), 1)
        
        result = self.optimizer.optimize_task_allocation([robot], [task])
        
        self.assertEqual(len(result["assignments"]), 1)
        self.assertIn(robot.robot_id, result["assignments"])
        self.assertEqual(result["assignments"][robot.robot_id], task.task_id)
        self.assertEqual(result["status"], "optimal")
        
    def test_multiple_robots_multiple_tasks(self):
        """Test assignment with multiple robots and tasks."""
        robots = [
            Robot((0, 0), max_battery=100),
            Robot((10, 10), max_battery=100),
            Robot((5, 5), max_battery=100)
        ]
        
        tasks = [
            Task((1, 1), (2, 2), 1),
            Task((11, 11), (12, 12), 2),
            Task((6, 6), (7, 7), 3)
        ]
        
        result = self.optimizer.optimize_task_allocation(robots, tasks)
        
        # Should make assignments
        self.assertGreater(len(result["assignments"]), 0)
        self.assertEqual(result["status"], "optimal")
        
        # No robot should be assigned multiple tasks
        assigned_robots = list(result["assignments"].keys())
        self.assertEqual(len(assigned_robots), len(set(assigned_robots)))
        
        # No task should be assigned to multiple robots
        assigned_tasks = list(result["assignments"].values())
        self.assertEqual(len(assigned_tasks), len(set(assigned_tasks)))
        
    def test_battery_constraint(self):
        """Test that battery constraints are respected."""
        # Robot with very low battery
        robot = Robot((0, 0), max_battery=100)
        robot.current_battery = 5  # Very low battery
        
        # Task requiring long distance
        task = Task((0, 0), (15, 15), 1)
        
        result = self.optimizer.optimize_task_allocation([robot], [task])
        
        # Should not assign task due to insufficient battery
        self.assertEqual(len(result["assignments"]), 0)
        
    def test_priority_consideration(self):
        """Test that task priorities are considered in optimization."""
        robot1 = Robot((0, 0), max_battery=100)
        robot2 = Robot((1, 1), max_battery=100)
        
        high_priority_task = Task((0, 1), (0, 2), 5)  # High priority
        low_priority_task = Task((2, 2), (3, 3), 1)   # Low priority
        
        result = self.optimizer.optimize_task_allocation(
            [robot1, robot2], 
            [high_priority_task, low_priority_task]
        )
        
        # Should make assignments considering priority
        self.assertGreater(len(result["assignments"]), 0)
        
    def test_cost_matrix_calculation(self):
        """Test cost matrix calculation for robot-task pairs."""
        robots = [Robot((0, 0)), Robot((5, 5))]
        tasks = [Task((1, 1), (2, 2), 1), Task((6, 6), (7, 7), 2)]
        
        costs = self.optimizer._calculate_cost_matrix(robots, tasks)
        
        # Should have costs for all robot-task combinations
        self.assertEqual(len(costs), 2)
        for robot_id in costs:
            self.assertEqual(len(costs[robot_id]), 2)
            
        # Costs should be positive
        for robot_id in costs:
            for task_id in costs[robot_id]:
                self.assertGreater(costs[robot_id][task_id], 0)
                
    def test_optimization_performance(self):
        """Test optimization performance with larger problem sizes."""
        # Create moderate-sized problem
        robots = [Robot((i, j)) for i in range(5) for j in range(5)]  # 25 robots
        tasks = [Task((i, j), (i+1, j+1), 1) for i in range(10) for j in range(10)]  # 100 tasks
        
        start_time = time.time()
        result = self.optimizer.optimize_task_allocation(robots, tasks)
        end_time = time.time()
        
        # Should complete reasonably quickly
        self.assertLess(end_time - start_time, 10.0)  # 10 seconds max
        self.assertEqual(result["status"], "optimal")

class TestGreedyTaskAllocator(unittest.TestCase):
    """
    Unit tests for greedy task allocation algorithm.
    Tests heuristic approach and performance characteristics.
    """
    
    def setUp(self):
        """Set up test fixtures."""
        self.grid = CityGrid(20, 20, obstacle_density=0.0)
        self.allocator = GreedyTaskAllocator(self.grid)
        
    def test_greedy_simple_assignment(self):
        """Test simple greedy assignment."""
        robot = Robot((0, 0), max_battery=100)
        task = Task((1, 1), (2, 2), 1)
        
        result = self.allocator.allocate_tasks_greedy([robot], [task])
        
        self.assertEqual(len(result["assignments"]), 1)
        self.assertIn(robot.robot_id, result["assignments"])
        
    def test_greedy_priority_ordering(self):
        """Test that greedy algorithm respects task priorities."""
        robot = Robot((0, 0), max_battery=100)
        
        high_priority_task = Task((1, 1), (2, 2), 5)
        low_priority_task = Task((1, 1), (2, 2), 1)
        
        result = self.allocator.allocate_tasks_greedy(
            [robot], 
            [low_priority_task, high_priority_task]  # Low priority first
        )
        
        # Should assign high priority task
        self.assertEqual(len(result["assignments"]), 1)
        assigned_task_id = result["assignments"][robot.robot_id]
        self.assertEqual(assigned_task_id, high_priority_task.task_id)
        
    def test_greedy_performance(self):
        """Test greedy algorithm performance."""
        # Large problem that would be slow for LP
        robots = [Robot((i, j)) for i in range(10) for j in range(10)]  # 100 robots
        tasks = [Task((i, j), (i+1, j+1), 1) for i in range(20) for j in range(20)]  # 400 tasks
        
        start_time = time.time()
        result = self.allocator.allocate_tasks_greedy(robots, tasks)
        end_time = time.time()
        
        # Should be very fast
        self.assertLess(end_time - start_time, 1.0)  # 1 second max
        self.assertGreater(len(result["assignments"]), 0)

class TestHybridOptimizer(unittest.TestCase):
    """
    Unit tests for hybrid optimizer that chooses between LP and greedy.
    Tests adaptive algorithm selection.
    """
    
    def setUp(self):
        """Set up test fixtures."""
        self.grid = CityGrid(20, 20, obstacle_density=0.0)
        self.optimizer = HybridOptimizer(self.grid, lp_threshold=50)
        
    def test_small_problem_uses_lp(self):
        """Test that small problems use linear programming."""
        robots = [Robot((i, 0)) for i in range(3)]  # 3 robots
        tasks = [Task((i, 1), (i, 2), 1) for i in range(3)]  # 3 tasks
        
        result = self.optimizer.optimize_allocation(robots, tasks)
        
        self.assertEqual(result["algorithm_type"], "hybrid_lp")
        self.assertIn("problem_size", result)
        self.assertIn("threshold_used", result)
        
    def test_large_problem_uses_greedy(self):
        """Test that large problems use greedy algorithm."""
        robots = [Robot((i, j)) for i in range(10) for j in range(10)]  # 100 robots
        tasks = [Task((i, j), (i+1, j+1), 1) for i in range(10) for j in range(10)]  # 100 tasks
        
        result = self.optimizer.optimize_allocation(robots, tasks)
        
        self.assertEqual(result["algorithm_type"], "hybrid_greedy")
        self.assertGreater(result["problem_size"], result["threshold_used"])
        
    def test_threshold_configuration(self):
        """Test custom threshold configuration."""
        # Very low threshold - should use greedy even for small problems
        low_threshold_optimizer = HybridOptimizer(self.grid, lp_threshold=1)
        
        robots = [Robot((0, 0)), Robot((1, 1))]
        tasks = [Task((0, 1), (0, 2), 1)]
        
        result = low_threshold_optimizer.optimize_allocation(robots, tasks)
        
        self.assertEqual(result["algorithm_type"], "hybrid_greedy")

if __name__ == '__main__':
    # Run all tests
    unittest.main(verbosity=2)