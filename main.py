#!/usr/bin/env python3
"""
SwarmDash - Multi-Agent Delivery System Simulation

A comprehensive simulation exploring autonomous delivery systems with:
- Object-oriented design and design patterns
- Advanced algorithms (A* pathfinding, linear programming optimization)
- Distributed systems architecture (REST API, client-server)
- Multi-threading and concurrency
- Database management and analytics
- Real-time GUI visualization
- Comprehensive testing

A passion project exploring the fascinating world of drone coordination and robotics.
"""

import argparse
import sys
import os
import time
import threading
from datetime import datetime

# Add src to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from services.task_assigner_service import TaskAssignerService
from services.robot_client import RobotClient, RobotFleetManager
from services.simulation_manager import SimulationManager
from gui.visualization import SwarmDashGUI

class SwarmDashMain:
    """
    Main orchestrator for the SwarmDash system.
    Demonstrates system integration and provides multiple operation modes.
    """
    
    def __init__(self):
        self.service = None
        self.simulation_manager = None
        self.gui = None
        
    def run_distributed_mode(self, num_robots: int = 10, num_tasks: int = 20, 
                           port: int = 5000, gui: bool = True):
        """
        Run SwarmDash in distributed mode with REST API service.
        
        This mode demonstrates:
        - Multi-tiered architecture (presentation, logic, data layers)
        - RESTful API design
        - Client-server communication
        - Real-time distributed task allocation
        """
        print("=" * 60)
        print("SwarmDash - Distributed Mode")
        print("=" * 60)
        print(f"Starting distributed system with {num_robots} robots")
        print(f"REST API server on port {port}")
        print(f"GUI visualization: {'Enabled' if gui else 'Disabled'}")
        print()
        
        try:
            # Start task assigner service in background thread
            self.service = TaskAssignerService(port=port)
            
            service_thread = threading.Thread(
                target=self.service.start_service,
                kwargs={'debug': False, 'threaded': True},
                daemon=True
            )
            service_thread.start()
            
            # Wait for service to start
            time.sleep(2)
            print(f"Task Assigner Service started on http://localhost:{port}")
            
            # Create robot fleet
            fleet_manager = RobotFleetManager(f"http://localhost:{port}")
            robots = fleet_manager.create_robot_fleet(num_robots)
            print(f"Created fleet of {len(robots)} robots")
            
            # Start all robots
            fleet_manager.start_all_robots()
            print("All robots started autonomous operation")
            
            # Generate some initial tasks
            self._generate_initial_tasks(f"http://localhost:{port}", num_tasks)
            print(f"Generated {num_tasks} initial delivery tasks")
            
            # Start GUI if requested
            if gui:
                try:
                    import pygame
                    print("Starting GUI visualization...")
                    self.gui = SwarmDashGUI(server_url=f"http://localhost:{port}")
                    self.gui.start_visualization()
                except ImportError:
                    print("pygame not available, running without GUI")
                    self._run_monitoring_loop()
            else:
                self._run_monitoring_loop()
                
        except KeyboardInterrupt:
            print("\nStopping distributed system...")
            if hasattr(fleet_manager, 'stop_all_robots'):
                fleet_manager.stop_all_robots()
            print("System stopped")
            
    def run_simulation_mode(self, num_robots: int = 15, task_rate: float = 2.0, 
                          duration: int = 300):
        """
        Run SwarmDash in multi-threaded simulation mode.
        
        This mode demonstrates:
        - Multi-threading and concurrency
        - Producer-consumer patterns
        - Thread-safe operations
        - Performance monitoring
        """
        print("=" * 60)
        print("SwarmDash - Multi-threaded Simulation Mode")
        print("=" * 60)
        print(f"Running simulation with {num_robots} robots")
        print(f"Task generation rate: {task_rate} tasks/second")
        print(f"Duration: {duration} seconds")
        print()
        
        try:
            # Create and start simulation
            self.simulation_manager = SimulationManager(
                grid_width=40, 
                grid_height=30,
                max_robots=num_robots,
                max_worker_threads=8
            )
            
            self.simulation_manager.start_simulation(
                num_robots=num_robots,
                task_generation_rate=task_rate
            )
            
            print("Multi-threaded simulation started")
            print("Robot threads active")
            print("Task generation and assignment threads running")
            print("Performance monitoring active")
            print()
            
            # Monitor simulation
            start_time = time.time()
            last_stats_time = start_time
            
            while time.time() - start_time < duration:
                time.sleep(10)  # Update every 10 seconds
                
                # Print statistics periodically
                if time.time() - last_stats_time >= 30:  # Every 30 seconds
                    stats = self.simulation_manager.get_simulation_statistics()
                    self._print_simulation_stats(stats)
                    last_stats_time = time.time()
                    
            print(f"\nSimulation completed after {duration} seconds")
            
            # Final statistics
            final_stats = self.simulation_manager.get_simulation_statistics()
            self._print_final_simulation_report(final_stats)
            
        except KeyboardInterrupt:
            print("\nStopping simulation...")
        finally:
            if self.simulation_manager:
                self.simulation_manager.stop_simulation()
            print("Simulation stopped")
            
    def run_algorithm_demo(self):
        """
        Demonstrate core algorithms with performance analysis.
        
        This mode demonstrates:
        - Algorithm implementation and analysis
        - Performance benchmarking
        - Complexity analysis
        - Mathematical optimization
        """
        print("=" * 60)
        print("SwarmDash - Algorithm Demonstration Mode")
        print("=" * 60)
        
        from models.city_grid import CityGrid
        from algorithms.pathfinding import AStarPathfinder
        from algorithms.optimization import TaskAllocationOptimizer, GreedyTaskAllocator, HybridOptimizer
        from models.robot import Robot
        from models.task import Task
        
        # Pathfinding demonstration
        print("1. A* Pathfinding Algorithm")
        print("-" * 30)
        
        grid = CityGrid(20, 20, obstacle_density=0.15)
        pathfinder = AStarPathfinder(grid)
        
        start = (0, 0)
        goal = (19, 19)
        
        start_time = time.time()
        path = pathfinder.find_path(start, goal)
        pathfinding_time = (time.time() - start_time) * 1000
        
        if path:
            print(f"Path found from {start} to {goal}")
            print(f"  Path length: {len(path)} steps")
            print(f"  Computation time: {pathfinding_time:.2f}ms")
            print(f"  Optimal path cost: {pathfinder.calculate_path_cost(path)}")
        else:
            print("No path found")
            
        # Optimization demonstration
        print("\n2. Task Allocation Optimization")
        print("-" * 30)
        
        # Create test scenario
        robots = [Robot((i*2, j*2)) for i in range(5) for j in range(3)]  # 15 robots
        tasks = [Task((i*3, j*3), (i*3+5, j*3+5), i+j+1) for i in range(4) for j in range(4)]  # 16 tasks
        
        # Test different optimizers
        optimizers = [
            ("Linear Programming", TaskAllocationOptimizer(grid)),
            ("Greedy Heuristic", GreedyTaskAllocator(grid)),
            ("Hybrid Optimizer", HybridOptimizer(grid, lp_threshold=50))
        ]
        
        for name, optimizer in optimizers:
            start_time = time.time()
            
            if hasattr(optimizer, 'optimize_task_allocation'):
                result = optimizer.optimize_task_allocation(robots, tasks)
            elif hasattr(optimizer, 'allocate_tasks_greedy'):
                result = optimizer.allocate_tasks_greedy(robots, tasks)
            else:
                result = optimizer.optimize_allocation(robots, tasks)
                
            optimization_time = (time.time() - start_time) * 1000
            
            print(f"\n{name}:")
            print(f"  Assignments made: {len(result['assignments'])}")
            print(f"  Computation time: {optimization_time:.2f}ms")
            print(f"  Algorithm status: {result.get('status', 'N/A')}")
            if result.get('objective_value'):
                print(f"  Objective value: {result['objective_value']:.2f}")
                
        print("\n3. Performance Scaling Analysis")
        print("-" * 30)
        
        # Test algorithm scaling
        problem_sizes = [(5, 5), (10, 10), (15, 15), (20, 20)]
        
        for num_robots, num_tasks in problem_sizes:
            robots = [Robot((i, j)) for i in range(num_robots) for j in range(1)][:num_robots]
            tasks = [Task((i, 0), (i, 10), 1) for i in range(num_tasks)]
            
            # Time greedy algorithm
            greedy = GreedyTaskAllocator(grid)
            start_time = time.time()
            greedy_result = greedy.allocate_tasks_greedy(robots, tasks)
            greedy_time = (time.time() - start_time) * 1000
            
            print(f"Problem size {num_robots}x{num_tasks}: Greedy = {greedy_time:.2f}ms")
            
    def run_tests(self):
        """
        Run comprehensive test suite.
        
        Demonstrates:
        - Unit testing methodology
        - Test coverage
        - Code quality assurance
        """
        print("=" * 60)
        print("SwarmDash - Test Suite")
        print("=" * 60)
        
        import unittest
        import sys
        
        # Discover and run tests
        loader = unittest.TestLoader()
        test_dir = os.path.join(os.path.dirname(__file__), 'tests')
        suite = loader.discover(test_dir, pattern='test_*.py')
        
        runner = unittest.TextTestRunner(verbosity=2)
        result = runner.run(suite)
        
        print("\n" + "=" * 60)
        print("Test Results Summary")
        print("=" * 60)
        print(f"Tests run: {result.testsRun}")
        print(f"Failures: {len(result.failures)}")
        print(f"Errors: {len(result.errors)}")
        print(f"Success rate: {((result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun * 100):.1f}%")
        
        if result.failures:
            print("\nFailures:")
            for test, traceback in result.failures:
                print(f"  - {test}")
                
        if result.errors:
            print("\nErrors:")
            for test, traceback in result.errors:
                print(f"  - {test}")
                
        return len(result.failures) == 0 and len(result.errors) == 0
        
    def _generate_initial_tasks(self, server_url: str, num_tasks: int):
        """Generate initial tasks for the distributed system."""
        import requests
        import random
        
        for i in range(num_tasks):
            pickup = (random.randint(0, 49), random.randint(0, 49))
            dropoff = (random.randint(0, 49), random.randint(0, 49))
            priority = random.randint(1, 5)
            
            payload = {
                "pickup_x": pickup[0],
                "pickup_y": pickup[1],
                "dropoff_x": dropoff[0],
                "dropoff_y": dropoff[1],
                "priority": priority
            }
            
            try:
                requests.post(f"{server_url}/api/tasks", json=payload, timeout=2.0)
            except requests.RequestException:
                pass  # Ignore failures during setup
                
    def _run_monitoring_loop(self):
        """Run monitoring loop for distributed mode without GUI."""
        import requests
        
        print("\nSystem Monitoring (Press Ctrl+C to stop)")
        print("=" * 50)
        
        try:
            while True:
                time.sleep(10)
                
                try:
                    response = requests.get("http://localhost:5000/api/system/status", timeout=2.0)
                    if response.status_code == 200:
                        stats = response.json()
                        grid_stats = stats.get('grid_statistics', {})
                        
                        print(f"[{datetime.now().strftime('%H:%M:%S')}] "
                              f"Robots: {grid_stats.get('total_robots', 0)} | "
                              f"Active: {grid_stats.get('active_robots', 0)} | "
                              f"Tasks: {grid_stats.get('total_tasks', 0)} | "
                              f"Completed: {grid_stats.get('completed_tasks', 0)} | "
                              f"Efficiency: {grid_stats.get('system_efficiency', 0):.3f}")
                              
                except requests.RequestException:
                    print(f"[{datetime.now().strftime('%H:%M:%S')}] Connection error")
                    
        except KeyboardInterrupt:
            pass
            
    def _print_simulation_stats(self, stats: dict):
        """Print simulation statistics."""
        print(f"\nSimulation Statistics [{datetime.now().strftime('%H:%M:%S')}]")
        print("-" * 50)
        
        robots = stats.get('robots', {})
        tasks = stats.get('tasks', {})
        performance = stats.get('performance', {})
        
        print(f"Robots: {robots.get('total', 0)} total, {robots.get('active_threads', 0)} active threads")
        print(f"Tasks: {tasks.get('pending', 0)} pending, {tasks.get('completed', 0)} completed")
        print(f"Queue: {tasks.get('queue_size', 0)}/{tasks.get('queue_max_size', 0)}")
        print(f"Performance: {performance.get('robot_operations', 0)} robot ops, "
              f"{performance.get('optimization_runs', 0)} optimizations")
              
        status_dist = robots.get('status_distribution', {})
        if status_dist:
            status_str = ", ".join([f"{status}: {count}" for status, count in status_dist.items()])
            print(f"Robot Status: {status_str}")
            
    def _print_final_simulation_report(self, stats: dict):
        """Print final simulation report."""
        print("\n" + "=" * 60)
        print("Final Simulation Report")
        print("=" * 60)
        
        uptime = stats.get('simulation_uptime', 'Unknown')
        performance = stats.get('performance', {})
        system_stats = stats.get('system_statistics', {})
        
        print(f"Simulation Duration: {uptime}")
        print(f"Total Tasks Generated: {performance.get('tasks_generated', 0)}")
        print(f"Total Tasks Completed: {performance.get('tasks_completed', 0)}")
        print(f"Task Completion Rate: {(performance.get('tasks_completed', 0) / max(performance.get('tasks_generated', 1), 1) * 100):.1f}%")
        print(f"Total Robot Operations: {performance.get('robot_operations', 0)}")
        print(f"Optimization Runs: {performance.get('optimization_runs', 0)}")
        print(f"Thread Pool Tasks: {performance.get('thread_pool_tasks', 0)}")
        print(f"Final System Efficiency: {system_stats.get('system_efficiency', 0):.3f}")

def main():
    """Main entry point with command-line interface."""
    parser = argparse.ArgumentParser(
        description="SwarmDash - Multi-Agent Delivery System Simulation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python main.py distributed --robots 15 --tasks 30 --gui
  python main.py simulation --robots 20 --rate 3.0 --duration 600
  python main.py algorithms
  python main.py test
        """
    )
    
    subparsers = parser.add_subparsers(dest='mode', help='Operation mode')
    
    # Distributed mode
    dist_parser = subparsers.add_parser('distributed', help='Run distributed system with REST API')
    dist_parser.add_argument('--robots', type=int, default=10, help='Number of robots (default: 10)')
    dist_parser.add_argument('--tasks', type=int, default=20, help='Initial number of tasks (default: 20)')
    dist_parser.add_argument('--port', type=int, default=5000, help='API server port (default: 5000)')
    dist_parser.add_argument('--gui', action='store_true', help='Enable GUI visualization')
    
    # Simulation mode
    sim_parser = subparsers.add_parser('simulation', help='Run multi-threaded simulation')
    sim_parser.add_argument('--robots', type=int, default=15, help='Number of robots (default: 15)')
    sim_parser.add_argument('--rate', type=float, default=2.0, help='Task generation rate (default: 2.0)')
    sim_parser.add_argument('--duration', type=int, default=300, help='Simulation duration in seconds (default: 300)')
    
    # Algorithm demo mode
    subparsers.add_parser('algorithms', help='Demonstrate core algorithms')
    
    # Test mode
    subparsers.add_parser('test', help='Run test suite')
    
    args = parser.parse_args()
    
    if not args.mode:
        parser.print_help()
        return
        
    app = SwarmDashMain()
    
    if args.mode == 'distributed':
        app.run_distributed_mode(
            num_robots=args.robots,
            num_tasks=args.tasks,
            port=args.port,
            gui=args.gui
        )
    elif args.mode == 'simulation':
        app.run_simulation_mode(
            num_robots=args.robots,
            task_rate=args.rate,
            duration=args.duration
        )
    elif args.mode == 'algorithms':
        app.run_algorithm_demo()
    elif args.mode == 'test':
        success = app.run_tests()
        sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()