-- SwarmDash Database Schema
-- Demonstrates relational database design and SQL knowledge

-- Table to store robot information and performance metrics
CREATE TABLE IF NOT EXISTS robots (
    robot_id VARCHAR(36) PRIMARY KEY,
    robot_type VARCHAR(20) NOT NULL DEFAULT 'drone',
    max_battery INTEGER NOT NULL DEFAULT 100,
    speed REAL NOT NULL DEFAULT 1.0,
    charging_station_x INTEGER NOT NULL,
    charging_station_y INTEGER NOT NULL,
    creation_time TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    total_tasks_completed INTEGER DEFAULT 0,
    total_distance_traveled REAL DEFAULT 0.0,
    total_uptime_seconds INTEGER DEFAULT 0,
    status VARCHAR(20) DEFAULT 'idle'
);

-- Table to store delivery task information
CREATE TABLE IF NOT EXISTS tasks (
    task_id VARCHAR(36) PRIMARY KEY,
    pickup_x INTEGER NOT NULL,
    pickup_y INTEGER NOT NULL,
    dropoff_x INTEGER NOT NULL,
    dropoff_y INTEGER NOT NULL,
    priority INTEGER NOT NULL DEFAULT 1,
    status VARCHAR(20) DEFAULT 'pending',
    assigned_robot_id VARCHAR(36),
    creation_time TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    assignment_time TIMESTAMP,
    start_time TIMESTAMP,
    completion_time TIMESTAMP,
    estimated_distance REAL,
    actual_distance REAL,
    FOREIGN KEY (assigned_robot_id) REFERENCES robots(robot_id)
);

-- Table to track robot movements and positions over time
CREATE TABLE IF NOT EXISTS robot_movements (
    movement_id INTEGER PRIMARY KEY AUTOINCREMENT,
    robot_id VARCHAR(36) NOT NULL,
    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    position_x INTEGER NOT NULL,
    position_y INTEGER NOT NULL,
    battery_level INTEGER NOT NULL,
    status VARCHAR(20) NOT NULL,
    current_task_id VARCHAR(36),
    FOREIGN KEY (robot_id) REFERENCES robots(robot_id),
    FOREIGN KEY (current_task_id) REFERENCES tasks(task_id)
);

-- Table to store system performance metrics at regular intervals
CREATE TABLE IF NOT EXISTS system_metrics (
    metric_id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    total_robots INTEGER NOT NULL,
    active_robots INTEGER NOT NULL,
    total_tasks INTEGER NOT NULL,
    pending_tasks INTEGER NOT NULL,
    completed_tasks INTEGER NOT NULL,
    average_battery_level REAL NOT NULL,
    system_efficiency REAL NOT NULL,
    grid_utilization REAL NOT NULL
);

-- Table to store optimization algorithm performance
CREATE TABLE IF NOT EXISTS optimization_runs (
    run_id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    algorithm_type VARCHAR(50) NOT NULL,
    execution_time_ms REAL NOT NULL,
    tasks_assigned INTEGER NOT NULL,
    robots_utilized INTEGER NOT NULL,
    optimal_solution BOOLEAN DEFAULT FALSE,
    objective_value REAL,
    constraints_satisfied INTEGER,
    total_constraints INTEGER
);

-- Table to store pathfinding algorithm performance
CREATE TABLE IF NOT EXISTS pathfinding_stats (
    path_id INTEGER PRIMARY KEY AUTOINCREMENT,
    robot_id VARCHAR(36) NOT NULL,
    task_id VARCHAR(36),
    start_x INTEGER NOT NULL,
    start_y INTEGER NOT NULL,
    goal_x INTEGER NOT NULL,
    goal_y INTEGER NOT NULL,
    algorithm_used VARCHAR(20) DEFAULT 'A*',
    path_length INTEGER NOT NULL,
    computation_time_ms REAL NOT NULL,
    nodes_explored INTEGER NOT NULL,
    path_optimal BOOLEAN DEFAULT TRUE,
    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (robot_id) REFERENCES robots(robot_id),
    FOREIGN KEY (task_id) REFERENCES tasks(task_id)
);

-- Indexes for better query performance
CREATE INDEX IF NOT EXISTS idx_robots_status ON robots(status);
CREATE INDEX IF NOT EXISTS idx_tasks_status ON tasks(status);
CREATE INDEX IF NOT EXISTS idx_tasks_assigned_robot ON tasks(assigned_robot_id);
CREATE INDEX IF NOT EXISTS idx_robot_movements_robot_time ON robot_movements(robot_id, timestamp);
CREATE INDEX IF NOT EXISTS idx_system_metrics_timestamp ON system_metrics(timestamp);
CREATE INDEX IF NOT EXISTS idx_optimization_runs_timestamp ON optimization_runs(timestamp);
CREATE INDEX IF NOT EXISTS idx_pathfinding_stats_robot ON pathfinding_stats(robot_id);

-- Views for common analytical queries
CREATE VIEW IF NOT EXISTS robot_performance AS
SELECT 
    r.robot_id,
    r.robot_type,
    r.total_tasks_completed,
    r.total_distance_traveled,
    CASE 
        WHEN r.total_distance_traveled > 0 
        THEN CAST(r.total_tasks_completed AS REAL) / r.total_distance_traveled 
        ELSE 0 
    END as efficiency_ratio,
    COUNT(t.task_id) as current_assigned_tasks,
    AVG(t.actual_distance) as avg_task_distance
FROM robots r
LEFT JOIN tasks t ON r.robot_id = t.assigned_robot_id AND t.status != 'completed'
GROUP BY r.robot_id;

CREATE VIEW IF NOT EXISTS task_completion_stats AS
SELECT 
    DATE(creation_time) as date,
    COUNT(*) as total_tasks,
    SUM(CASE WHEN status = 'completed' THEN 1 ELSE 0 END) as completed_tasks,
    SUM(CASE WHEN status = 'failed' THEN 1 ELSE 0 END) as failed_tasks,
    AVG(CASE 
        WHEN completion_time IS NOT NULL AND start_time IS NOT NULL 
        THEN (julianday(completion_time) - julianday(start_time)) * 24 * 60 
        ELSE NULL 
    END) as avg_completion_time_minutes
FROM tasks
GROUP BY DATE(creation_time)
ORDER BY date DESC;

CREATE VIEW IF NOT EXISTS system_performance_summary AS
SELECT 
    DATE(timestamp) as date,
    AVG(system_efficiency) as avg_efficiency,
    AVG(grid_utilization) as avg_utilization,
    AVG(average_battery_level) as avg_battery,
    MAX(total_robots) as peak_robots,
    MAX(total_tasks) as peak_tasks
FROM system_metrics
GROUP BY DATE(timestamp)
ORDER BY date DESC;