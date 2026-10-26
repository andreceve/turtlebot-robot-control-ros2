#!/usr/bin/env python3
"""Task 2: complete the ICP steps in registration().

The supplied parameters were tested with the exercise world's 5 Hz lidar,
straight driving at 0.16 m/s and slow turns at 0.08 rad/s. Process every 12th
scan to give this point-to-point ICP sufficient separation between scans.
Revisit scan_stride if the scan rate or driving speed changes.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from scipy.spatial import KDTree
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import TransformStamped
import tf2_ros
import open3d as o3d
import threading
import copy
import time


class ScanMapperNode(Node):
    def __init__(self):
        super().__init__('scan_mapper')
        
        # Parameters
        self.map_resolution = 0.025  # meters per pixel
        self.map_width = 1040  # pixels
        self.map_height = 640  # pixels
        self.map_origin_x = -6.0  # meters, relative to the initial scan frame
        self.map_origin_y = -4.0  # meters
        self.max_scan_range = 30.0  # max range to consider for scan points
        self.icp_max_iterations = 60
        self.icp_distance_threshold = 0.4  # meters
        self.is_first_scan = True

        # Process scans 0, 12, 24, ... (about 2.4 seconds apart at 5 Hz).
        # The map publishing timer remains independent of scan selection.
        self.scan_stride = 12
        self.scan_count = 0
        
        # Registration optimization parameters
        # Scan stride provides the larger separation; these small keyframe thresholds
        # suppress negligible estimated motion in the selected scans.
        self.keyframe_trans_threshold = 0.001  # metres
        self.keyframe_rot_threshold = 0.001    # radians
        # Registration-quality gate: reject a scan match with poor overlap / high error.
        self.min_fitness = 0.35                # fraction of current points with a close match
        self.max_rmse = 0.15                   # metres, RMSE of the inlier correspondences
        # ICP convergence thresholds (per-iteration step size): stop iterating once a
        # single ICP step moves/rotates the cloud by less than these amounts.
        self.convergence_translation_threshold = 0.0001  # metres
        self.convergence_rotation_threshold = 0.0001     # radians
        self.max_processing_time = 0.4  # Max time for registration (seconds) for 2Hz
        
        # Probabilistic mapping parameters
        self.prob_occ = 0.75  # Probability that cell is occupied when hit by laser
        self.prob_free = 0.40  # Probability that cell is free when laser passes through
        self.prob_prior = 0.5  # Prior probability (unknown)
        
        # Convert probabilities to log-odds for efficient computation
        self.logodds_occ = self.prob_to_logodds(self.prob_occ)
        self.logodds_free = self.prob_to_logodds(self.prob_free)
        self.logodds_prior = self.prob_to_logodds(self.prob_prior)
        
        # Clamping values to prevent overflow
        self.logodds_min = self.prob_to_logodds(0.001)  # Very confident free
        self.logodds_max = self.prob_to_logodds(0.999)  # Very confident occupied
        
        # Robot pose (x, y, theta)
        self.robot_pose = np.array([0.0, 0.0, 0.0])
        
        # Initialize log-odds map (0 = log-odds of prior probability)
        self.logodds_map = np.ones((self.map_height, self.map_width), dtype=np.float32) * self.logodds_prior
        
        # Store previous scan for registration
        self.prev_scan_points = None
        
        # Create publishers and subscribers
        # QoS profile for scan subscription
        scan_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # QoS profile for map publication (compatible with nav2 map_saver)
        from rclpy.qos import QoSDurabilityPolicy
        map_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  # This is what nav2 expects
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            scan_qos_profile
        )
        
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/map',
            map_qos_profile
        )
        
        # Initialize TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Create a timer for publishing the map periodically at 2Hz.
        # (Publishing shares the single-threaded executor with scan processing, so we keep
        # this modest; the map still fills smoothly because scans are no longer blocked.)
        self.map_timer = self.create_timer(0.5, self.publish_map)
        
        self.get_logger().info('Probabilistic Scan Mapper Node initialized')
        self.get_logger().info(f'Log-odds - Occupied: {self.logodds_occ:.3f}, Free: {self.logodds_free:.3f}, Prior: {self.logodds_prior:.3f}')

    def prob_to_logodds(self, prob):
        """Convert probability to log-odds"""
        return math.log(prob / (1.0 - prob))
    
    def logodds_to_prob(self, logodds):
        """Convert log-odds to probability"""
        return 1.0 - 1.0 / (1.0 + math.exp(logodds))
    
    def logodds_to_occupancy_grid_value(self, logodds):
        """Convert log-odds to occupancy grid value (0-100, -1 for unknown)"""
        prob = self.logodds_to_prob(logodds)
        if abs(logodds - self.logodds_prior) < 0.1:  # Close to prior = unknown
            return -1
        else:
            return int(prob * 100)

    def scan_callback(self, msg):
        """Callback function for processing incoming laser scan messages"""
        scan_index = self.scan_count
        self.scan_count += 1
        if scan_index % self.scan_stride != 0:
            return

        start_time = time.time()
        
        # Convert scan to Cartesian coordinates
        current_scan_points = self.polar_to_cartesian(msg)
        
        if current_scan_points.size == 0:
            self.get_logger().warn('No valid scan points received')
            return
        
        if self.is_first_scan:
            self.get_logger().info(f'Processing first scan with {len(current_scan_points)} points')
            self.prev_scan_points = current_scan_points
            self.is_first_scan = False
            
            # Update map with first scan
            self.update_map_probabilistic(current_scan_points, self.robot_pose)
            self.broadcast_transform()
            
            return
        
        # Perform scan registration to find transformation between scans
        transform_matrix = self.registration(self.prev_scan_points, current_scan_points)
        
        # Extract translation and rotation from transformation matrix
        delta_x = transform_matrix[0, 2]
        delta_y = transform_matrix[1, 2]
        delta_theta = math.atan2(transform_matrix[1, 0], transform_matrix[0, 0])
        
        movement_magnitude = math.sqrt(delta_x**2 + delta_y**2)

        # Registration-quality gate: measure how well the estimated transform aligns the
        # current scan onto the reference scan, and reject poor matches (e.g. low overlap
        # during fast turns) so they do not corrupt the pose and the map.
        fitness, rmse = self.registration_fitness(self.prev_scan_points, current_scan_points, transform_matrix)
        good_match = (fitness >= self.min_fitness) and (rmse <= self.max_rmse)

        # Keyframe gate: only integrate motion and advance the reference scan once the robot
        # has actually moved. Between keyframes the pose is held fixed, so per-scan ICP noise
        # is not accumulated into the trajectory (this is what keeps the map from smearing).
        is_keyframe = (movement_magnitude > self.keyframe_trans_threshold or
                       abs(delta_theta) > self.keyframe_rot_threshold)

        if is_keyframe and good_match:
            self.get_logger().info(
                f'Keyframe: dx={delta_x:.3f}, dy={delta_y:.3f}, dtheta={delta_theta:.3f} '
                f'(fitness={fitness:.2f}, rmse={rmse:.3f})')
            # Update robot pose with the estimated motion
            self.update_pose(delta_x, delta_y, delta_theta)
            # Integrate the new scan into the map at the updated pose
            self.update_map_probabilistic(current_scan_points, self.robot_pose)
            # Advance the reference scan ONLY on a good keyframe (limits drift accumulation)
            self.prev_scan_points = current_scan_points
        elif is_keyframe and not good_match:
            # Enough motion but a poor match: skip integrating and keep the current reference
            self.get_logger().warn(
                f'Rejected low-quality registration (fitness={fitness:.2f}, rmse={rmse:.3f})')
        # else: sub-keyframe motion -> hold pose and keep reference (nothing integrated)

        # Broadcast the transform
        self.broadcast_transform()
        
        # Log processing time for 2Hz monitoring
        processing_time = time.time() - start_time
        if processing_time > self.max_processing_time:
            self.get_logger().warn(f'Processing took {processing_time:.3f}s (target: {self.max_processing_time:.3f}s for 2Hz)')
        else:
            self.get_logger().debug(f'Processing completed in {processing_time:.3f}s')

    def polar_to_cartesian(self, scan_msg):
        """Convert laser scan from polar to Cartesian coordinates"""
        ranges = np.array(scan_msg.ranges)
        
        # Make sure the angles array has the same length as ranges
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))
        
        # Filter out invalid measurements
        valid_indices = np.isfinite(ranges) & (ranges > scan_msg.range_min) & (ranges < min(scan_msg.range_max, self.max_scan_range))
        valid_ranges = ranges[valid_indices]
        valid_angles = angles[valid_indices]
        
        # Convert to Cartesian coordinates
        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)
        
        return np.column_stack((x, y))

    def registration(self, prev_scan, current_scan):
        """
        Perform scan registration using ICP 

        Args:
        prev_scan: Nx2 numpy array (previous scan)
        current_scan: Mx2 numpy array (current scan)
        self.icp_max_iterations: Maximum number of ICP iterations
        self.convergence_translation_threshold: stop when a step's translation is below this (metres)
        self.convergence_rotation_threshold: stop when a step's rotation is below this (radians)

        Returns:
        3x3 numpy array - homogeneous transformation matrix (from current -> prev)
        """
        # Initialize transformation matrix (3x3 homogeneous)
        T = np.eye(3)
        # Build KD-tree for efficient nearest neighbor search
        tree = KDTree(prev_scan)
        for iteration in range(self.icp_max_iterations):
            # Find nearest neighbors
            distances, indices = tree.query(current_scan)

            # Filter out correspondences that are too far apart
            valid_correspondences = distances < self.icp_distance_threshold
            if np.sum(valid_correspondences) < 3:  # Need at least 3 points
                self.get_logger().warn(f'Too few valid correspondences: {np.sum(valid_correspondences)}')
                break
            current_valid = current_scan[valid_correspondences]
            matched_prev = prev_scan[indices[valid_correspondences]]

            # current_valid is the current scan with valid points (Nx2 numpy array).
            # matched_prev is the previous scan with matched valid points (Nx2 numpy array).

            # You may use (prev_scan, current_scan) or (current_valid, matched_prev);
            # the latter have slightly better performance.
            # ----------------------------------------------------------------

            # YOUR CODE STARTS HERE
            # Step 1 & 2: Calculate centroids and center the point clouds
            centroid_current = np.mean(current_valid, axis=0)
            centroid_prev = np.mean(matched_prev, axis=0)

            current_centered = current_valid - centroid_current
            prev_centered = matched_prev - centroid_prev

            # Step 3: Compute the covariance matrix H
            H = current_centered.T @ prev_centered

            # Step 4: Generate SVD of the covariance matrix
            U, S, Vt = np.linalg.svd(H)

            # Step 5 & 6: Compute optimal rotation matrix ensuring no reflection (det = +1)
            R = Vt.T @ U.T
            if np.linalg.det(R) < 0:
                Vt[1, :] *= -1
                R = Vt.T @ U.T

            # Step 7: Compute optimal translation vector
            t = centroid_prev.T - R @ centroid_current.T

            # Step 8: Construct homogeneous transformation matrix for this iteration
            T_iter = np.eye(3)
            T_iter[:2, :2] = R
            T_iter[:2, 2] = t
            
            # Accumulate the total transformation matrix
            T = T_iter @ T

            # Apply transformation to current points for the next iteration loop
            current_scan_homogeneous = np.hstack((current_scan, np.ones((current_scan.shape[0], 1))))
            current_scan = (T_iter @ current_scan_homogeneous.T).T[:, :2]

            # Check convergence thresholds
            translation_magnitude = np.linalg.norm(t)
            rotation_angle = abs(math.atan2(R[1, 0], R[0, 0]))

            if translation_magnitude < self.convergence_translation_threshold and rotation_angle < self.convergence_rotation_threshold:
                break
            # ----------------------------------------------------------------
             
        return T

    def registration_fitness(self, prev_scan, current_scan, transform_matrix):
        """Self-contained quality metric for a scan match (no Open3D needed).

        Applies the estimated transform to the current scan and measures how well it
        overlaps the reference (previous) scan.

        Returns:
            fitness: fraction of current points with a match closer than
                     self.icp_distance_threshold (higher is better, range 0..1)
            rmse:    RMSE of those inlier correspondences in metres (lower is better)
        """
        R = transform_matrix[:2, :2]
        t = transform_matrix[:2, 2]
        moved = (R @ current_scan.T).T + t
        tree = KDTree(prev_scan)
        distances, _ = tree.query(moved)
        inliers = distances < self.icp_distance_threshold
        if not np.any(inliers):
            return 0.0, float('inf')
        fitness = float(np.mean(inliers))
        rmse = float(np.sqrt(np.mean(distances[inliers] ** 2)))
        return fitness, rmse

    def evaluate_registration_with_open3d(self, prev_scan, current_scan, transform_matrix):
        """
        Evaluate registration quality using Open3D ICP as ground truth
        
        Args:
            prev_scan: Points from previous scan in Cartesian coordinates (Nx2 array)
            current_scan: Points from current scan in Cartesian coordinates (Mx2 array)
            transform_matrix: 3x3 transformation matrix from student's registration
            
        Returns:
            dict: Registration quality metrics
        """
        # Convert 2D points to 3D for Open3D (z=0)
        prev_points_3d = np.column_stack([prev_scan, np.zeros(len(prev_scan))])
        current_points_3d = np.column_stack([current_scan, np.zeros(len(current_scan))])
        
        # Create Open3D point clouds
        source = o3d.geometry.PointCloud()
        target = o3d.geometry.PointCloud()
        source.points = o3d.utility.Vector3dVector(current_points_3d)
        target.points = o3d.utility.Vector3dVector(prev_points_3d)
        
        # Convert 2D transformation to 3D
        transform_3d = np.eye(4)
        transform_3d[:2, :2] = transform_matrix[:2, :2]
        transform_3d[:2, 3] = transform_matrix[:2, 2]
        
        # Perform Open3D ICP for ground truth
        threshold = self.icp_distance_threshold
        reg_p2p = o3d.pipelines.registration.registration_icp(
            source, target, threshold, np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=self.icp_max_iterations)
        )
        
        # Calculate metrics
        ground_truth_transform = reg_p2p.transformation
        
        # Translation error
        student_translation = transform_3d[:3, 3]
        ground_truth_translation = ground_truth_transform[:3, 3]
        translation_error = np.linalg.norm(student_translation - ground_truth_translation)
        
        # Rotation error (angle difference)
        student_rotation = transform_3d[:3, :3]
        ground_truth_rotation = ground_truth_transform[:3, :3]
        rotation_diff = np.linalg.inv(student_rotation) @ ground_truth_rotation
        rotation_error = abs(math.atan2(rotation_diff[1, 0], rotation_diff[0, 0]))
        
        # Fitness score (overlap percentage)
        fitness_score = reg_p2p.fitness
        
        # RMSE
        rmse = reg_p2p.inlier_rmse
        
        # Log evaluation results
        self.get_logger().info(f'Registration Evaluation:')
        self.get_logger().info(f'  Translation Error: {translation_error:.4f} m')
        self.get_logger().info(f'  Rotation Error: {rotation_error:.4f} rad ({math.degrees(rotation_error):.2f} deg)')
        self.get_logger().info(f'  Fitness Score: {fitness_score:.4f}')
        self.get_logger().info(f'  RMSE: {rmse:.4f}')
        
        return {
            'translation_error': translation_error,
            'rotation_error': rotation_error,
            'fitness_score': fitness_score,
            'rmse': rmse,
            'ground_truth_transform': ground_truth_transform,
            'student_transform': transform_3d
        }

    def update_pose(self, delta_x, delta_y, delta_theta):
        """Update the robot pose based on the estimated transformation"""
        self.get_logger().debug(f'Updating pose with deltas: dx={delta_x:.3f}, dy={delta_y:.3f}, dtheta={delta_theta:.3f}')
        
        # Transform deltas from robot frame to global frame
        cos_theta = math.cos(self.robot_pose[2])
        sin_theta = math.sin(self.robot_pose[2])
        
        global_delta_x = cos_theta * delta_x - sin_theta * delta_y
        global_delta_y = sin_theta * delta_x + cos_theta * delta_y
        
        # Update pose in global frame
        self.robot_pose[0] += global_delta_x
        self.robot_pose[1] += global_delta_y
        self.robot_pose[2] += delta_theta
        
        # Normalize angle
        self.robot_pose[2] = (self.robot_pose[2] + math.pi) % (2 * math.pi) - math.pi
        
        self.get_logger().info(f'Updated robot pose: x={self.robot_pose[0]:.3f}, y={self.robot_pose[1]:.3f}, theta={self.robot_pose[2]:.3f}')

    def update_map_probabilistic(self, scan_points, robot_pose):
        """Update the global map with new scan points using probabilistic occupancy mapping"""
        self.get_logger().debug(f'Updating map probabilistically with {len(scan_points)} points')
        
        # Calculate robot position in grid coordinates
        robot_grid_x = int((robot_pose[0] - self.map_origin_x) / self.map_resolution)
        robot_grid_y = int((robot_pose[1] - self.map_origin_y) / self.map_resolution)
        
        # Ensure robot is within map bounds
        if (robot_grid_x < 0 or robot_grid_x >= self.map_width or 
            robot_grid_y < 0 or robot_grid_y >= self.map_height):
            self.get_logger().warn(f'Robot position outside map bounds: ({robot_grid_x}, {robot_grid_y})')
            return
        
        # Transform scan points to global frame
        cos_theta = math.cos(robot_pose[2])
        sin_theta = math.sin(robot_pose[2])
        
        transformed_points = np.zeros_like(scan_points)
        transformed_points[:, 0] = cos_theta * scan_points[:, 0] - sin_theta * scan_points[:, 1] + robot_pose[0]
        transformed_points[:, 1] = sin_theta * scan_points[:, 0] + cos_theta * scan_points[:, 1] + robot_pose[1]
        
        # Convert to grid coordinates
        grid_points = np.zeros_like(transformed_points, dtype=int)
        grid_points[:, 0] = np.round((transformed_points[:, 0] - self.map_origin_x) / self.map_resolution).astype(int)
        grid_points[:, 1] = np.round((transformed_points[:, 1] - self.map_origin_y) / self.map_resolution).astype(int)
        
        # Filter points outside map boundaries
        valid_indices = (grid_points[:, 0] >= 0) & (grid_points[:, 0] < self.map_width) & \
                        (grid_points[:, 1] >= 0) & (grid_points[:, 1] < self.map_height)
        valid_grid_points = grid_points[valid_indices]
        
        # Update log-odds for each laser beam
        occupied_updates = 0
        free_updates = 0
        
        for point in valid_grid_points:
            # Update occupied cell (endpoint of laser beam)
            self.update_cell_logodds(point[0], point[1], self.logodds_occ)
            occupied_updates += 1
            
            # Update free space along the ray from robot to obstacle
            free_cells = self.get_ray_cells(robot_grid_x, robot_grid_y, point[0], point[1])
            for cell_x, cell_y in free_cells:
                self.update_cell_logodds(cell_x, cell_y, self.logodds_free)
                free_updates += 1
        
        self.get_logger().debug(f'Updated {occupied_updates} cells as occupied, {free_updates} cells as free')

    def update_cell_logodds(self, x, y, logodds_update):
        """Update a single cell's log-odds value"""
        if 0 <= x < self.map_width and 0 <= y < self.map_height:
            # Bayesian update: posterior = prior + measurement - prior_assumption
            self.logodds_map[y, x] += logodds_update - self.logodds_prior
            
            # Clamp to prevent overflow
            self.logodds_map[y, x] = np.clip(self.logodds_map[y, x], self.logodds_min, self.logodds_max)

    def get_ray_cells(self, x0, y0, x1, y1):
        """Get all cells along a ray using Bresenham's line algorithm, excluding endpoints"""
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            # Don't include the starting point (robot) or ending point (obstacle)
            if not ((x == x0 and y == y0) or (x == x1 and y == y1)):
                if 0 <= x < self.map_width and 0 <= y < self.map_height:
                    cells.append((x, y))
            
            if x == x1 and y == y1:
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        
        return cells

    def publish_map(self):
        """Publish the global map as an OccupancyGrid message"""
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_width
        map_msg.info.height = self.map_height
        map_msg.info.origin.position.x = self.map_origin_x
        map_msg.info.origin.position.y = self.map_origin_y
        
        # Convert the whole log-odds map to occupancy values in one vectorized pass.
        # (A per-cell Python double loop over 1,000,000 cells took ~seconds and blocked
        # scan processing, which made the map appear to update very slowly.)
        prob = 1.0 - 1.0 / (1.0 + np.exp(self.logodds_map))         # log-odds -> probability
        occupancy_grid = np.full(self.logodds_map.shape, -1, dtype=np.int8)   # default: unknown
        known = np.abs(self.logodds_map - self.logodds_prior) >= 0.1
        occupancy_grid[known] = (prob[known] * 100).astype(np.int8)

        # Flatten the map and convert to list
        map_msg.data = occupancy_grid.flatten().tolist()
        
        self.map_pub.publish(map_msg)
        
        # Count occupied, free, and unknown cells
        occupied_cells = np.sum(occupancy_grid >= 65)  # High confidence occupied
        free_cells = np.sum(occupancy_grid <= 25)      # High confidence free
        unknown_cells = np.sum(occupancy_grid == -1)   # Unknown
        uncertain_cells = np.sum((occupancy_grid > 25) & (occupancy_grid < 65) & (occupancy_grid != -1))  # Uncertain
        
        self.get_logger().info(f'Map stats - Occupied: {occupied_cells}, Free: {free_cells}, Unknown: {unknown_cells}, Uncertain: {uncertain_cells}')
        self.get_logger().info(f'Current robot pose: x={self.robot_pose[0]:.3f}, y={self.robot_pose[1]:.3f}, theta={self.robot_pose[2]:.3f}')

    def broadcast_transform(self):
        """Broadcast the transform from map to base_link"""
        t = TransformStamped()
        
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.robot_pose[0]
        t.transform.translation.y = self.robot_pose[1]
        t.transform.translation.z = 0.0
        
        # Convert yaw to quaternion
        yaw = self.robot_pose[2]
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ScanMapperNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
