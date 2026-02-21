import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Int32
import numpy as np
import math
import subprocess
import json
from std_msgs.msg import String



class YAxisTrackerAndCleaner(Node):
    def __init__(self):
        super().__init__('y_axis_tracker')

        self.OFFSET_MEASURE = 0.6
        self.OFFSET_CLEAN   = 0.6
        self.SAFETY_MARGIN  = 0.5 

        self.WASTE_DETECT_RANGE = 4.5     
        self.WASTE_MERGE_DIST = 0.8 
        self.waste_registry = {} # <--- ADD THIS
        self.load_waste_map()        
        self.WASTE_ANGLE_WINDOW = math.radians(90) 
        
        self.cmd_pub = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        self.dim_pub = self.create_publisher(Float32, '/map/width', 10)
        self.length_sub = self.create_subscription(Float32, '/map/length', self.length_callback, 10)
        
        self.scan_sub = self.create_subscription(LaserScan, '/robot1/scan', self.lidar_callback, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(Odometry, '/robot1/odom1', self.odom_callback, qos_profile_sensor_data)

        self.robot_count_sub = self.create_subscription(Int32, '/swarm_count', self.robot_count_callback, 10)
        self.robot_count = 1 
        self.robot_id = "robot1" # Y-Axis tracker is robot 1
        self.transfer_pub = self.create_publisher(String, '/swarm/waste_transfer', 10)
        self.transfer_sub = self.create_subscription(String, '/swarm/waste_transfer', self.transfer_callback, 10)
        # === SWARM SYNC ADDITIONS ===
        
        # New variables for target claiming
        self.claimed_wastes = set() 
        self.current_target_name = None 
        
        self.claim_pub = self.create_publisher(String, '/swarm/claimed_waste', 10)
        self.claim_sub = self.create_subscription(String, '/swarm/claimed_waste', self.claim_callback, 10)

        self.state = "MEASURING"
        self.map_width = None
        self.map_length = None
        
        self.start_x, self.start_y, self.start_yaw = None, None, None
        self.current_x = -self.OFFSET_CLEAN 
        self.current_y = self.OFFSET_MEASURE
        self.current_yaw = 0.0
        self.world_x = 0.0
        self.world_y = 0.0
        self.world_yaw = 0.0
        
        self.clean_step = 0
        self.fixed_axis_value = 0.0
        self.step_start_pos = 0.0
        self.partition_max_x = 100.0
        
        self.backup_start_pos = None
        self.wait_start_time = None
        self.front_idx = None
        self.left_idx = None
        self.current_front_dist = 10.0 

        self.is_turning = False
        self.approaching_waste = False
        self.current_waste_distance = None
        self.detected_wastes = [] 
        self.detected_wastes = [] 
        self.deleted_names = set()

        self.get_logger().info("Robot 1 Ready: Measuring Width...")
        # self.test_timer = self.create_timer(10.0, self.test_deletion_callback)
        # self.get_logger().info("TEST: Scheduled deletion of 'waste_2' in 10 seconds...")
    def test_deletion_callback(self):
        # We only want to run this once
        self.test_timer.cancel()
        self.get_logger().info("TEST: Attempting to delete 'waste_2' now!")
        self.delete_waste_visual("waste_2")
    def load_waste_map(self):
        try:
            # Give it a second to ensure the file is written
            import time
            time.sleep(1.0) 
            with open("waste_locations.csv", "r") as f:
                for line in f:
                    name, x, y = line.strip().split(',')
                    self.waste_registry[(float(x), float(y))] = name
        except Exception as e:
            self.get_logger().error(f"Could not load map: {e}")

    def robot_count_callback(self, msg):
        if msg.data > self.robot_count:
            self.robot_count = msg.data
            self.get_logger().info(f"Updated Swarm Size: {self.robot_count} robots")
            if self.map_length is not None:
                self.partition_max_x = self.map_length / float(self.robot_count)
                self.get_logger().info(f"Partition Updated Live: X Limit = {self.partition_max_x:.2f}")

    def length_callback(self, msg):
        self.map_length = msg.data
        self.check_start_cleaning()

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.world_x = msg.pose.pose.position.x
        self.world_y = msg.pose.pose.position.y
        self.world_yaw = yaw

        if self.start_x is None:
            self.start_x = self.world_x 
            self.start_y = self.world_y
            self.start_yaw = yaw

        dx = self.world_x - self.start_x
        dy = self.world_y - self.start_y

        self.current_y = (dx * math.cos(self.start_yaw) + dy * math.sin(self.start_yaw)) + self.OFFSET_MEASURE
        self.current_x = (-dx * math.sin(self.start_yaw) + dy * math.cos(self.start_yaw)) - self.OFFSET_CLEAN
        self.current_yaw = yaw - self.start_yaw

    def compute_indices(self, scan):
        angle_min = scan.angle_min
        inc = scan.angle_increment
        def a2i(a): return int((a - angle_min) / inc)
        self.front_idx = max(0, min(len(scan.ranges)-1, a2i(0.0)))
        self.left_idx = max(0, min(len(scan.ranges)-1, a2i(math.pi/2)))

    def turn_in_place(self, target_yaw):
        cmd = Twist()
        cyaw = math.atan2(math.sin(self.current_yaw), math.cos(self.current_yaw))
        
        # NOTE: For Global navigation (COLLECTING), we use world_yaw
        if self.state == "COLLECTING":
            cyaw = self.world_yaw

        diff = target_yaw - cyaw
        diff = math.atan2(math.sin(diff), math.cos(diff))
        
        if abs(diff) > 0.05:
            self.is_turning = True
            cmd.angular.z = max(-0.5, min(0.5, 1.5 * diff))
            self.cmd_pub.publish(cmd); return False
        else:
            self.is_turning = False; self.cmd_pub.publish(Twist()); return True

    def move_backward(self, distance):
        if self.backup_start_pos is None:
            self.backup_start_pos = (self.current_x, self.current_y)
            return False
        dx = self.current_x - self.backup_start_pos[0]
        dy = self.current_y - self.backup_start_pos[1]
        dist_moved = math.sqrt(dx*dx + dy*dy)
        if dist_moved < distance:
            cmd = Twist(); cmd.linear.x = -0.2; self.cmd_pub.publish(cmd); return False
        else:
            self.cmd_pub.publish(Twist()); self.backup_start_pos = None; return True

    def wait_in_place(self, duration):
        if self.wait_start_time is None:
            self.wait_start_time = self.get_clock().now()
            self.cmd_pub.publish(Twist())
            return False
        elapsed = (self.get_clock().now() - self.wait_start_time).nanoseconds / 1e9
        if elapsed < duration: self.cmd_pub.publish(Twist()); return False
        else: self.wait_start_time = None; return True

    def move_straight_locked(self, target_yaw, axis_to_hold, target_val):
        cmd = Twist()
        if self.current_front_dist < 0.6:
            cmd.linear.x = 0.1 # Very slow for final approach
        elif self.current_front_dist < 1.5:
            cmd.linear.x = 0.2
        else:
            cmd.linear.x = 0.8

        cyaw = math.atan2(math.sin(self.current_yaw), math.cos(self.current_yaw))
        yaw_err = target_yaw - cyaw
        yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))
        pos_err = 0.0
        if axis_to_hold == 'x': pos_err = target_val - self.current_x
        elif axis_to_hold == 'y': pos_err = target_val - self.current_y
        K_YAW = 2.0; K_POS = 1.0
        
        correction = K_YAW * yaw_err
        if abs(target_yaw - 0.0) < 0.1: correction += K_POS * pos_err
        elif abs(target_yaw - math.pi) < 0.1 or abs(target_yaw + math.pi) < 0.1: correction -= K_POS * pos_err
        elif abs(target_yaw + math.pi/2) < 0.1: correction += K_POS * pos_err
        elif abs(target_yaw - math.pi/2) < 0.1: correction -= K_POS * pos_err
        cmd.angular.z = max(-0.5, min(0.5, correction))
        self.cmd_pub.publish(cmd)

    #====COMMUNICATION LOGIC=====


    def request_more_work(self):
        self.stop_robot()
        self.state = "REQUESTING_WORK"
        self.get_logger().info("My queue is empty. Requesting waste list from the swarm...")
        msg = String()
        # Broadcast a request for work
        msg.data = json.dumps({"action": "request", "sender": self.robot_id})
        self.transfer_pub.publish(msg)

    def transfer_callback(self, msg):
        data = json.loads(msg.data)
        
        # Ignore our own messages
        if data["sender"] == self.robot_id:
            return 
            
        if data["action"] == "request":
            # The other bot wants work. Share our list.
            reply = {
                "action": "share",
                "sender": self.robot_id,
                "wastes": self.detected_wastes,
                "deleted": list(self.deleted_names)
            }
            out_msg = String()
            out_msg.data = json.dumps(reply)
            self.transfer_pub.publish(out_msg)
            
        elif data["action"] == "share" and self.state == "REQUESTING_WORK":
            # We asked for work, and the other bot sent us its list
            for name in data["deleted"]:
                self.deleted_names.add(name)
                
            for w in data["wastes"]:
                if w["name"] not in self.deleted_names and not any(dw["name"] == w["name"] for dw in self.detected_wastes):
                    self.detected_wastes.append({'pos': (w['pos'][0], w['pos'][1]), 'name': w['name']})
                    
            # === THE FIX: Only resume if there is UNCLAIMED work available ===
            available_wastes = [w for w in self.detected_wastes if w['name'] not in self.claimed_wastes and w['name'] not in self.deleted_names]
            
            if available_wastes:
                waste_names = [w['name'] for w in available_wastes]
                self.get_logger().info(f"Received new tasks! AVAILABLE TARGETS: {waste_names}")
                self.state = "COLLECTING"
            else:
                self.get_logger().info("All remaining tasks are claimed by other bots. Swarm tasks complete. FINISHED.")
                self.state = "FINISHED"
                self.stop_robot()

    #===For claiming the waste when both attempts to collect the same waste===
    
    def claim_callback(self, msg):
        data = json.loads(msg.data)
        
        # Ignore our own messages
        if data["sender"] != self.robot_id:
            # Register that the other bot has claimed this waste
            self.claimed_wastes.add(data["waste_name"])
            
            # If we are currently heading for this exact piece of waste, we need to resolve the conflict
            if self.current_target_name == data["waste_name"]:
                # Tie-breaker: The robot with the alphabetically "lower" ID wins (robot1 beats robot2)
                if self.robot_id > data["sender"]:
                    self.get_logger().info(f"Conflict on {data['waste_name']}. Yielding to {data['sender']}...")
                    self.current_target_name = None # Drop the target so we pick a new one next loop
                    self.stop_robot()
                else:
                    self.get_logger().info(f"Conflict on {data['waste_name']}. I have priority, keeping target.")


    # === NEW LOGIC ===

    def trigger_rotation_phase(self):
        self.stop_robot()
        if not self.detected_wastes:
            self.request_more_work()
        else:
            # Extract just the names for a clean terminal output
            waste_names = [w['name'] for w in self.detected_wastes]
            self.get_logger().info(f"CURRENT TARGET LIST: {waste_names}")
            self.get_logger().info(f"Partition Reached. Starting Collection of {len(self.detected_wastes)} items.")
            self.state = "COLLECTING"

    def navigate_to_waste(self):
        # 1. Filter out wastes that have been deleted or claimed by other bots
        available_wastes = [w for w in self.detected_wastes if w['name'] not in self.claimed_wastes and w['name'] not in self.deleted_names]

        if not available_wastes:
            self.request_more_work()
            return

        # 2. If we don't have an active target, pick the closest available one
        if self.current_target_name is None:
            closest_idx = 0
            min_dist = float('inf')
            for i, w in enumerate(available_wastes):
                d = math.hypot(w['pos'][0] - self.world_x, w['pos'][1] - self.world_y)
                if d < min_dist:
                    min_dist = d
                    closest_idx = i

            target_dict = available_wastes[closest_idx]
            self.current_target_name = target_dict['name']
            
            # Broadcast our claim to the swarm
            claim_msg = String()
            claim_msg.data = json.dumps({"sender": self.robot_id, "waste_name": self.current_target_name})
            self.claim_pub.publish(claim_msg)
            self.get_logger().info(f"Claimed target: {self.current_target_name}")

        # 3. Retrieve our locked target's coordinates
        target_dict = next((w for w in available_wastes if w['name'] == self.current_target_name), None)
        
        # Safety check: If our target vanished (e.g., deleted by the other bot), reset and try again
        if target_dict is None:
            self.current_target_name = None
            return

        target_pos = target_dict['pos']
        
        # 4. Standard Navigation Math
        dx = target_pos[0] - self.world_x
        dy = target_pos[1] - self.world_y
        dist = math.sqrt(dx*dx + dy*dy)
        
        target_angle = math.atan2(dy, dx)
        yaw_err = math.atan2(math.sin(target_angle - self.world_yaw), math.cos(target_angle - self.world_yaw))

        cmd = Twist()

        if abs(yaw_err) > 0.2:
            cmd.angular.z = max(-0.6, min(0.6, 1.5 * yaw_err))
        elif dist > 0.7: 
            cmd.linear.x = 0.2
            cmd.angular.z = 1.0 * yaw_err 
        else:
            # 5. Reached Target! Delete and Cleanup
            self.stop_robot()
            self.get_logger().info(f"ACTION: Deleting {target_dict['name']} at dist {dist:.2f}")
            
            self.deleted_names.add(target_dict['name'])
            self.delete_waste_visual(target_dict['name'])
            
            # Remove from our local lists
            self.detected_wastes = [w for w in self.detected_wastes if w['name'] != target_dict['name']]
            self.current_target_name = None # Clear the lock so we can pick a new target next loop
            
            waste_names = [w['name'] for w in self.detected_wastes]
            self.get_logger().info(f"Item collected. REMAINING TARGET LIST: {waste_names}")

            # Check if that was the last available item
            remaining = [w for w in self.detected_wastes if w['name'] not in self.claimed_wastes]
            if not remaining:
                self.request_more_work()
            return 

        self.cmd_pub.publish(cmd)

    def scan_for_waste(self, scan, ranges):
        half_window = int(self.WASTE_ANGLE_WINDOW / scan.angle_increment)
        start = max(0, self.front_idx - half_window)
        end   = min(len(ranges), self.front_idx + half_window)
        
        for i in range(start, end):
            dist = ranges[i]
            if not math.isfinite(dist): continue
            
            if 0.3 < dist < self.WASTE_DETECT_RANGE:
                angle = scan.angle_min + i * scan.angle_increment
                
                # Global Calc                     
                global_angle = self.world_yaw + angle
                waste_x = self.world_x + dist * math.cos(global_angle)
                waste_y = self.world_y + dist * math.sin(global_angle)
                                                                           
                if abs(waste_x) > (self.partition_max_x - 1.0): 
                    continue 

                if self.map_width is not None:
                    if waste_y < 0.8 or waste_y > (self.map_width - 0.8):
                        continue

                
                
                is_new = True
                for waste_entry in self.detected_wastes:
                    # We must extract the tuple (ex, ey) from the dictionary first
                    ex, ey = waste_entry['pos'] 
                    if math.hypot(waste_x - ex, waste_y - ey) < self.WASTE_MERGE_DIST:
                        is_new = False
                        break
                
                if is_new:
                    rounded = (round(waste_x, 2), round(waste_y, 2))
                    real_name = "unknown"
                    min_match_dist = 1.0 
                    for (rx, ry), name in self.waste_registry.items():
                        d = math.hypot(waste_x - rx, waste_y - ry)
                        if d < min_match_dist:
                            real_name = name
                            min_match_dist = d

                    # if real_name != "unknown":
                    #     # Check if this name is already in the list to avoid duplicates
                    #     if not any(w['name'] == real_name for w in self.detected_wastes):
                    #         self.detected_wastes.append({'pos': (waste_x, waste_y), 'name': real_name})
                    #         self.get_logger().info(f"REGISTERED: {real_name}")
                    if real_name != "unknown":
                        # UPDATE THIS IF STATEMENT
                        if real_name not in self.deleted_names and not any(w['name'] == real_name for w in self.detected_wastes):
                            self.detected_wastes.append({'pos': (waste_x, waste_y), 'name': real_name})
                            self.get_logger().info(f"REGISTERED: {real_name}")
                
                    # THIS BREAK MUST BE AT THIS INDENTATION LEVEL (line 284 in your image)
                    break 
    # =================

    def run_cleaning_fsm(self):
        if self.clean_step == 0:
            if self.turn_in_place(-math.pi/2): self.fixed_axis_value = self.current_y; self.step_start_pos = self.current_x; self.clean_step = 1
        elif self.clean_step == 1:
            if abs(self.current_x) >= (self.partition_max_x - self.SAFETY_MARGIN): self.trigger_rotation_phase(); return 
            if abs(self.current_x - self.step_start_pos) < 2.0: self.move_straight_locked(-math.pi/2, 'y', self.fixed_axis_value)
            else: self.clean_step = 2 
        elif self.clean_step == 2:
            if self.wait_in_place(2.0): self.clean_step = 3
        elif self.clean_step == 3:
            if self.turn_in_place(math.pi): self.fixed_axis_value = self.current_x; self.clean_step = 4
        elif self.clean_step == 4:
            # ONLY move to step 5 (backward) if we are actually at the end of the room (y < 0.5)
            if self.current_y > 0.7: 
                self.move_straight_locked(math.pi, 'x', self.fixed_axis_value)
            else: 
                self.get_logger().info("Bottom wall reached. Turning...")
                self.clean_step = 5
        elif self.clean_step == 5:
            if self.move_backward(1.0): self.clean_step = 6
        elif self.clean_step == 6:
            if self.turn_in_place(-math.pi/2): self.fixed_axis_value = self.current_y; self.step_start_pos = self.current_x; self.clean_step = 7
        elif self.clean_step == 7:
            if abs(self.current_x) >= (self.partition_max_x - self.SAFETY_MARGIN): self.trigger_rotation_phase(); return 
            if abs(self.current_x - self.step_start_pos) < 2.0: self.move_straight_locked(-math.pi/2, 'y', self.fixed_axis_value)
            else: self.clean_step = 8
        elif self.clean_step == 8:
            if self.wait_in_place(2.0): self.clean_step = 9
        elif self.clean_step == 9:
            if self.turn_in_place(0.0): self.fixed_axis_value = self.current_x; self.clean_step = 10
        elif self.clean_step == 10:
           # ONLY move to step 11 if we are near the top wall
            if self.current_y < (self.map_width - 0.7):
                self.move_straight_locked(0.0, 'x', self.fixed_axis_value)
            else:
                self.get_logger().info("Top wall reached. Turning...")
                self.clean_step = 11
        elif self.clean_step == 11:
            if self.move_backward(1.0): self.clean_step = 0 

    def lidar_callback(self, scan):
        if self.front_idx is None: self.compute_indices(scan)
        ranges = np.array(scan.ranges)
        ranges = np.where(np.isfinite(ranges), ranges, 12.0)
        HALF_WIDTH = 0.45 
        self.current_front_dist = 12.0
        self.best_blocker_angle = 0.0  # Track the angle of the closest hit

        # Search a wider window (e.g., +/- 45 degrees) to catch side-clipping
        search_window = int(math.radians(45) / scan.angle_increment)
        start_i = max(0, self.front_idx - search_window)
        end_i = min(len(ranges), self.front_idx + search_window)

        for i in range(start_i, end_i):
            d = ranges[i]
            angle = (i - self.front_idx) * scan.angle_increment
            
            # If the hit is physically within the robot's width corridor
            if abs(d * math.sin(angle)) < HALF_WIDTH:
                if d < self.current_front_dist:
                    self.current_front_dist = d
                    # Store relative angle to pinpoint the waste later
                    self.best_blocker_angle = angle

        if self.state == "MEASURING":
            if self.current_front_dist < 0.6:
                self.map_width = abs(self.current_y) + self.current_front_dist + 0.10
                msg = Float32(); msg.data = float(self.map_width); self.dim_pub.publish(msg)
                self.stop_robot()
                self.state = "BACKING_AFTER_MEASURE"
                self.get_logger().info(f"Width: {self.map_width:.2f}. Backing up...")
                return
            left = ranges[self.left_idx]
            cmd = Twist(); error = 0.5 - left
            cmd.linear.x = 0.8; cmd.angular.z = -1.5 * error
            self.cmd_pub.publish(cmd)

        elif self.state == "BACKING_AFTER_MEASURE":
            if self.move_backward(1.0): self.state = "WAITING"

        elif self.state == "WAITING":
            self.check_start_cleaning()

        elif self.state == "CLEANING":
            # 1. Determine if we are near a wall (to avoid trying to "clean" the room boundaries)
            is_near_wall = False
            if self.clean_step in [4, 10]: 
                if self.current_y < 0.8 or self.current_y > (self.map_width - 0.8):
                    is_near_wall = True

            # 2. Check for physical blockage (Waste)
            # We use a 0.8m threshold to match your collection logic
            BLOCK_DIST = 1.0 # Detection range
            if self.current_front_dist < BLOCK_DIST and not is_near_wall:
                self.stop_robot()

                if self.detected_wastes:
                    # USE THE ACTUAL ANGLE of the beam that detected the waste
                    # world_yaw (robot heading) + best_blocker_angle (beam offset)
                    total_angle = self.world_yaw + self.best_blocker_angle
                    obs_x = self.world_x + self.current_front_dist * math.cos(total_angle)
                    obs_y = self.world_y + self.current_front_dist * math.sin(total_angle)

                    best_idx = -1
                    min_dist_found = 1.0 
        
                    for i, waste in enumerate(self.detected_wastes):
                        wx, wy = waste['pos']
                        if math.hypot(obs_x - wx, obs_y - wy) < min_dist_found:
                            min_dist_found = math.hypot(obs_x - wx, obs_y - wy)
                            best_idx = i

                    if best_idx != -1:
                        target_name = self.detected_wastes[best_idx]['name']
                        if target_name not in self.deleted_names:
                            self.get_logger().info(f"MATCH FOUND (Side): Deleting {target_name}")
                            self.deleted_names.add(target_name)
                            self.delete_waste_visual(target_name)
                            self.detected_wastes.pop(best_idx)
                            
                            # Re-anchor to prevent rotation drift
                            if self.clean_step in [1, 7]: self.fixed_axis_value = self.current_y
                            elif self.clean_step in [4, 10]: self.fixed_axis_value = self.current_x
                            
                            return self.wait_in_place(1.0)
            # 3. Only scan for new waste if we aren't currently blocked
            if self.clean_step in [1, 4, 7, 10]:
                self.scan_for_waste(scan, ranges)

            self.run_cleaning_fsm()
        elif self.state == "COLLECTING":
            self.navigate_to_waste()

        elif self.state == "FINISHED":
            self.stop_robot()

    def check_start_cleaning(self):
        if self.state == "WAITING" and self.map_length is not None:
            self.state = "CLEANING"
            self.partition_max_x = self.map_length / float(self.robot_count)
            self.get_logger().info(f"R1 Partition: X [0 -> {self.partition_max_x:.2f}]")
    def delete_waste_visual(self, waste_name):
        # # If using Gazebo Classic (standard for many ROS 2 tutorials)
        # # The most reliable way is using the 'delete_entity' service
        # from gazebo_msgs.srv import DeleteEntity
    
        # client = self.create_client(DeleteEntity, '/delete_entity')
        # while not client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Delete service not available, waiting...')
    
        # request = DeleteEntity.Request()
        # request.name = waste_name
        # client.call_async(request)
        # self.get_logger().info(f"!!! ROS SERVICE SENT: Deleting {waste_name} !!!")
        import subprocess
        # Native Gazebo Harmonic (Jazzy) command
        cmd = [
        'gz', 'service', '-s', '/world/default/remove',
        '--reqtype', 'gz.msgs.Entity',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '2000',
        '--req', f'name: "{waste_name}", type: MODEL'
        ]
        try:
            # Use Popen instead of run so it doesn't block the robot's movement
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.get_logger().info(f"GAZEBO CMD SENT: {waste_name}")
        except Exception as e:
            self.get_logger().error(f"Command failed: {e}")

    def stop_robot(self): self.cmd_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(YAxisTrackerAndCleaner())
    rclpy.shutdown()

if __name__ == '__main__': main()