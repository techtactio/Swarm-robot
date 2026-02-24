import sys
import random
import subprocess
import time
import math  # Added math module for distance calculation

def spawn_waste():
    # Number of waste items to spawn
    N_WASTE = 10
    
    # Arena Bounds
    MIN_XY = -8.5
    MAX_XY = 8.5
    
    # --- NEW SPATIAL CONSTRAINTS ---
    MIN_DIST_BETWEEN_WASTE = 1.5  # Minimum distance (in meters) between items
    MAX_RETRIES = 100             # Prevent infinite loops if the arena gets too crowded
    
    waste_map = []
    accepted_positions = []       # List to store (x, y) tuples of spawned waste

    print(f"Spawning {N_WASTE} waste items (1 fixed, {N_WASTE-1} random)...")

    for i in range(N_WASTE):
        
        # --- NEW: FORCE FIRST WASTE AT PARTITION LINE ---
        if i == 0:
            rx, ry = -7.0, 0.16
            position_found = True
            print(f"--> Placing waste_0 precisely at the partition line: ({rx}, {ry})")
        else:
            # 1. Generate Random Position with constraints for the rest
            rx, ry = 0.0, 0.0
            position_found = False
            retries = 0
            
            while retries < MAX_RETRIES:
                rx = random.uniform(MIN_XY, MAX_XY)
                ry = random.uniform(MIN_XY, MAX_XY)
                
                # A. SAFE ZONE CHECK (Keep away from robot start zones)
                if rx < -8.0 and ry > 7.0:
                    retries += 1
                    continue
                
                # B. MINIMUM DISTANCE CHECK (Keep away from other waste)
                too_close = False
                for (px, py) in accepted_positions:
                    dist = math.hypot(rx - px, ry - py)
                    if dist < MIN_DIST_BETWEEN_WASTE:
                        too_close = True
                        break  # Break out of the inner for-loop early
                
                if too_close:
                    retries += 1
                    continue
                
                # If it passes both checks, mark as found and break the while loop
                position_found = True
                break
        
        # If we couldn't find a spot after MAX_RETRIES, skip this item
        if not position_found:
            print(f"Warning: Could not find a valid location for waste_{i} after {MAX_RETRIES} attempts. Skipping.")
            continue
            
        # Add the valid position to our tracking list
        accepted_positions.append((rx, ry))

        name = f"waste_{i}"
        waste_map.append(f"{name},{rx},{ry}\n")
        
        # 2. Define the SDF for a Green Pole (Waste)
        sdf_xml = f"""
<?xml version='1.0'?>
<sdf version='1.7'>
<model name='{name}'>
    <static>true</static>
    <link name='link'>
    <visual name='visual'>
        <geometry><box><size>0.1 0.1 2.0</size></box></geometry>
        <material><ambient>0 1 0 1</ambient><diffuse>0 1 0 1</diffuse></material>
    </visual>
    <collision name='collision'>
        <geometry><box><size>0.1 0.1 2.0</size></box></geometry>
    </collision>
    </link>
</model>
</sdf>
"""
        # 3. Build the ROS 2 command to spawn it
        cmd = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', 'default',
            '-string', sdf_xml,
            '-name', name,
            '-x', str(rx),
            '-y', str(ry),
            '-z', '0.0'
        ]

        # 4. Execute
        subprocess.run(cmd)
        time.sleep(0.2) 
        
    with open("waste_locations.csv", "w") as f:
        f.writelines(waste_map)
    print(f"Map saved with {len(waste_map)} items.")

if __name__ == "__main__":
    spawn_waste()