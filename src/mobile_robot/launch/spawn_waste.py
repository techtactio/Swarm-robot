import sys
import random
import subprocess
import time
import math

def spawn_waste():
    # --- HARDCODED RANGE: 10 to 20 ---
    MIN_RANGE = 10
    MAX_RANGE = 20
    N_WASTE = random.randint(MIN_RANGE, MAX_RANGE)
    
    # Arena Bounds
    MIN_XY = -8.5
    MAX_XY = 8.5
    
    # SPATIAL CONSTRAINTS
    MIN_DIST_BETWEEN_WASTE = 1.5  # Minimum distance between items
    MAX_RETRIES = 100             # Prevent infinite loops
    
    waste_map = []
    accepted_positions = []

    print(f"Starting spawn sequence...")
    print(f"Randomly selected to spawn {N_WASTE} waste items.")

    for i in range(N_WASTE):
        
        # 1. FORCE FIRST WASTE AT PARTITION LINE
        if i == 0:
            rx, ry = -7.0, 0.16
            position_found = True
            print(f"--> [Fixed] waste_0 placed at partition: ({rx}, {ry})")
        else:
            rx, ry = 0.0, 0.0
            position_found = False
            retries = 0
            
            while retries < MAX_RETRIES:
                rx = random.uniform(MIN_XY, MAX_XY)
                ry = random.uniform(MIN_XY, MAX_XY)
                
                # A. SAFE ZONE CHECK (Robot start zones)
                if rx < -8.0 and ry > 7.0:
                    retries += 1
                    continue
                
                # B. MINIMUM DISTANCE CHECK
                too_close = False
                for (px, py) in accepted_positions:
                    dist = math.hypot(rx - px, ry - py)
                    if dist < MIN_DIST_BETWEEN_WASTE:
                        too_close = True
                        break 
                
                if too_close:
                    retries += 1
                    continue
                
                position_found = True
                break
        
        if not position_found:
            print(f"Warning: Could not find a valid location for waste_{i}. Skipping.")
            continue
            
        accepted_positions.append((rx, ry))
        name = f"waste_{i}"
        waste_map.append(f"{name},{rx},{ry}\n")
        
        # SDF for a Green Pole
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
        # ROS 2 spawn command
        cmd = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', 'default',
            '-string', sdf_xml,
            '-name', name,
            '-x', str(rx),
            '-y', str(ry),
            '-z', '0.0'
        ]

        subprocess.run(cmd)
        time.sleep(0.1) 
        
    with open("waste_locations.csv", "w") as f:
        f.writelines(waste_map)
    
    print("-" * 30)
    print(f"SUCCESS: {len(waste_map)} items spawned.")
    print(f"Coordinates saved to 'waste_locations.csv'")

if __name__ == "__main__":
    spawn_waste()