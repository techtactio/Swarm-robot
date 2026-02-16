import sys
import random
import subprocess
import time

def spawn_waste():
    # Number of waste items to spawn
    N_WASTE = 10
    
    # Arena Bounds (Inner area to avoid walls)
    # Arena is 20x20 (-10 to 10), so spawn in -9 to 9
    MIN_XY = -9.0
    MAX_XY = 9.0
    waste_map = []

    print(f"Spawning {N_WASTE} waste items...")

    for i in range(N_WASTE):
        # 1. Generate Random Position
        while True:
            rx = random.uniform(MIN_XY, MAX_XY)
            ry = random.uniform(MIN_XY, MAX_XY)
            
            # SAFE ZONE CHECK: Keep away from robot start zones
            # Robot 1 starts at (-9.4, 9.4)
            # Robot 2 starts at (-9.4, 8.4)
            # Avoid the top-left corner (-10 to -8 in X, 7 to 10 in Y)
            if rx < -8.0 and ry > 7.0:
                continue
            break

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
        # Note: 'ros_gz_sim' might be 'ros_ign_sim' on older versions
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
        time.sleep(0.2) # Small delay to prevent crashing Gazebo service
    with open("waste_locations.csv", "w") as f:
        f.writelines(waste_map)
    print(f"Map saved with {len(waste_map)} items.")

if __name__ == "__main__":
    spawn_waste()