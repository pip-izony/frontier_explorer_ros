#!/bin/bash

cd /root/ros2_ws/src

git clone -b ros2 https://github.com/NovoG93/sjtu_drone.git

python3 << 'PYEOF'
sensor_block = '''
      <!-- ===== Velodyne VLP-16 (Frontier Explorer) ===== -->
      <sensor name='velodyne_vlp16' type='ray'>
        <pose>0 0 0.10 0 0 0</pose>
        <visualize>true</visualize>
        <update_rate>10</update_rate>
        <always_on>true</always_on>
        <ray>
          <scan>
            <horizontal>
              <samples>900</samples>
              <resolution>1</resolution>
              <min_angle>-3.14159</min_angle>
              <max_angle>3.14159</max_angle>
            </horizontal>
            <vertical>
              <samples>16</samples>
              <resolution>1</resolution>
              <min_angle>-0.2617993878</min_angle>
              <max_angle>0.2617993878</max_angle>
            </vertical>
          </scan>
          <range>
            <min>0.30</min>
            <max>30.0</max>
            <resolution>0.01</resolution>
          </range>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </ray>
        <plugin name='gazebo_ros_velodyne' filename='libgazebo_ros_ray_sensor.so'>
          <ros>
            <namespace>/simple_drone</namespace>
            <remapping>~/out:=velodyne_points</remapping>
          </ros>
          <output_type>sensor_msgs/PointCloud2</output_type>
          <frame_name>velodyne_link</frame_name>
        </plugin>
      </sensor>
      <!-- ===== /Velodyne ===== -->
'''

path = '/root/ros2_ws/src/sjtu_drone/sjtu_drone_description/models/sjtu_drone/sjtu_drone.sdf'
with open(path) as f:
    content = f.read()
if 'velodyne_vlp16' in content:
    print("SKIP (already patched)")
else:
    content = content.replace('</link>', sensor_block + '\n    </link>', 1)
    with open(path, 'w') as f:
        f.write(content)
    print("PATCHED")
PYEOF

python3 << 'PYEOF'
urdf_block = '''
  <!-- ===== Velodyne VLP-16 (Frontier Explorer) ===== -->
  <link name="velodyne_link">
    <inertial>
      <mass value="0.83"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.000908" ixy="0" ixz="0" iyy="0.000908" iyz="0" izz="0.001104"/>
    </inertial>
    <visual>
      <geometry><cylinder radius="0.0516" length="0.0717"/></geometry>
    </visual>
    <collision>
      <geometry><cylinder radius="0.0516" length="0.0717"/></geometry>
    </collision>
  </link>

  <joint name="velodyne_joint" type="fixed">
    <parent link="base_link"/>
    <child link="velodyne_link"/>
    <origin xyz="0 0 0.10" rpy="0 0 0"/>
  </joint>

  <gazebo reference="velodyne_link">
    <sensor type="ray" name="velodyne_vlp16">
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>10</update_rate>
      <always_on>true</always_on>
      <ray>
        <scan>
          <horizontal>
            <samples>900</samples><resolution>1</resolution>
            <min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle>
          </horizontal>
          <vertical>
            <samples>16</samples><resolution>1</resolution>
            <min_angle>-0.2617993878</min_angle>
            <max_angle> 0.2617993878</max_angle>
          </vertical>
        </scan>
        <range>
          <min>0.30</min><max>30.0</max><resolution>0.01</resolution>
        </range>
        <noise>
          <type>gaussian</type><mean>0.0</mean><stddev>0.01</stddev>
        </noise>
      </ray>
      <plugin name="gazebo_ros_velodyne" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <namespace>/simple_drone</namespace>
          <remapping>~/out:=velodyne_points</remapping>
        </ros>
        <output_type>sensor_msgs/PointCloud2</output_type>
        <frame_name>velodyne_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>
  <!-- ===== /Velodyne ===== -->
'''

import os
base = '/root/ros2_ws/src/sjtu_drone/sjtu_drone_description/urdf'
for fname in ['sjtu_drone.urdf.xacro', 'sjtu_drone.urdf']:
    path = os.path.join(base, fname)
    if not os.path.exists(path):
        print(f"skip (not found): {fname}")
        continue
    with open(path) as f:
        content = f.read()
    if 'velodyne_vlp16' in content:
        print(f"skip (already patched): {fname}")
        continue
    content = content.replace('</robot>', urdf_block + '\n</robot>')
    with open(path, 'w') as f:
        f.write(content)
    print(f"patched: {fname}")
PYEOF

cd /root/ros2_ws

colcon build --symlink-install --packages-select sjtu_drone_description sjtu_drone_bringup sjtu_drone_control
