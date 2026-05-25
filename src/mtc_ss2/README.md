In src/ur_onrobot/ur_onrobot_moveit_config/srdf/ur_onrobot_macro.srdf.xacro

Change "closed" to "0.049", "open" to "0.100"

    <group_state name="closed" group="${prefix}${name}_gripper">
      <joint name="finger_width" value="0.049"/>
    </group_state>
    <group_state name="open" group="${prefix}${name}_gripper">
      <joint name="finger_width" value="0.100"/>
    </group_state>

Add ready_pose2 and search (under test_configuration)

    <group_state name="${prefix}ready_pose2" group="${prefix}${name}_manipulator">
      <joint name="${prefix}shoulder_pan_joint" value="-0.82" />
      <joint name="${prefix}wrist_3_joint" value="0.74" />
      <joint name="${prefix}wrist_2_joint" value="1.57" />
      <joint name="${prefix}wrist_1_joint" value="-2.24" />
      <joint name="${prefix}elbow_joint" value="-0.66" />
      <joint name="${prefix}shoulder_lift_joint" value="-1.8" />
    </group_state>
    <group_state name="${prefix}search" group="${prefix}${name}_manipulator">
      <joint name="${prefix}shoulder_pan_joint" value="-1.23" />
      <joint name="${prefix}wrist_3_joint" value="0.32" />
      <joint name="${prefix}wrist_2_joint" value="1.57" />
      <joint name="${prefix}wrist_1_joint" value="-2.78" />
      <joint name="${prefix}elbow_joint" value="-0.21" />
      <joint name="${prefix}shoulder_lift_joint" value="-1.70" />
    </group_state>

In src/ur_onrobot/ur_onrobot_moveit_config/config/controllers.yaml

add at the end

    trajectory_execution:
      allowed_execution_duration_scaling: 2.0
      execution_duration_monitoring: false
      allowed_start_tolerance: 0.05

In src/ur_onrobot/ur_onrobot_moveit_config/config/ompl_planning.yaml

add at the end

    ur_onrobot_manipulator:
      planner_configs:
        - SBLkConfigDefault
        - ESTkConfigDefault
        - LBKPIECEkConfigDefault
        - BKPIECEkConfigDefault
        - KPIECEkConfigDefault
        - RRTkConfigDefault
        - RRTConnectkConfigDefault
        - RRTstarkConfigDefault
        - TRRTkConfigDefault
        - PRMkConfigDefault
        - PRMstarkConfigDefault
      longest_valid_segment_fraction: 0.001
      enforce_joint_model_state_space: true


In src/ur_onrobot/ur_onrobot_description/urdf/ur_onrobot_macro.xacro

add camera collision

      <!-- Connect the OnRobot to the UR robot -->
      <joint name="${tf_prefix}onrobot_base_link_joint" type="fixed">
          <parent link="${tf_prefix}tool0"/>
          <child link="${tf_prefix}onrobot_base_link"/>
          <origin xyz="0 0 0" rpy="0 0 ${-pi/2}"/>  <!-- modified, original xyz="0 0 0", increase Z for offset-->
      </joint>

      <!-- Camera mount and RealSense D435i -->
      <link name="${tf_prefix}realsense_mount_camera">
          <collision>
              <geometry>
                  <box size="0.090 0.12 0.04"/>
              </geometry>
              <origin xyz="0 0 0" rpy="0 0 0"/>
          </collision>
          <visual>
              <geometry>
                  <box size="0.090 0.12 0.04"/>
              </geometry>
              <origin xyz="0 0 0" rpy="0 0 0"/>
          </visual>
      </link>
      <joint name="${tf_prefix}camera_mount_joint" type="fixed">
          <parent link="${tf_prefix}tool0"/>
          <child link="${tf_prefix}realsense_mount_camera"/>
          <origin xyz="0 -0.025 -0.02" rpy="0 0 0"/> <!-- increase Z for offset -->
      </joint>

      <!-- Include cable connector collision links -->