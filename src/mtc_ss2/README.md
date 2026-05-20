In src/ur_onrobot/ur_onrobot_moveit_config/srdf/ur_onrobot_macro.srdf.xacro

Change "closed" from "0.0" to "0.049"

    <group_state name="closed" group="${prefix}${name}_gripper">
      <joint name="finger_width" value="0.049"/>
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