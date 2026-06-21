std_msgs/Header header

# Madde 6: common FIELD_ENU frame (architecture §6). field_position_enu is
# what swarm_agent_node uses for mission/formation/separation math;
# px4_origin_field_enu is the live offset between this UAV's PX4 local-NED
# zero and FIELD_ENU, refreshed from real GNSS (replaces the static spawn_gz
# manifest constant used before this item).
bool valid
float32[3] field_position_enu
float32[3] px4_origin_field_enu
float32[3] local_position_ned
float32 heading
