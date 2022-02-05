set_round() {
    local velocity_constant="$1" \
          velocity_keep_position="$2" \
          velocity_max="$3"

    shift 3               # get rid of the first two arguments

    ros2 param set /mission_manager_node velocity.constant $velocity_constant
    ros2 param set /mission_manager_node velocity.keep_position $velocity_keep_position
    ros2 param set /mission_manager_node velocity.max $velocity_max
    # ...
   
}

get_round() {

    echo "velocity.constant: -------------------------------"
    ros2 param get /mission_manager_node velocity.constant

    echo "velocity.keep_position: -------------------------------"
    ros2 param get /mission_manager_node velocity.keep_position

    echo "velocity.max: -------------------------------"
    ros2 param get /mission_manager_node velocity.max
    # ...
   
}