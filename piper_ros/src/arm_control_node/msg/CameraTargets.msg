# CameraTargets.msg - 支持红色和绿色果实检测的消息

Header header

# 红色果实位置（需要采集的目标）
geometry_msgs/PoseStamped red_apple_pose

# 绿色果实位置（需要避障的安全目标）
geometry_msgs/PoseStamped green_apple_pose

# 目标类型标识
string target_type    # "red_apple", "green_apple", "both", "none"

# 红色果实ID（用于跟踪多个红色果实）
int32 red_apple_id

# 绿色果实ID（用于跟踪多个绿色果实）
int32 green_apple_id

# 红色果实置信度
float32 red_confidence

# 绿色果实置信度
float32 green_confidence

# 置信度（相机检测的可靠性）
float32 confidence
