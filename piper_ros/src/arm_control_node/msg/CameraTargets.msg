# CameraTargets.msg - 支持果树和苹果两种目标类型的消息

Header header

# 苹果目标位置
geometry_msgs/PoseStamped apple_pose

# 果树位置（苹果所在的果树）
geometry_msgs/PoseStamped tree_pose

# 目标类型标识
string target_type    # "apple" 或 "tree"

# 苹果ID（用于跟踪多个苹果）
int32 apple_id

# 果树ID（用于跟踪多个果树）
int32 tree_id

# 置信度（相机检测的可靠性）
float32 confidence
