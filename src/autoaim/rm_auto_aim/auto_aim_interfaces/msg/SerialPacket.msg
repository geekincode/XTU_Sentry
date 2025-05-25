# SerialPacket.msg
uint8 header
uint8 detect_color      # 0-red 1-blue
uint8 task_mode         # 0-手打 1-自瞄 2-大符
bool reset_tracker      # 1-重启跟踪器
bool is_play            # 1-play 0-stop
bool change_target      # 1-change target
uint8 reserved
float32 roll
float32 pitch
float32 yaw
uint16 robot_hp         # 机器人自身血量
uint16 game_time        # (s) game time [0, 450]
uint16 checksum