# 0x0209 机器人 RFID 模块状态
uint8 NOT_DETECTED = 0                              # RFID card not detected
uint8 DETECTED = 1                                  # RFID card detected
bool friendly_supply_zone_non_exchange  # 己方与兑换区不重叠的补给区/RMUL 补给区
bool center_gain_point                  # 中心增益点（仅 RMUL 适用）