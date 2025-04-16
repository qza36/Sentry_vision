# 裁判系统协议附录V1.7.0 
# 0x0201 机器人性能体系数据
uint16 current_hp # 机器人当前血量
uint16 maximum_hp # 机器人血量上限

# 0x0203 机器人位置数据
geometry_msgs/Pose robot_pos # 本机器人姿态

# 0x0204 机器人增益和底盘能量数据
uint8 recovery_buff # 机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%
uint8 cooling_buff # 机器人射击热量冷却倍率（直接值，值为 5 表示 5 倍冷却）
uint8 defence_buff # 机器人防御增益（百分比，值为 50 表示 50%防御增益）
uint8 vulnerability_buff # 机器人负防御增益（百分比，值为 30 表示-30%防御增益）
uint16 attack_buff # 机器人攻击增益（百分比，值为 50 表示 50%攻击增益）
uint8 remaining_energy # 机器人剩余能量值反馈，以 16 进制标识机器人剩余能量值比例，仅在机器人剩余能量小于 50%时反馈，其余默认反馈 0x32。

# 0x0206 伤害状态数据，伤害发生后发送
uint8 ARMOR_HIT = 0                     # 装甲模块被弹丸攻击导致扣血
uint8 SYSTEM_OFFLINE = 1                # 裁判系统重要模块离线导致扣血
uint8 OVER_SHOOT_SPEED = 2              # 射击初速度超限导致扣血
uint8 OVER_HEAT = 3                     # 枪口热量超限导致扣血
uint8 OVER_POWER = 4                    # 底盘功率超限导致扣血
uint8 ARMOR_COLLISION = 5               # 装甲模块受到撞击导致扣血

uint8 armor_id                          # 当扣血原因为装甲模块被弹丸攻击、受撞击、离线或测速模块离线时，
                                        # 数值为装甲模块或测速模块的 ID 编号；当其他原因导致扣血时，该数值为 0
uint8 hp_deduction_reason               # 血量变化类型

# 0x0208 允许发弹量
uint16 projectile_allowance_17mm        # 17mm 弹丸剩余发射次数
uint16 remaining_gold_coin              # 剩余金币数量