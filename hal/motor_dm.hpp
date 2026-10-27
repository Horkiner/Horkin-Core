// 达妙 CAN 协议：只管 8 字节数据段，不碰串口、不碰总线
#pragma once

#include <array>
#include <cstdint>
#include <optional>

namespace horkin
{
    /** CAN 数据段，固定 8 字节 */
    using CanData = std::array<std::uint8_t, 8>;

    /** 读写寄存器统一发的 CAN ID */
    constexpr std::uint32_t kDmRegCanId = 0x7FF;


    /**
     * 达妙寄存器地址
     *
     * 说明
     * - 见 DM-J6248P 手册「寄存器列表及范围」
     * - 启动时读 PMax / VMax / TMax，和配置不一致就拒绝启动
     */
    enum class DmReg : std::uint8_t
    {
        CtrlMode = 0x0A,    // uint32_t, 1=MIT
        PMax = 0x15,        // float，位置映射半幅，rad
        VMax = 0x16,        // float，速度映射半幅，rad/s
        TMax = 0x17,        // float，扭矩映射半幅，N·m
    };

    /**
     * MIT 映射范围，输出轴单位
     *
     * 说明
     * - 必须与电机内 PMAX / VMAX / TMAX 一致
     * - 默认值取手册预设 ±12.5 / ±45 / ±18
     */
    struct MITLimits
    {
        double p_max{12.5};
        double v_max{45.0};
        double t_max{18.0};
    };

    /**
     * 反馈帧解包结果，输出轴单位
     *
     * 成员
     * - id：D[0] 低 4 位；CAN_ID 需 < 16，否则和 ERR 挤在一起
     * - err：0 失能，1 使能，8~E 故障
     * - pos / vel / tau：rad、rad/s、N·m
     * - t_mos / t_rotor：MOS、线圈温度，℃
     */
    struct MITFeedback
    {
        std::uint8_t id{0};
        std::uint8_t err{0};
        double pos{0};
        double vel{0};
        double tau{0};
        int t_mos{0};
        int t_rotor{0};
    };

    /** 定点 → 浮点：[0, 2^bits-1] 线性映射到 [lo, hi] */
    double uint_to_double(std::uint32_t x, double lo, double hi, int bits);

    /**
     * 浮点 → 定点
     *
     * 流程：先夹到 [lo, hi]，再映射到 [0, 2^bits-1]
     */
    std::uint32_t double_to_uint(double x, double lo, double hi, int bits);

    /**
     * 解反馈帧（帧 ID 为 Master ID）
     *
     * 布局：ID|ERR<<4 | POS 16 位 | VEL 12 位 | T 12 位 | T_MOS | T_Rotor
     */
    MITFeedback mit_unpack_feedback(const CanData& d, const MITLimits& lim);

    /** 使能 FFx7 FC，发往 CAN_ID */
    CanData mit_enable_data();

    /**
     * 失能 FF×7 FD，发往 CAN_ID
     *
     * 说明
     * - 电机收到任何发给自己的帧都会回一帧反馈
     * - 只读阶段周期发它来取状态，电机保持失能
     */
    CanData mit_disable_data();

    /** 清除错误 FF×7 FB，发往 CAN_ID */
    CanData mit_clear_error_data();

    /**
     * 读寄存器请求，发往 kDmRegCanId
     *
     * 布局：id 低字节 | id 高字节 | 0x33 | RID | 0…
     */
    CanData dm_read_reg_data(std::uint16_t can_id, DmReg rid);

    /**
     * 解读寄存器回复（帧 ID 为 Master ID）
     *
     * 返回：D[4..7] 小端拼成的 32 位原始值；对不上这条请求时返回空
     *
     * 说明：启动阶段只发读寄存器、不发控制帧，回复不会和反馈帧混在一起
     */
    std::optional<std::uint32_t> dm_parse_reg_reply(const CanData& d,
                                                    std::uint16_t can_id,
                                                    DmReg rid);

    /** 32 位原始值按 IEEE-754 float 解释 */
    float dm_reg_to_float(std::uint32_t raw);
}