// 达妙 MIT / 寄存器：数据段打包与解包
#include "hal/motor_dm.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <optional>

namespace horkin
{
    double uint_to_double(std::uint32_t x, double lo, double hi, int bits)
    {
        const double span = hi - lo;
        const double full = static_cast<double>((1u << bits) - 1u);
        return static_cast<double>(x) * span / full + lo;
    }

    std::uint32_t double_to_uint(double x, double lo, double hi, int bits)
    {
        const double span = hi - lo;
        const double full = static_cast<double>((1u << bits) - 1u);
        x = std::clamp(x, lo, hi);
        return static_cast<std::uint32_t>((x - lo) * full / span);
    }

    MITFeedback mit_unpack_feedback(const CanData &d, const MITLimits &lim)
    {
        // POS：D[1] 高字节，D[2] 低字节
        const std::uint32_t p_int = (std::uint32_t(d[1]) << 8) | d[2];
        // VEL：D[3] 高 8 位，D[4] 高 4 位
        const std::uint32_t v_int = (std::uint32_t(d[3]) << 4) | (d[4] >> 4);
        // T：D[4] 低 4 位，D[5] 低 8 位
        const std::uint32_t t_int = (std::uint32_t(d[4] & 0x0F) << 8) | d[5];

        MITFeedback fb;
        fb.id = d[0] & 0x0F;
        fb.err = d[0] >> 4;
        fb.pos = uint_to_double(p_int, -lim.p_max, lim.p_max, 16);
        fb.vel = uint_to_double(v_int, -lim.v_max, lim.v_max, 12);
        fb.tau = uint_to_double(t_int, -lim.t_max, lim.t_max, 12);
        fb.t_mos = d[6];
        fb.t_rotor = d[7];
        return fb;
    }

    CanData mit_enable_data()
    {
        return {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    }

    CanData mit_disable_data()
    {
        return {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    }

    CanData mit_clear_error_data()
    {
        return {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
    }

    CanData dm_read_reg_data(std::uint16_t can_id, DmReg rid)
    {
        return {static_cast<std::uint8_t>(can_id & 0xFF),
                static_cast<std::uint8_t>(can_id >> 8),
                0x33,
                static_cast<std::uint8_t>(rid),
                0, 0, 0, 0};
    }

    std::optional<std::uint32_t> dm_parse_reg_reply(const CanData &d, std::uint16_t can_id, DmReg rid)
    {
        if (d[0] != (can_id & 0xFF) || d[1] != (can_id >> 8) || 
            d[2] != 0x33 || d[3] != static_cast<std::uint8_t>(rid))
        {
            return std::nullopt;
        }
        // 手册：数据低位在 D4，最高位 D7
        return  std::uint32_t(d[4])
             | (std::uint32_t(d[5]) << 8)
             | (std::uint32_t(d[6]) << 16)
             | (std::uint32_t(d[7]) << 24);
    }

    float dm_reg_to_float(std::uint32_t raw)
    {
        float f;
        std::memcpy(&f, &raw, sizeof f);
        return f;
    }
}