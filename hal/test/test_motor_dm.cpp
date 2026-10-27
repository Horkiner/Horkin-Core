#include "hal/motor_dm.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

int main()
{
    const horkin::MITLimits lim{12.5, 45.0, 18.0};

    // 反向构造：id=1，err=1，pos=1.0，vel=-2.0，tau=3.0
    const std::uint32_t p = horkin::double_to_uint(1.0, -lim.p_max, lim.p_max, 16);
    const std::uint32_t v = horkin::double_to_uint(-2.0, -lim.v_max, lim.v_max, 12);
    const std::uint32_t t = horkin::double_to_uint(3.0, -lim.t_max, lim.t_max, 12);

    horkin::CanData d{};
    d[0] = (1 << 4) | 1;
    d[1] = static_cast<std::uint8_t>(p >> 8);
    d[2] = static_cast<std::uint8_t>(p & 0xFF);
    d[3] = static_cast<std::uint8_t>(v >> 4);
    d[4] = static_cast<std::uint8_t>(((v & 0x0F) << 4) | (t >> 8));
    d[5] = static_cast<std::uint8_t>(t & 0xFF); 
    d[6] = 30;
    d[7] = 35;

    const horkin::MITFeedback fb = horkin::mit_unpack_feedback(d, lim);

    // 容差取一个量化步长
    const double tol_p = 2 * lim.p_max / 65535.0;
    const double tol_v = 2 * lim.v_max / 4095.0;
    const double tol_t = 2 * lim.t_max / 4095.0;

    if (fb.id != 1 || fb.err != 1)
    {
        std::fprintf(stderr, "id/err wrong: %u %u\n", fb.id, fb.err);
        return 1;
    }
    if (std::abs(fb.pos - 1.0) > tol_p || 
        std::abs(fb.vel + 2.0) > tol_v ||
        std::abs(fb.tau - 3.0) > tol_t)
    {
        std::fprintf(stderr, "unpack wrong: %.5f %.5f %.5f\n", fb.pos, fb.vel, fb.tau);
        return 1;
    }
    if (fb.t_mos != 30 || fb.t_rotor != 35)
    {
        std::fprintf(stderr, "temperature wrong\n");
        return 1;
    }

    // 超范围应夹到上限
    if (horkin::double_to_uint(100.0, -lim.p_max, lim.p_max, 16) != 65535u)
    {
        std::fprintf(stderr, "clamp failed\n");
        return 1;
    }

    // 使能 / 失能 / 清错，对照手册读数
    if (horkin::mit_enable_data()[7] != 0xFC ||
        horkin::mit_disable_data()[7] != 0xFD ||
        horkin::mit_clear_error_data()[7] != 0xFB)
    {
        std::fprintf(stderr, "command suffix wrong\n");
        return 1;
    }

    // 读 PMAX：01 00 33 15 00 00 00 00
    const horkin::CanData req = horkin::dm_read_reg_data(0x01, horkin::DmReg::PMax);
    const horkin::CanData req_ok{0x01, 0x00, 0x33, 0x15, 0,0, 0, 0};
    if (req != req_ok)
    {
        std::fprintf(stderr, "read reg request wrong\n");
        return 1;
    }

    // 12.5f 的位模式是 0x41480000，低位在 D4
    const horkin::CanData rep{0x01, 0x00, 0x33, 0x15, 0x00, 0x00, 0x48, 0x41};
    const auto raw = horkin::dm_parse_reg_reply(rep, 0x01, horkin::DmReg::PMax);
    if (!raw || horkin::dm_reg_to_float(*raw) != 12.5f)
    {
        std::fprintf(stderr, "reg reply parse wrong\n");
        return 1;
    }

    // RID 对不上应返回空
    if (horkin::dm_parse_reg_reply(rep, 0x01, horkin::DmReg::VMax))
    {
        std::fprintf(stderr, "reg reply accepted wrong rid\n");
        return 1;
    }

    std::printf("test_motor_dm ok\n");
    return 0;
}