#pragma once

#include "types/command.hpp"
#include "types/joint.hpp"

namespace horkin
{
    /** 可替换的关节后端，真机与仿真同一套 */
    class IJointDriver
    {
        public:
            virtual ~IJointDriver() = default;

            /** 使能关节驱动 */
            virtual void Enable() = 0;

            /** 下使能 */
            virtual void Disable() = 0;

            /**
             * 读关节状态，非阻塞
             *
             * 参数
             * - out：写入 JointState；失败时 valid=false
             */
            virtual void Read(JointState& out) = 0;

            /**
             * 写关节指令，仅实时线程调用
             *
             * 参数
             * - cmd：JointCommand
             */
            virtual void Write(const JointCommand& cmd) = 0;
    };
}
