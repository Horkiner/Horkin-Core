#pragma once

#include "types/wrench.hpp"

namespace horkin
{
    /** 可替换的六维力后端，输出传感器系原始力 */
    class IForceSensor
    {
        public:
            virtual ~IForceSensor() = default;

            /**
             * 读六维力，非阻塞
             *
             * 参数
             * - out：传感器系原始力；失败时 valid=false
             */
            virtual void Read(WrenchSample& out) = 0;
    };
}
