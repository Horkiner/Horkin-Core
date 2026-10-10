#pragma once

#include "types/time.hpp"

namespace horkin
{
    /** 可替换的时钟后端，便于单测注入假时间 */
    class IClock
    {
        public:
            virtual ~IClock() = default;

            /**
             * 读单调时钟
             *
             * 返回：TimeNs，纳秒
             */
            virtual TimeNs Now() const = 0;
    };
}
