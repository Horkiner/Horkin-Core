#include "model/detail/impl.hpp"

#include "types/joint_layout.hpp"

#include <pinocchio/multibody/joint/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <cstddef>
#include <stdexcept>
#include <string>

namespace horkin
{
    namespace
    {
        bool is_prismatic(const pinocchio::JointModel& j)
        {
            const std::string s = j.shortname();
            return s.find("Prismatic") != std::string::npos ||
                   s == "JointModelPX" || s == "JointModelPY" || s == "JointModelPZ";
        }
    }

    RobotModel::RobotModel(const std::string& urdf_path, const std::string& ee_frame)
        : impl_(std::make_unique<Impl>())
    {
        // 读取 URDF，并将模型写入 impl_->model
        pinocchio::urdf::buildModel(urdf_path, impl_->model, /*verbose=*/false);

        // 比较 URDF 读取到的关节数是否和定义数 kNJoints 相等
        if (impl_->model.nq != static_cast<int>(kNJoints))
        {
            throw std::runtime_error(
                "URDF nq=" + std::to_string(impl_->model.nq) +
                " != kNJoints=" + std::to_string(kNJoints));
        }

        std::size_t logical = 0;
        for (pinocchio::JointIndex i = 1; i < static_cast<pinocchio::JointIndex>(impl_->model.njoints); ++i)
        {
            const pinocchio::JointModel& j = impl_->model.joints[i];

            // 检查可活动关节数是否和定义的对齐
            if (j.nq() == 0)   // Pinocchio 中 nq()==0 表示固定关节，nq()==1 表示转动 / 移动关节
            {
                continue;
            }
            if (logical >= kNJoints)
            {
                throw std::runtime_error("URDF has more actuated joints than kNJoints");
            }

            // 检查关节类型是否和定义的对齐
            const bool urdf_pris = is_prismatic(j);
            const bool layout_pris = (kJointKind[logical] == JointKind::Prismatic);
            if (urdf_pris != layout_pris)
            {
                throw std::runtime_error("URDF joint " + std::to_string(logical) +
                                         " kind does not match kJointKind");
            }
            ++logical;
        }
        if (logical != kNJoints)
        {
            throw std::runtime_error("URDF actuated joint count != kNJoints");
        }

        if (!impl_->model.existFrame(ee_frame))
        {
            throw std::runtime_error("URDF missing frame: " + ee_frame);
        }

        impl_->ee = impl_->model.getFrameId(ee_frame, pinocchio::FIXED_JOINT);
        impl_->data = pinocchio::Data(impl_->model);

        // 读取关节限位
        for (int i = 0; i < impl_->model.nq; ++i)
        {
            const auto k = static_cast<std::size_t>(i);
            impl_->q_lower[k] = impl_->model.lowerPositionLimit[i];
            impl_->q_upper[k] = impl_->model.upperPositionLimit[i];
        }
    }

    RobotModel::~RobotModel() = default;

    JointVec RobotModel::q_lower() const { return impl_->q_lower; }
    JointVec RobotModel::q_upper() const { return impl_->q_upper; }
}
