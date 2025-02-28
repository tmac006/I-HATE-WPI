#pragma once

#include <frc/apriltag/AprilTagFieldLayout.h>

#include "frc/apriltag/AprilTag.h"
#include "frc/apriltag/AprilTagFields.h"
#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Quaternion.h"
#include "frc/geometry/Rotation3d.h"
#include "frc/geometry/Translation3d.h"

namespace consts::yearspecific {
inline const frc::AprilTagFieldLayout TAG_LAYOUT =
    frc::AprilTagFieldLayout::LoadField(
        frc::AprilTagField::kDefaultField);
inline constexpr units::inch_t CLAW_OFFSET_L = 4.75_in;
inline constexpr units::inch_t CLAW_OFFSET_R = 6_in;
inline constexpr frc::Transform2d CLAW_TRANS_L{0_m, CLAW_OFFSET_L,
                                               frc::Rotation2d{}};
inline constexpr frc::Transform2d CLAW_TRANS_R{0_m, CLAW_OFFSET_R,
                                               frc::Rotation2d{}};
// inline const frc::AprilTag singleTagTesting{
//     19,
//     frc::Pose3d{frc::Translation3d{4.073905999999999_m, 4.745482_m,
//     0.308102_m},
//                 frc::Rotation3d{frc::Quaternion{0.5000000000000001, 0.0, 0.0,
//                                                 0.8660254037844386}}}};
// inline const frc::AprilTagFieldLayout TAG_LAYOUT =
//     frc::AprilTagFieldLayout({singleTagTesting}, 17.548_m, 8.052_m);
}  // namespace consts::yearspecific