#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/NetworkButton.h>

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include "Autos.h"
#include "frc/geometry/Pose2d.h"
#include "frc2/command/button/Trigger.h"
#include "str/SuperstructureDisplay.h"
#include "str/vision/VisionSystem.h"
#include "subsystems/Drive.h"


class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();
  Drive& GetDrive();
  str::vision::VisionSystem& GetVision();
  str::SuperstructureDisplay& GetSuperStructureDisplay();

 private:
  void ConfigureBindings();
  void ConfigureSysIdBinds();
  frc2::CommandPtr SteerVoltsSysIdCommands(std::function<bool()> fwd,
                                           std::function<bool()> quasistatic);
  frc2::CommandPtr SteerTorqueCurrentSysIdCommands(
      std::function<bool()> fwd, std::function<bool()> quasistatic);
  frc2::CommandPtr DriveSysIdCommands(std::function<bool()> fwd,
                                      std::function<bool()> quasistatic);
  frc2::CommandPtr WheelRadiusSysIdCommands(std::function<bool()> fwd);
  frc2::Trigger NoButtonsPressed();

  frc2::CommandXboxController driverJoystick{0};

  str::SuperstructureDisplay display{};

  Drive driveSub{};
 

  str::vision::VisionSystem vision{
      [this](const frc::Pose2d& pose, units::second_t time,
             const Eigen::Vector3d& stdDevs) {
        driveSub.AddVisionMeasurement(pose, time, stdDevs);
      },
      [this](const frc::Pose2d& pose, units::second_t time,
             const Eigen::Vector3d& stdDevs) {
        driveSub.AddSingleTagVisionMeasurement(pose, time, stdDevs);
      }};

  Autos autos{driveSub};

  std::shared_ptr<nt::NetworkTable> tuningTable{
      nt::NetworkTableInstance::GetDefault().GetTable("Tuning")};
  frc2::NetworkButton steerTuneBtn{tuningTable, "SteerPidTuning"};
  frc2::NetworkButton driveTuneBtn{tuningTable, "DrivePidTuning"};
  frc2::NetworkButton steerSysIdVoltsBtn{tuningTable, "SteerSysIdVolts"};
  frc2::NetworkButton steerSysIdTorqueCurrentBtn{tuningTable,
                                                 "SteerSysIdTorqueCurrent"};
  frc2::NetworkButton driveSysIdBtn{tuningTable, "DriveSysId"};
  frc2::NetworkButton wheelRadiusBtn{tuningTable, "WheelRadius"};
};