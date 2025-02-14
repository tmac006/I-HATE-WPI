#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>

#include "subsystems/Drive.h"


class Autos {
 public:
  explicit Autos(Drive& driveSub)
      : m_driveSub{driveSub} {
    BindCommandsAndTriggers();

    selectCommand = frc2::cmd::Select<AutoSelector>(
        [this] { return autoChooser.GetSelected(); },
        std::pair{NOTHING, frc2::cmd::None()},
        std::pair{LEFT_EDGE_TWO_CORAL,
                  pathplanner::PathPlannerAuto("LeftEdgeTwoCoral").ToPtr()},
        std::pair{RIGHT_EDGE_TWO_CORAL,
                  pathplanner::PathPlannerAuto("RightEdgeTwoCoral").ToPtr()});

    autoChooser.SetDefaultOption("Do Nothing", AutoSelector::NOTHING);
    autoChooser.AddOption("Left Edge Two Coral",
                          AutoSelector::LEFT_EDGE_TWO_CORAL);
    autoChooser.AddOption("Right Edge Two Coral",
                          AutoSelector::RIGHT_EDGE_TWO_CORAL);

    frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
  }

  frc2::Command* GetSelectedCommand() { return selectCommand.get(); }

 private:
  void BindCommandsAndTriggers() {}

  enum AutoSelector { NOTHING, LEFT_EDGE_TWO_CORAL, RIGHT_EDGE_TWO_CORAL };

  frc::SendableChooser<AutoSelector> autoChooser;

  Drive& m_driveSub;

  frc2::CommandPtr selectCommand{frc2::cmd::None()};
};
