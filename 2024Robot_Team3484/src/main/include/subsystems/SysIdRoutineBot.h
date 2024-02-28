#ifndef SYS
#define SYS

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "subsystems/Drivetrain.h"

class SysIdRoutineBot {
 public:
  SysIdRoutineBot();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  void ConfigureBindings();
  frc2::CommandXboxController m_driverController{0};
  Drive m_drive;
};

#endif