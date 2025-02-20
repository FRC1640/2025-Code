package frc.robot.subsystems.drive;

import frc.robot.constants.RobotConstants.MotorInfo;
import frc.robot.constants.RobotConstants.PivotId;

// class to store constants for a module
public class ModuleInfo {

  PivotId id;
  int driveChannel;
  int steerChannel;
  int resolverChannel;
  double angleOffset;

  public ModuleInfo(
      PivotId id, int driveChannel, int steerChannel, int resolverChannel, double angleOffset) {
    MotorInfo.motorLoggingManager.addMotorAlias(driveChannel, id.name() + " Drive");
    MotorInfo.motorLoggingManager.addMotorAlias(steerChannel, id.name() + " Steer");

    this.id = id;
    this.driveChannel = driveChannel;
    this.steerChannel = steerChannel;
    this.resolverChannel = resolverChannel;
    this.angleOffset = angleOffset;
  }
}
