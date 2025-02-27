package frc.robot.subsystems.drive;

import frc.robot.constants.RobotConstants.MotorInfo;
import frc.robot.constants.RobotConstants.PivotId;

// class to store constants for a module
public class ModuleInfo {

  PivotId id;
  int driveID;
  int steerID;
  int resolverChannel;
  double angleOffset;

  public ModuleInfo(PivotId id, int driveID, int steerID, int resolverChannel, double angleOffset) {
    MotorInfo.motorLoggingManager.addMotorAlias(driveID, id.name() + " Drive");
    MotorInfo.motorLoggingManager.addMotorAlias(steerID, id.name() + " Steer");

    this.id = id;
    this.driveID = driveID;
    this.steerID = steerID;
    this.resolverChannel = resolverChannel;
    this.angleOffset = angleOffset;
  }
}
