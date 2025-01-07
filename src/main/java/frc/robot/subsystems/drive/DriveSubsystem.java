package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.constants.RobotConstants.DriveConstants;
import frc.robot.constants.RobotConstants.PivotId;
import frc.robot.sensors.gyro.Gyro;

public class DriveSubsystem extends SubsystemBase {
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  Gyro gyro;
  SysIdRoutine sysIdRoutine;
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  public DriveSubsystem(Gyro gyro) {
    this.gyro = gyro;
    switch (Robot.getMode()) { // create modules
      case REAL:
        modules[0] = new Module(new ModuleIOSparkMax(DriveConstants.FL), PivotId.FL);
        modules[1] = new Module(new ModuleIOSparkMax(DriveConstants.FR), PivotId.FR);
        modules[2] = new Module(new ModuleIOSparkMax(DriveConstants.BL), PivotId.BL);
        modules[3] = new Module(new ModuleIOSparkMax(DriveConstants.BR), PivotId.BR);
        break;

      case SIM:
        modules[0] = new Module(new ModuleIOSim(DriveConstants.FL), PivotId.FL);
        modules[1] = new Module(new ModuleIOSim(DriveConstants.FR), PivotId.FR);
        modules[2] = new Module(new ModuleIOSim(DriveConstants.BL), PivotId.BL);
        modules[3] = new Module(new ModuleIOSim(DriveConstants.BR), PivotId.BR);
        break;

      default:
        modules[0] = new Module(new ModuleIO() {}, PivotId.FL);
        modules[1] = new Module(new ModuleIO() {}, PivotId.FR);
        modules[2] = new Module(new ModuleIO() {}, PivotId.BL);
        modules[3] = new Module(new ModuleIO() {}, PivotId.BR);
        break;
    }
  }
}
