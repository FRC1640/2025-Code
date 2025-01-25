package frc.robot.subsystems.lift;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;

public class LiftSubsystem extends SubsystemBase{
  private final Module[] modules = new Module[2];
  public LiftSubsystem () {
    switch (Robot.getMode()) { // create modules
      case REAL:
        modules[0] = new Module(new ModuleIOSparkMax(0), 0);
        modules[1] = new Module(new ModuleIOSparkMax(1), 1);
        break;

      case SIM:
        modules[0] = new Module(new ModuleIOSim(0), 0);
        modules[1] = new Module(new ModuleIOSim(1) 1);
        break;

      default:
        modules[0] = new Module(new ModuleIO() {}, PivotId.FL);
        modules[1] = new Module(new ModuleIO() {}, PivotId.FR);
        break;
    }
  }
}
