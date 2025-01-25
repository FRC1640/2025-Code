package frc.robot.subsystems.lift;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LiftSubsystem extends SubsystemBase {
  LiftIOSpark liftIO;

  public LiftSubsystem(LiftIOSpark liftIO) {
    this.liftIO = liftIO;
  }
}
