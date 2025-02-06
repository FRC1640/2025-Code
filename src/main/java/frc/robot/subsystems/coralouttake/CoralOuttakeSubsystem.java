package frc.robot.subsystems.coralouttake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants.ReefDetectorConstants;
import org.littletonrobotics.junction.Logger;

public class CoralOuttakeSubsystem extends SubsystemBase {

  CoralOuttakeIOInputsAutoLogged inputs = new CoralOuttakeIOInputsAutoLogged();
  CoralOuttakeIO io;
  private double time = 0;
  private double lastTime = 0.0;

  public CoralOuttakeSubsystem(CoralOuttakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/", inputs);
    if (isCoralDetected()) {
      time += (System.currentTimeMillis() - lastTime) / 1000;
    } else {
      time = 0;
    }
    lastTime = System.currentTimeMillis();
    Logger.recordOutput("CoralDetector/DetectionTime", time);
    Logger.recordOutput("CoralDetector/DetectionTimeBool", isDetectingTimed());
  }

  public void stop() {
    io.setIntakeVoltage(0);
  }

  public double returnTemp() {
    return inputs.tempCelcius;
  }

  public boolean isCoralDetected() {
    return inputs.coralDetected;
  }

  public double returnAppliedVoltage() {
    return inputs.appliedVoltage;
  }

  public void setIntakeVoltage(double voltage) {
    io.setIntakeVoltage(voltage);
  }

  public boolean isDetectingTimed() {
    return (time > ReefDetectorConstants.waitTimeSeconds);
  }
}
