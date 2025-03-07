package frc.robot.subsystems.coralouttake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants.ReefDetectorConstants;
import org.littletonrobotics.junction.Logger;

public class CoralOuttakeSubsystem extends SubsystemBase {

  CoralOuttakeIOInputsAutoLogged inputs = new CoralOuttakeIOInputsAutoLogged();
  CoralOuttakeIO io;
  private double time = 0;
  private double lastTime = 0.0;
  private double releaseTime = 0.0;
  private boolean hasCoral;

  public CoralOuttakeSubsystem(CoralOuttakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/", inputs);
    if (hasCoral()) {
      time += (System.currentTimeMillis() - lastTime) / 1000;
    } else {
      time = 0;
    }

    if (returnAppliedVoltage() > 0.5 && hasCoral()) {
      releaseTime += (System.currentTimeMillis() - lastTime) / 1000;
    }
    if (releaseTime > 0.4) {
      hasCoral = false;
      releaseTime = 0;
    }

    lastTime = System.currentTimeMillis();
    Logger.recordOutput("Sensors/CoralDetector/DetectionTime", time);
    Logger.recordOutput("Sensors/CoralDetector/DetectionTimeBool", isDetectingTimed());
    Logger.recordOutput("Sensors/CoralDetector/HasCoral", hasCoral);
  }

  public void stop() {
    io.setIntakeVoltage(0);
  }

  public double returnTemp() {
    return inputs.tempCelcius;
  }

  public void setHasCoral(boolean coral) {
    hasCoral = coral;
  }

  public boolean isCoralDetected() {
    return inputs.coralDetectedHigh;
  }

  public double returnAppliedVoltage() {
    return inputs.appliedVoltage;
  }

  public void setIntakeVoltage(double voltage) {
    io.setIntakeVoltage(voltage);
  }

  public void setIntakeVelocity(double velocity) {
    io.setIntakeVelocity(velocity, inputs);
  }

  public double getVelocity() {
    return inputs.outtakeVelocity;
  }

  public boolean isDetectingTimed() {
    return (time > ReefDetectorConstants.waitTimeSeconds);
  }

  public boolean hasCoral() {
    return inputs.hasCoral;
  }
}
