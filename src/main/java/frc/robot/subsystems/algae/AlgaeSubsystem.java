package frc.robot.subsystems.algae;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants.AlgaeConstants;
import frc.robot.util.misc.EMA;
import org.littletonrobotics.junction.Logger;

public class AlgaeSubsystem extends SubsystemBase {
  private AlgaeIO io;
  private AlgaeIOInputsAutoLogged inputs = new AlgaeIOInputsAutoLogged();
  private boolean hasAlgae = false;
  private double releaseTime = 0.0;
  private double lastTime = 0.0;
  private EMA emaCurrent = new EMA(AlgaeConstants.emaSmoothing, AlgaeConstants.emaPeriod);
  // bwaahaha not rolling average. its actually an Exponential Moving Average

  public AlgaeSubsystem(AlgaeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Algae", inputs);

    // Update the EMACurrent
    emaCurrent.update((inputs.intakeMotorLeftCurrent + inputs.intakeMotorRightCurrent) / 2);

    Logger.recordOutput("Algae/EMACurrent", emaCurrent.get());

    if (algaeCurrentHit() && !hasAlgae) {
      hasAlgae = true;
    } else if (inputs.intakeMotorRightVoltage < 0 || inputs.intakeMotorLeftVoltage < 0) {
      releaseTime += (System.currentTimeMillis() - lastTime) / 1000;
      if (releaseTime > 0.8) {
        hasAlgae = false;
        releaseTime = 0;
      }
    } else {
      releaseTime = 0;
    }
    lastTime = System.currentTimeMillis();

    Logger.recordOutput("Sensors/HasAlgae", hasAlgae());
  }

  public double getIntakeMotorLeftVoltage() {
    return inputs.intakeMotorLeftVoltage;
  }

  public double getIntakeMotorLeftVelocity() {
    return inputs.intakeMotorLeftVelocity;
  }

  public double getIntakeMotorLeftCurrent() {
    return inputs.intakeMotorLeftCurrent;
  }

  public double getIntakeMotorRightVoltage() {
    return inputs.intakeMotorRightVoltage;
  }

  public double getIntakeMotorRightVelocity() {
    return inputs.intakeMotorRightVelocity;
  }

  public double getIntakeMotorRightCurrent() {
    return inputs.intakeMotorRightCurrent;
  }

  public boolean getSolenoid() {
    return inputs.solenoidForward;
  }

  public void setSolenoid(boolean set) {
    io.setSolenoid(set);
  }

  public void setVoltage(double left, double right) {
    io.setVoltage(left, right);
  }

  public boolean algaeCurrentHit() {
    return emaCurrent.get() > AlgaeConstants.currentThresh || io.hasSimAlgae();
  }

  public boolean setHasAlgae(boolean has) {
    hasAlgae = has;
    return hasAlgae;
  }

  public boolean hasAlgae() {
    return hasAlgae;
  }
}
