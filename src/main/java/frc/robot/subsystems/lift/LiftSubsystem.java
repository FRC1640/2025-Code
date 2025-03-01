package frc.robot.subsystems.lift;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.logging.LogRunner;
import frc.robot.util.logging.VelocityLogStorage;
import frc.robot.util.sysid.SimpleMotorSysidRoutine;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class LiftSubsystem extends SubsystemBase {
  LiftIO io;
  LiftIOInputsAutoLogged inputs = new LiftIOInputsAutoLogged();
  // boolean limit = false;
  SysIdRoutine sysIdRoutine;

  private LoggedMechanism2d liftMechanism = new LoggedMechanism2d(3, 3);
  LoggedMechanismLigament2d liftHeight = new LoggedMechanismLigament2d("lift", 2, 90);

  public LiftSubsystem(LiftIO liftIO) {
    this.io = liftIO;
    LoggedMechanismRoot2d liftMechanismRoot = liftMechanism.getRoot("lift base", 1, 0);
    liftMechanismRoot.append(liftHeight);
    sysIdRoutine =
        new SimpleMotorSysidRoutine()
            .createNewRoutine(
                this::setLiftVoltage,
                this::getLeaderMotorVoltage,
                this::getLeaderMotorPosition,
                this::getLeaderMotorVelocity,
                this,
                new SysIdRoutine.Config(
                    Volts.per(Seconds).of(0.5),
                    Volts.of(3.5),
                    Seconds.of(100),
                    (state) -> Logger.recordOutput("SysIdTestState", state.toString())));
  }

  @Override
  public void periodic() {
    liftHeight.setLength(getLeaderMotorPosition()); // conversion?
    io.updateInputs(inputs);
    Logger.recordOutput("Mechanisms/Lift", liftMechanism);
    Logger.processInputs("Lift/", inputs);
    LogRunner.addLog(
        new VelocityLogStorage(
            () -> getLeaderMotorVelocity(), () -> io.velocitySetpoint(), getName()));
  }

  public double getMotorPosition() {
    return inputs.motorPosition;
  }

  public double getLeaderMotorPosition() {
    return inputs.leaderMotorPosition;
  }

  public double getFollowerMotorPosition() {
    return inputs.followerMotorPosition;
  }

  public double getLeaderMotorVelocity() {
    return inputs.leaderMotorVelocity;
  }

  public double getFollowerMotorVelocity() {
    return inputs.followerMotorVelocity;
  }

  public double getLeaderMotorCurrent() {
    return inputs.leaderMotorCurrent;
  }

  public double getFollowerMotorCurrent() {
    return inputs.followerMotorCurrent;
  }

  public double getLeaderMotorVoltage() {
    return inputs.leaderMotorVoltage;
  }

  public double getLeaderTemperature() {
    return inputs.leaderTemperature;
  }

  public double getFollowerTemperature() {
    return inputs.followerTemperature;
  }

  public void setLiftPosition(double pos) {
    io.setLiftPosition(pos, inputs);
  }

  public void setLiftVoltage(double voltage) {
    io.setLiftVoltage(voltage, inputs);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  public void runLiftMotionProfile(double pos) {
    io.setLiftPositionMotionProfile(pos, inputs);
  }

  public void resetLiftMotionProfile() {
    io.resetLiftMotionProfile(inputs);
  }

  public void resetEncoder() {
    io.resetEncoder();
  }

  public boolean isAtPreset(double pos) {
    return Math.abs(getMotorPosition() - pos) < 0.02;
  }

  public boolean isLimitSwitchPressed() {
    return inputs.isLimitSwitchPressed;
  }

  public void setLimitEnabled(boolean enable) {
    io.setLimitEnabled(enable);
  }
}
