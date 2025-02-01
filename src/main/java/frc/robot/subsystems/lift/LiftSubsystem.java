package frc.robot.subsystems.lift;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.sysid.SimpleMotorSysidRoutine;
import org.littletonrobotics.junction.Logger;

public class LiftSubsystem extends SubsystemBase {
  LiftIO liftIO;
  LiftIOInputsAutoLogged inputs = new LiftIOInputsAutoLogged();
  SysIdRoutine sysIdRoutine;

  private Mechanism2d liftMechanism = new Mechanism2d(3, 3);
  MechanismLigament2d liftHeight = new MechanismLigament2d("lift", 2, 90);

  public LiftSubsystem(LiftIO liftIO) {
    this.liftIO = liftIO;
    MechanismRoot2d liftMechanismRoot = liftMechanism.getRoot("lift base", 1, 0);
    liftMechanismRoot.append(liftHeight);
    sysIdRoutine =
        new SimpleMotorSysidRoutine()
            .createNewRoutine(
                this::setLiftVoltage,
                this::getLeaderMotorVoltage,
                this::getFollowerMotorPosition,
                this::getFollowerMotorVelocity,
                this,
                new SysIdRoutine.Config(Volts.per(Seconds).of(2), Volts.of(8), Seconds.of(5)));
  }

  @Override
  public void periodic() {
    inputs.leaderMotorPosition = 1.5;
    liftHeight.setLength(inputs.leaderMotorPosition); // conversion?
    liftIO.updateInputs(inputs);
    SmartDashboard.putData("lift :/", liftMechanism); // to be unstipidified
    // Logger.recordOutput("Lift/mechanism", liftMechanism);
    Logger.processInputs("Lift/", inputs);
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
    liftIO.setLiftPosition(pos, inputs);
  }

  public void setLiftVoltage(double voltage) {
    liftIO.setLiftVoltage(voltage, inputs);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  public void runLiftMotionProfile(double pos) {
    liftIO.setLiftPositionMotionProfile(pos, inputs);
  }

  public void resetLiftMotionProfile() {
    liftIO.resetLiftMotionProfile(inputs);
  }
}
