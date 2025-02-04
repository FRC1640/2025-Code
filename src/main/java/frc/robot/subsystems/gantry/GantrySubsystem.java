package frc.robot.subsystems.gantry;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.sysid.SimpleMotorSysidRoutine;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class GantrySubsystem extends SubsystemBase {
  GantryIOInputsAutoLogged inputs = new GantryIOInputsAutoLogged();
  GantryIO io;

  private LoggedMechanism2d gantryMechanism =
      new LoggedMechanism2d(2, 2); // represents gantry position left/right

  private LoggedMechanismLigament2d gantryPos = new LoggedMechanismLigament2d("gantry pos", 1, 0);

  SysIdRoutine sysIdRoutine;

  public GantrySubsystem(GantryIO io) {
    this.io = io;
    LoggedMechanismRoot2d gantryRoot = gantryMechanism.getRoot("gantry root", 0, 1);
    gantryRoot.append(gantryPos);

    sysIdRoutine =
        new SimpleMotorSysidRoutine()
            .createNewRoutine(
                this::setGantryVoltage,
                this::getGantryVoltage,
                this::getCarriagePosition,
                this::getGantryVelocity,
                this,
                new SysIdRoutine.Config(Volts.per(Seconds).of(0.5), Volts.of(4), Seconds.of(3)));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Gantry/", inputs);
    gantryPos.setLength(getCarriagePosition()); // conversion?
    Logger.recordOutput("Mechanisms/Gantry", gantryMechanism);
  }

  public void stop() {
    io.setGantryVoltage(0, inputs);
  }

  public double getGantryVoltage() {
    return inputs.appliedVoltage;
  }

  public double getGantryVelocity() {
    return inputs.encoderVelocity;
  }

  public double getCarriagePosition() {
    return inputs.encoderPosition;
  }

  public boolean isLimitSwitchPressed() {
    return inputs.isLimitSwitchPressed;
  }

  public void setCarriagePosition(double pos) {
    io.setGantryPosition(pos, inputs);
  }

  public void setGantryVoltage(double voltage) {
    io.setGantryVoltage(voltage, inputs);
  }

  public void resetEncoder() {
    io.resetEncoder();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  public void setVelocity(double velocity) {
    io.setGantryVelocity(velocity, inputs);
  }
}
