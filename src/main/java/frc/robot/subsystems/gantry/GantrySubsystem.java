package frc.robot.subsystems.gantry;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  public GantrySubsystem(GantryIO io) {
    this.io = io;
    LoggedMechanismRoot2d gantryRoot = gantryMechanism.getRoot("gantry root", 0, 1);
    gantryRoot.append(gantryPos);
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
    System.out.println("gaaa");
    return inputs.appliedVoltage;
  }

  public double getCarriagePosition() {
    return inputs.encoderPosition;
  }

  public boolean isLimitSwitchPressed() {
    System.out.println("goo");
    return inputs.isLimitSwitchPressed;
  }

  public void setCarriagePosition(double pos) {
    System.out.println("snarp");
    io.setGantryPosition(pos, inputs);
  }

  public void setGantryVoltage(double voltage) {
    System.out.println("hhee");
    io.setGantryVoltage(voltage, inputs);
  }

  public void resetEncoder() {
    System.out.println("reset");
    io.resetEncoder();
  }
}
