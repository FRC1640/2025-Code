package frc.robot.subsystems.gantry;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class GantrySubsystem extends SubsystemBase {
  GantryIOInputsAutoLogged inputs = new GantryIOInputsAutoLogged();
  GantryIO io;

  private Mechanism2d gantryMechanism =
      new Mechanism2d(2, 2); // represents gantry position left/right
  private MechanismLigament2d gantryPos = new MechanismLigament2d("gantry pos", 1, 0);

  public GantrySubsystem(GantryIO io) {
    this.io = io;
    MechanismRoot2d gantryRoot = gantryMechanism.getRoot("gantry root", 0, 1);
    gantryRoot.append(gantryPos);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Gantry/", inputs);
    inputs.encoderPosition = .5;
    gantryPos.setLength(inputs.encoderPosition); // conversion?
    SmartDashboard.putData("gantry mechanism weee", gantryMechanism);
  }

  public void stop() {
    io.setGantryVoltage(0, inputs);
  }

  public double getGantryVoltage() {
    return inputs.appliedVoltage;
  }

  public double getCarriagePosition() {
    return inputs.encoderPosition;
  }

  public void setCarriagePosition(double pos) {
    io.setGantryPosition(pos, inputs);
  }

  public void setGantryVoltage(double voltage) {
    io.setGantryVoltage(voltage, inputs);
  }
}
