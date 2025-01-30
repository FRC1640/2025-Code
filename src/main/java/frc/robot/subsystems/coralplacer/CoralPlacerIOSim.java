package frc.robot.subsystems.coralplacer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.RobotConstants.GantryConstants;

public class CoralPlacerIOSim implements CoralPlacerIO {
  private final DCMotorSim intakeSim;
  private double appliedVoltage;

  public CoralPlacerIOSim() {
    DCMotor intakeGearbox = DCMotor.getNeo550(1);
    intakeSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                intakeGearbox, 0.00019125, GantryConstants.gantryGearRatio),
            intakeGearbox);
  }

  @Override
  public void updateInputs(CoralPlacerIOInputs inputs) {
    intakeSim.setInputVoltage(appliedVoltage);
    intakeSim.update(.02);
  }

  @Override
  public void setIntakeVoltage(double voltage) {
    appliedVoltage = voltage;
  }
}
