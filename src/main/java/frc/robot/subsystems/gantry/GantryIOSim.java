package frc.robot.subsystems.gantry;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.RobotConstants.GantryConstants;

public class GantryIOSim implements GantryIO {
  private final DCMotorSim carriageSim;
  private double gantryAppliedVolts = 0.0;

  public GantryIOSim() {
    DCMotor gantryGearbox = DCMotor.getNeo550(1); // what are we actually using here?
    carriageSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gantryGearbox, 0, GantryConstants.gantryGearRatio),
            gantryGearbox);
  }

  @Override
  public void updateInputs(GantryIOInputs inputs) {
    inputs.currentAmps = carriageSim.getCurrentDrawAmps();
    inputs.appliedVoltage = gantryAppliedVolts;

    carriageSim.setInputVoltage(gantryAppliedVolts);
    carriageSim.update(.02);
  }

  @Override
  public void setGantryVoltage(double voltage) {
    gantryAppliedVolts = voltage;
  }
}
