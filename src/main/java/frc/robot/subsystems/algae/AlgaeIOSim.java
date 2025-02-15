package frc.robot.subsystems.algae;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import frc.robot.constants.RobotConstants.AlgaeConstants;

public class AlgaeIOSim implements AlgaeIO {
  private final DCMotorSim motorLeftSim;
  private final DCMotorSim motorRightSim;
  private final DoubleSolenoidSim solenoidSim;

  public AlgaeIOSim() {
    DCMotor motorSimGearbox = DCMotor.getNEO(1);

    motorLeftSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                motorSimGearbox, 0.00019125, AlgaeConstants.gearRatio),
            motorSimGearbox);
    motorRightSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                motorSimGearbox, 0.00019125, AlgaeConstants.gearRatio),
            motorSimGearbox);
    solenoidSim = new DoubleSolenoidSim(PneumaticsModuleType.REVPH, 0, 1);
  }

  @Override
  public void setSolenoid(boolean set) {
    solenoidSim.set(set ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void setVoltage(double left, double right) {
    motorLeftSim.setInputVoltage(left);
    motorRightSim.setInputVoltage(right);
  }

  @Override
  public void updateInputs(AlgaeIOInputs inputs) {
    motorLeftSim.update(0.02);
    motorRightSim.update(0.02);

    inputs.intakeMotorLeftVoltage = motorLeftSim.getInputVoltage();
    inputs.intakeMotorLeftVelocity = motorLeftSim.getAngularVelocityRadPerSec();
    inputs.intakeMotorLeftCurrent = motorLeftSim.getCurrentDrawAmps();

    inputs.intakeMotorRightVoltage = motorRightSim.getInputVoltage();
    inputs.intakeMotorRightVelocity = motorRightSim.getAngularVelocityRadPerSec();
    inputs.intakeMotorRightCurrent = motorRightSim.getCurrentDrawAmps();

    inputs.solenoidForward = solenoidSim.get() == DoubleSolenoid.Value.kForward;
  }
}
