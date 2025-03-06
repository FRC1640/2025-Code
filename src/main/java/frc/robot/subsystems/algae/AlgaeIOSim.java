package frc.robot.subsystems.algae;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import frc.robot.constants.RobotConstants.AlgaeConstants;
import frc.robot.constants.RobotPIDConstants;
import java.util.function.BooleanSupplier;

public class AlgaeIOSim implements AlgaeIO {
  private final DCMotorSim motorLeftSim;
  private final DCMotorSim motorRightSim;
  private final DoubleSolenoidSim solenoidSim;
  private final BooleanSupplier hasSimAlgae;
  private final SimpleMotorFeedforward ff =
      RobotPIDConstants.constructFFSimpleMotor(RobotPIDConstants.algaeFF, "algaeFF");
  private final PIDController algaeVelocityPID =
      RobotPIDConstants.constructPID(RobotPIDConstants.algaeVelocityPID, "algaeVelocityPID");

  public AlgaeIOSim(BooleanSupplier hasSimAlgae) {
    this.hasSimAlgae = hasSimAlgae;
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
  public void setVelocity(double leftVel, double rightVel, AlgaeIOInputs inputs) {
    setVoltage(
        (ff.calculate(leftVel)
            + algaeVelocityPID.calculate(inputs.intakeMotorLeftVelocity, leftVel)),
        (ff.calculate(rightVel)
            + algaeVelocityPID.calculate(inputs.intakeMotorRightVelocity, rightVel)));
  }

  @Override
  public boolean hasSimAlgae() {
    return hasSimAlgae.getAsBoolean();
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
