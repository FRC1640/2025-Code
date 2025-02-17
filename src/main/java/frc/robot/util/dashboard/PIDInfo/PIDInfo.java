package frc.robot.util.dashboard.PIDInfo;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer.Subsystems;
import java.util.function.Function;

public class PIDInfo {
  public PIDController controller;
  public String name;
  public Subsystems connectedSubsystem;
  public Function<SubsystemBase, Void> hookedFunction;

  public PIDInfo(PIDController controller, String name, Subsystems connectedSubsystem) {
    this.controller = controller;
    this.name = name;
    this.connectedSubsystem = connectedSubsystem;
  }

  public PIDInfo(
      PIDController controller,
      String name,
      Subsystems connectedSubsystem,
      Function<SubsystemBase, Void> hookedFunction) {
    this.controller = controller;
    this.name = name;
    this.connectedSubsystem = connectedSubsystem;
    this.hookedFunction = hookedFunction;
  }

  public Command pidHookGenerateSetpoint(double setPoint) {
    SubsystemRegistry.registry.get(connectedSubsystem).runOnce(null);
    Command command =
        new Command() {
          @Override
          public void initialize() {
            if (controller.atSetpoint()) {
              this.end(false);
            }
          }

          @Override
          public void execute() {}
        };
    command.addRequirements(SubsystemRegistry.registry.get(connectedSubsystem));
    return command;
  }
}
