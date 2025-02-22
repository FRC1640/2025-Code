package frc.robot.util.logging.PIDTracking;

import edu.wpi.first.math.controller.PIDController;

public class TrackedRobotPID {
  public static double calculate(PIDController controller) {
    double p = controller.getError();
    double i = controller.getAccumulatedError();
    double d = controller.getErrorDerivative();
    return controller.getP() * p + controller.getI() * i + controller.getD() * d;
  }

  public static double calculateDerivative(PIDController controller) {
    double d = controller.getErrorDerivative();
    return controller.getD() * d;
  }

  public static double calculateIntegral(PIDController controller) {
    double i = controller.getAccumulatedError();
    return controller.getI() * i;
  }

  public static double calculateProportional(PIDController controller) {
    double p = controller.getError();
    return controller.getP() * p;
  }
}
