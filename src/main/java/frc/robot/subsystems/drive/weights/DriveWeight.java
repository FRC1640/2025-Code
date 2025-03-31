package frc.robot.subsystems.drive.weights;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface DriveWeight {

  public ChassisSpeeds getSpeeds();

  public default double getWeight() {
    return 1;
  }

  public default void setWeight(double weight) {}

  public default boolean cancelCondition() {
    return false;
  }

  public default boolean isEnabled() {
    return true;
  }

  public default void onStart() {}
  public default void onFinish() {}
}
