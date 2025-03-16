package frc.robot.subsystems.gantry.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants.GantryConstants;
import frc.robot.constants.RobotConstants.LiftConstants.CoralPreset;
import frc.robot.constants.RobotConstants.LiftConstants.GantrySetpoint;
import frc.robot.sensors.reefdetector.ReefDetector;
import frc.robot.subsystems.gantry.GantrySubsystem;
import frc.robot.util.misc.AllianceManager;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class GantryCommandFactory {
  GantrySubsystem gantrySubsystem;
  private ReefDetector reefDetector;
  private boolean threshSet = false;
  private boolean direction = false;

  public GantryCommandFactory(GantrySubsystem gantrySubsystem, ReefDetector reefDetector) {
    this.gantrySubsystem = gantrySubsystem;
    this.reefDetector = reefDetector;
  }

  public Command gantryPIDCommand(DoubleSupplier pos) {
    return new RunCommand(
            () -> gantrySubsystem.setCarriagePosition(pos.getAsDouble()), gantrySubsystem)
        // .until(() -> Math.abs(gantrySubsystem.getCarriagePosition() - pos.getAsDouble()) < 0.01)
        .finallyDo(() -> gantrySubsystem.setGantryVoltage(0));
  }

  public Command gantryApplyVoltageCommand(DoubleSupplier voltage) {
    return new RunCommand(
            () -> gantrySubsystem.setGantryVoltage(voltage.getAsDouble()), gantrySubsystem)
        .finallyDo(() -> gantrySubsystem.setGantryVoltage(0));
  }

  public Command gantryHomeCommand() {
    return gantryApplyVoltageCommand(() -> -2)
        .repeatedly()
        .until(() -> gantrySubsystem.isLimitSwitchPressed())
        .andThen(
            gantryApplyVoltageCommand(() -> 1)
                .repeatedly()
                .until(() -> !gantrySubsystem.isLimitSwitchPressed()))
        .andThen(
            gantryApplyVoltageCommand(() -> -0.5)
                .repeatedly()
                .until(() -> gantrySubsystem.isLimitSwitchPressed()))
        // .andThen(
        //     gantryApplyVoltageCommand(() -> -GantryConstants.gantryHomeFastVoltage)
        //         .repeatedly()
        //         .until(() -> !gantrySubsystem.isLimitSwitchPressed()))
        .andThen(new InstantCommand(() -> gantrySubsystem.resetEncoder()))
        .finallyDo(() -> gantrySubsystem.setLimitEnabled(true))
        .beforeStarting(() -> gantrySubsystem.setLimitEnabled(false));
  }
  // I dont think the limit switch as the bound is a good idea because then the gantry will be
  // slamming into the limit switch all the time. -> Bad for the limit switch. So I think we
  // should just use the encoder value as the bound.

  public Command gantrySetVelocityCommand(DoubleSupplier velocity) {
    return new RunCommand(
            () -> gantrySubsystem.setVelocity(velocity.getAsDouble()), gantrySubsystem)
        .finallyDo(() -> gantrySubsystem.setGantryVoltage(0));
  }

  public Command gantryDriftCommandMinima() {
    return (gantrySetVelocityCommand(
                () ->
                    gantrySubsystem.getCarriagePosition() < GantryConstants.gantryLimitCenter
                        ? GantryConstants.alignSpeed
                        : -GantryConstants.alignSpeed)
            .alongWith(new RunCommand(() -> reefDetector.reefFind()))
            .until(
                () ->
                    Math.abs(
                            gantrySubsystem.getCarriagePosition()
                                - GantryConstants.gantryLimitCenter)
                        < GantryConstants.gantryPadding)
            .andThen(new InstantCommand(() -> threshSet = true))
            .andThen(
                gantrySetVelocityCommand(
                        () ->
                            gantrySubsystem.getCarriagePosition()
                                    < GantryConstants.gantryLimitCenter
                                ? -GantryConstants.alignSpeed
                                : GantryConstants.alignSpeed)
                    .until(
                        () ->
                            Math.abs(
                                        gantrySubsystem.getCarriagePosition()
                                            - GantryConstants.gantryLimits.high)
                                    < GantryConstants.gantryPadding
                                || Math.abs(
                                        gantrySubsystem.getCarriagePosition()
                                            - GantryConstants.gantryLimits.low)
                                    < GantryConstants.gantryPadding)))
        .andThen(new InstantCommand(() -> reefDetector.reefFindReset()))
        .andThen(new InstantCommand(() -> threshSet = false))
        .repeatedly()
        .until(
            () -> reefDetector.getDistanceToReef() < reefDetector.getFoundThresh() + 6 && threshSet)
        .andThen(
            gantrySetVelocityCommand(() -> 0)
                .until(() -> Math.abs(gantrySubsystem.getGantryVelocity()) < 0.01))
        .finallyDo(
            () -> {
              reefDetector.reefFindReset();
              threshSet = false;
            });
  }

  public Command gantryDriftCommandThresh() {
    return (gantrySetVelocityCommand(
                () ->
                    gantrySubsystem.getCarriagePosition() < GantryConstants.gantryLimitCenter
                        ? GantryConstants.alignSpeed
                        : -GantryConstants.alignSpeed)
            .andThen(
                new InstantCommand(
                    () ->
                        direction =
                            gantrySubsystem.getCarriagePosition()
                                < GantryConstants.gantryLimitCenter))
            .until(
                () ->
                    Math.abs(
                            gantrySubsystem.getCarriagePosition()
                                - GantryConstants.gantryLimitCenter)
                        < GantryConstants.gantryPadding)
            .andThen(
                new InstantCommand(() -> direction = !direction)
                    .andThen(
                        gantrySetVelocityCommand(
                            () ->
                                direction
                                    ? GantryConstants.alignSpeed
                                    : -GantryConstants.alignSpeed))
                    .until(
                        () ->
                            Math.abs(
                                        gantrySubsystem.getCarriagePosition()
                                            - GantryConstants.gantryLimits.high)
                                    < GantryConstants.gantryPadding
                                || Math.abs(
                                        gantrySubsystem.getCarriagePosition()
                                            - GantryConstants.gantryLimits.low)
                                    < GantryConstants.gantryPadding)))
        .repeatedly()
        .until(() -> reefDetector.getDistanceToReef() < 500)
        .andThen(new InstantCommand(() -> gantrySubsystem.setGantryVoltage(0)));
  }

  public Command runGantryMotionProfile(DoubleSupplier pos) {
    return new InstantCommand(() -> gantrySubsystem.resetGantryMotionProfile())
        .andThen(
            new RunCommand(
                    () -> {
                      gantrySubsystem.runGantryMotionProfile(pos.getAsDouble());
                    },
                    gantrySubsystem)
                .finallyDo(
                    () -> {
                      gantrySubsystem.setGantryVoltage(0);
                      gantrySubsystem.resetGantryMotionProfile();
                    }));
  }

  public Command runGantryVelocityMotionProfile(DoubleSupplier vel) {
    return new InstantCommand(() -> gantrySubsystem.resetGantryVelocityMotionProfile())
        .andThen(
            new RunCommand(
                    () -> {
                      gantrySubsystem.runGantryVelocityMotionProfile(vel.getAsDouble());
                    },
                    gantrySubsystem)
                .finallyDo(
                    () -> {
                      gantrySubsystem.setGantryVoltage(0);
                      gantrySubsystem.resetGantryVelocityMotionProfile();
                    }));
  }

  public double getSetpointOdometry(Supplier<CoralPreset> coralPreset, Supplier<Pose2d> getPose) {
    // select reef positions
    Pose2d[] reefPositions =
        AllianceManager.chooseFromAlliance(
            FieldConstants.reefPositionsBlue, FieldConstants.reefPositionsRed);
    // find target face
    Pose2d reefPos = reefPositions[0];
    double closest = Double.MAX_VALUE;
    for (Pose2d face : reefPositions) {
      double distance = face.getTranslation().minus(getPose.get().getTranslation()).getNorm();
      if (distance < closest) {
        closest = distance;
        reefPos = face;
      }
    }
    System.out.println(reefPos);
    // calculate gantry offset
    double gyroRadians = getPose.get().getRotation().getRadians();
    double deltaY = reefPos.getY() - getPose.get().getTranslation().getY();
    double deltaX = reefPos.getX() - getPose.get().getTranslation().getX();
    double gantryCenter =
        Math.cos(gyroRadians + Math.atan(deltaY / deltaX))
            * reefPos
                .getTranslation()
                .getDistance(getPose.get().getTranslation()); // TODO flip gyro sign?
    Logger.recordOutput("A_DEBUG/gyroRadians", gyroRadians);
    Logger.recordOutput("A_DEBUG/deltaY", deltaY);
    Logger.recordOutput("A_DEBUG/deltaX", deltaX);
    Logger.recordOutput("A_DEBUG/atan", Math.atan(deltaY / deltaX));
    Logger.recordOutput(
        "A_DEBUG/translation",
        reefPos.getTranslation().getDistance(getPose.get().getTranslation()));
    Logger.recordOutput("A_DEBUG/robotX", getPose.get().getTranslation().getX());
    Logger.recordOutput("A_DEBUG/robotY", getPose.get().getTranslation().getY());
    double poleOffset =
        coralPreset.get().getGantrySetpoint(AllianceManager.onDsSideReef(getPose))
                == GantrySetpoint.LEFT
            ? -Units.inchesToMeters(13 / 2)
            : Units.inchesToMeters(13 / 2);
    double setpoint = GantryConstants.gantryLimitCenter + gantryCenter + poleOffset;
    Logger.recordOutput("A_DEBUG/setpoint", setpoint);
    return setpoint;
  }

  public void constructTriggers() {}
}
