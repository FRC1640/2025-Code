package frc.robot.sensors.apriltag;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.constants.CameraConstant;
import frc.robot.constants.FieldConstants;
import frc.robot.sensors.apriltag.AprilTagVisionIO.PoseObservation;
import frc.robot.sensors.apriltag.AprilTagVisionIO.TrigTargetObservation;
import frc.robot.sensors.odometry.RobotOdometry;
import frc.robot.util.alerts.AlertsManager;
import frc.robot.util.periodic.PeriodicBase;
import java.util.ArrayList;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends PeriodicBase {
  AprilTagVisionIO io;
  AprilTagVisionIOInputsAutoLogged inputs;
  private String cameraName;
  public final double standardDeviation;

  public double getStandardDeviation() {
    return standardDeviation;
  }

  public AprilTagVision(AprilTagVisionIO io, CameraConstant cameraConstants) {
    this.io = io;
    cameraName = cameraConstants.name;
    this.inputs = new AprilTagVisionIOInputsAutoLogged();
    this.standardDeviation = cameraConstants.standardDevConstant;
    AlertsManager.addAlert(
        () -> !inputs.connected, "April tag vision disconnected.", AlertType.kError);
  }

  public Optional<Pose2d> calculateTrigResults(Rotation3d gyroRotation) {
    // trig solution
    ArrayList<PoseObservation> trigPoses = new ArrayList<>();
    if (inputs.trigTargetObservations.length == 0) {
      return Optional.empty();
    }
    for (TrigTargetObservation observation : inputs.trigTargetObservations) {
      trigPoses.add(calculateTrigResult(observation, gyroRotation));
    }
    double bestDist = Double.MAX_VALUE;
    Pose2d bestPose = new Pose2d();
    for (PoseObservation poseObservation : trigPoses) {
      if (poseObservation.averageTagDistance() < bestDist) {
        bestDist = poseObservation.averageTagDistance();
        bestPose = poseObservation.pose().toPose2d();
      }
    }
    // Logger.recordOutput(cameraName, null);
    return Optional.of(bestPose);
  }

  public PoseObservation calculateTrigResult(
      TrigTargetObservation observation, Rotation3d gyroRotation) {
    // calculate
    double deltaH = observation.targetTransform().getZ() - io.getCameraDisplacement().getZ();
    double distance2d = observation.distance2D();
    double x = distance2d * Math.cos((observation.tx().getRadians() - gyroRotation.getZ()));
    double y = distance2d * Math.sin(-(observation.tx().getRadians() - gyroRotation.getZ()));
    double z = 0;
    // calculate transforms
    Transform3d cameraToTarget = new Transform3d(new Translation3d(x, y, z), new Rotation3d());
    Transform3d fieldToCamera =
        new Transform3d(observation.targetTransform().getTranslation(), new Rotation3d())
            .plus(cameraToTarget.inverse());
    Transform3d fieldToRobot = fieldToCamera.plus(inputs.cameraDisplacement.inverse());

    // convert and record
    Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), gyroRotation);
    return new PoseObservation(observation.timestamp(), robotPose, 0, 1, observation.distance());
  }

  public PoseObservation[] getPhotonResults() {
    return inputs.photonPoseObservations;
  }

  public boolean isConnected() {
    return inputs.connected;
  }

  public String getCameraName() {
    return cameraName;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("AprilTagVision/" + cameraName, inputs);
    ArrayList<Pose3d> tagPoses = new ArrayList<>();
    for (int i = 0; i < inputs.tagIds.length; i++) {
      Optional<Pose3d> pose = FieldConstants.aprilTagLayout.getTagPose(inputs.tagIds[i]);
      if (pose.isPresent()) {
        tagPoses.add(pose.get());
      }
    }
    Logger.recordOutput(
        "Drive/Odometry/Vision/Camera_" + cameraName + "/TagPoses",
        tagPoses.toArray(Pose3d[]::new));
    // testing
    Rotation2d gyroRotation = RobotOdometry.instance.getPose("Main").getRotation();
    Optional<Pose2d> robotTrig =
        calculateTrigResults(new Rotation3d(0, 0, gyroRotation.getRadians()));

    Pose2d robotPose = robotTrig.orElse(null);
    Logger.recordOutput("AprilTagVision/" + cameraName + "/TrigEstimate/RobotPose", robotPose);
  }
}
