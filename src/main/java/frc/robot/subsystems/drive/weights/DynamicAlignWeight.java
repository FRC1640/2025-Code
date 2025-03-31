package frc.robot.subsystems.drive.weights;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DynamicAlignWeight implements DriveWeight {
  private FollowPathNearest globalAlign;
  private LocalTagAlignWeight localAlign;

  public DynamicAlignWeight(FollowPathNearest globalAlign, LocalTagAlignWeight localAlign) {
    this.globalAlign = globalAlign;
    this.localAlign = localAlign;
  }

  @Override
  public ChassisSpeeds getSpeeds() {
    boolean[] conditions = {
      localAlign.isReady(), globalAlign.isEnabled(), !globalAlign.isAutoalignComplete()
    };
    System.out.println(conditions[0] + " " + conditions[1] + " " + conditions[2]);
    if (conditions[0]) {
      globalAlign.stopPath();
      return localAlign.getSpeeds();
    } else if (conditions[2]) {
      if (!globalAlign.isEnabled()) {
        globalAlign.startPath();
      }
      return new ChassisSpeeds();
    } else {
      globalAlign.stopPath();
      return new ChassisSpeeds();
    }
  }

  public boolean globalAlignComplete() {
    return globalAlign.isAutoalignComplete();
  }

  @Override
  public void onFinish() {
    globalAlign.stopPath();
  }
}
