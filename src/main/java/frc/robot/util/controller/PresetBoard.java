package frc.robot.util.controller;

import edu.wpi.first.wpilibj.GenericHID;

public class PresetBoard extends GenericHID {
  public static class Button {
    public static int kLl3 = 1;
    public static int kLl2 = 2;
    public static int kRl3 = 3;
    public static int kRl2 = 4;
    public static int kRl4 = 5;
    public static int kTrough = 6;
    public static int kShare = 7;
    public static int kOptions = 8;
    public static int kLeft3 = 9;
    public static int kRight3 = 10;
  }

  public static class Axis {
    public static int kXAxis = 0;
    public static int kYAxis = 1;
    public static int kLTAxis = 2;
    public static int kRTAxis = 3;
  }

  public PresetBoard(int port) {
    super(port);
  }

  public boolean getLl3() {
    return super.getRawButton(Button.kLl3);
  }

  public boolean getTroph(){
    return super.getRawButton(Button.kTrough);
  }

  public boolean getLl2() {
    return super.getRawButton(Button.kLl2);
  }

  public boolean getRl3() {
    return super.getRawButton(Button.kRl3);
  }

  public boolean getRl2() {
    return super.getRawButton(Button.kRl2);
  }

  public boolean getRl4() {
    return super.getRawButton(Button.kRl4);
  }

  public boolean getLl4() {
    return super.getRawAxis(Axis.kLTAxis) == 1;
  }

  public boolean povIsActive() {
    return povIsActive(getPOV());
  }

  public boolean povIsUpwards() {
    int pov = getPOV();
    return povIsActive(pov) && (pov <= 45 || pov >= 315);
  }

  public boolean povIsDownwards() {
    int pov = getPOV();
    return povIsActive(pov) && (pov >= 135 && pov <= 225);
  }

  public boolean povIsRightwards() {
    int pov = getPOV();
    return povIsActive(pov) && (pov >= 45 && pov <= 135);
  }

  public boolean povIsLeftwards() {
    int pov = getPOV();
    return povIsActive(pov) && (pov >= 225 && pov <= 315);
  }

  /*
   * Can be used to treat an axis like a button.
   */
  public boolean getAxisButton(int axis) {
    double value = getRawAxis(axis);
    return (value >= 0.5) || (value <= -0.5);
  }

  private boolean povIsActive(int pov) {
    return pov >= 0;
  }
}
