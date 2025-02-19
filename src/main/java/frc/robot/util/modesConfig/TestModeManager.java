package frc.robot.util.modesConfig;

public class TestModeManager {
  protected static boolean commandEnabled = false;
  public static void setCommandEnabled(boolean enabled) {
    commandEnabled = enabled;
  }
  public static boolean isCommandEnabled() {
    return commandEnabled;
  }
}
