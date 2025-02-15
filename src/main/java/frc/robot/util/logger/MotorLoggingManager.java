package frc.robot.util.logger;

import java.util.HashMap;
import java.util.Map;

public class MotorLoggingManager {
  public Map<Integer, String> motorNameMap = new HashMap<>();

  public MotorLoggingManager addMotorAlias(int id, String alais) {
    motorNameMap.put(id, alais);
    return this;
  }

  public Map<Integer, String> getMap() {
    return motorNameMap;
  }
}
