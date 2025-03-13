// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import au.grapplerobotics.CanBridge;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.RobotConstants.MotorInfo;
import frc.robot.constants.RobotConstants.RobotConfigConstants;
import frc.robot.constants.RobotConstants.TestConfig;
import frc.robot.subsystems.drive.commands.DriveWeightCommand;
import frc.robot.subsystems.drive.weights.PathplannerWeight;
import frc.robot.util.dashboard.Dashboard;
import frc.robot.util.logging.LoggerManager;
import frc.robot.util.periodic.PeriodicScheduler;
import frc.robot.util.robotswitch.RobotSwitchManager.RobotType;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.Collection;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  Collection<Runnable> r;

  private final RobotContainer m_robotContainer;

  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  };

  public static enum RobotState {
    DISABLED,
    AUTONOMOUS,
    TELEOP,
    TEST
  }

  public static RobotState state = RobotState.DISABLED;

  public static RobotState getState() {
    return state;
  }

  public Robot() {
    CanBridge.runTCP();
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
    Logger.recordMetadata("RobotMode", getMode().toString());
    // Logger.recordMetadata("MACAddress", getMACAddress());
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }
    System.out.println(getMode().toString());
    // Set up data receivers & replay source
    switch (getMode()) {
        // Running on a real robot, log to a USB stick
      case REAL:
        LoggedPowerDistribution.getInstance(21, ModuleType.kRev);
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());

        break;

        // Running a physics simulator, log to local folder
      case SIM:
        Logger.addDataReceiver(new WPILOGWriter("logs"));
        Logger.addDataReceiver(new NT4Publisher());
        break;

        // Replaying a log, set up replay source
      case REPLAY:
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay")));
        break;
    }

    // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
    // Logger.getInstance().disableDeterministicTimestamps()
    // Register URCL
    Logger.registerURCL(URCL.startExternal(MotorInfo.motorLoggingManager.getMap()));

    // Start AdvantageKit Logger
    Logger.start();

    m_robotContainer = new RobotContainer();

    WebServer.start(
        5800,
        Filesystem.getDeployDirectory()
            .getPath()); // instructed to add to get elastic config to load automatically
  }

  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();
    FollowPathCommand.warmupCommand().schedule();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    PeriodicScheduler.getInstance().run();
    LoggerManager.updateLog();
  }

  @Override
  public void disabledInit() {
    DriveWeightCommand.removeAllWeights();
    PathplannerWeight.setSpeeds(new ChassisSpeeds());
    state = RobotState.DISABLED;
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    state = RobotState.AUTONOMOUS;

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    state = RobotState.TELEOP;
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void simulationPeriodic() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    state = RobotState.TEST;
    switch (TestConfig.tuningMode) {
      case sysIDTesting:
        CommandScheduler.getInstance().cancelAll();
        Dashboard.getSysidCommand().schedule();
        CommandScheduler.getInstance().getActiveButtonLoop().clear();
        break;
      default:
        LiveWindow.setEnabled(false);
        CommandScheduler.getInstance().enable();
        break;
    }
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  public static boolean isReplay() {
    if (RobotConfigConstants.robotType == RobotType.Replay) {
      return true;
    }
    String replay = System.getProperty("REPLAY");
    return replay != null && replay.toLowerCase().equals("true");
  }

  public static Mode getMode() {
    if (isReal()) {
      return Mode.REAL;
    }
    if (isReplay()) {
      return Mode.REPLAY;
    }

    return Mode.SIM;
  }

  public static String getMACAddress() {
    try {
      InetAddress localHost = InetAddress.getLocalHost();
      NetworkInterface ni = NetworkInterface.getByInetAddress(localHost);
      byte[] hardwareAddress = ni.getHardwareAddress();
      String[] hexadecimal = new String[hardwareAddress.length];
      for (int i = 0; i < hardwareAddress.length; i++) {
        hexadecimal[i] = String.format("%02X", hardwareAddress[i]);
      }
      return String.join("-", hexadecimal);
    } catch (UnknownHostException | SocketException e) {
      e.printStackTrace();
      return "Not found";
    }
  }
}
