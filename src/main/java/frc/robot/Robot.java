package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frc5010.common.robot.Mode;
import org.frc5010.common.robot.RobotMode;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;

  public Robot() {
    // AdvantageKit Logger setup — must come before Logger.start() and before
    // constructing RobotContainer (which constructs AkitSwerveDrive subsystems).
    Logger.recordMetadata("ProjectName", "FRC5010Claude");
    Logger.recordMetadata("RuntimeType", getRuntimeType().toString());

    if (isReal()) {
      // Real hardware: log to USB drive (/U/logs) and publish over NT4.
      Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new NT4Publisher());
      RobotMode.set(Mode.REAL);
    } else {
      String replayLog = LogFileUtil.findReplayLog();
      if (replayLog != null) {
        // REPLAY mode: re-run robot code against a previously recorded log.
        // Run via: .\gradlew.bat replayWatch (select log in the UI picker)
        RobotMode.set(Mode.REPLAY);
        setUseTiming(false); // run as fast as possible, not real-time
        Logger.setReplaySource(new WPILOGReader(replayLog));
        Logger.addDataReceiver(
            new WPILOGWriter(LogFileUtil.addPathSuffix(replayLog, "_sim")));
      } else {
        // SIM mode: write log to logs/ for later analysis and publish over NT4.
        Logger.addDataReceiver(new WPILOGWriter("logs"));
        Logger.addDataReceiver(new NT4Publisher());
        RobotMode.set(Mode.SIM);
      }
    }

    Logger.start();
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    // CommandScheduler.run() calls periodic() and (in sim) simulationPeriodic()
    // on every registered subsystem, then executes scheduled commands.
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    if (RobotBase.isSimulation()) {
      m_robotContainer.resetToAllianceStart();
    }
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    if (RobotBase.isSimulation()) {
      m_robotContainer.resetToAllianceStart();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {
    // When launched with -PvisualTest (Gradle forwards it as -DvisualTest=true),
    // automatically enable the Driver Station in autonomous so the sequence starts
    // without the user having to click Enable in Glass.
    if (Boolean.getBoolean("visualTest")) {
      DriverStationSim.setDsAttached(true);
      DriverStationSim.setEnabled(true);
      DriverStationSim.setAutonomous(true);
      DriverStationSim.notifyNewData();
    }
  }

  @Override
  public void simulationPeriodic() {
    // CommandScheduler already calls AkitSwerveDrive.simulationPeriodic() (which
    // advances the IronMaple physics engine) as part of robotPeriodic() above.
    // Nothing extra needed here.
  }
}
