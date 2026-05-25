package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frc5010.common.robot.Mode;
import org.frc5010.common.robot.RobotMode;
import org.littletonrobotics.junction.LoggedRobot;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;

  public Robot() {
    RobotMode.set(RobotBase.isReal() ? Mode.REAL : Mode.SIM);
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
