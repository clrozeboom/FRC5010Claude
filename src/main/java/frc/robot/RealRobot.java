package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.LinkedHashMap;
import java.util.Map;
import org.frc5010.common.profiles.SwerveRobotContainer;

/**
 * Team-specific robot container — extends {@link SwerveRobotContainer} with
 * competition hardware constants ({@link RealRobotProfile}) and the {@link DemoIntake} demo.
 *
 * <p>Profile selection is handled by {@link SwerveRobotContainer#selectProfile(String)}: pass
 * the fully-qualified class name of the desired profile. To use a custom profile that extends
 * {@link RealRobotProfile}, just change the class name string here.
 *
 * <p>Note: {@link SwerveRobotContainer}'s constructor calls {@link #configureBindings()} before
 * this class's constructor body runs. Fields initialised after {@code super(...)} would be
 * {@code null} when {@code configureBindings()} fires — initialise such fields inside
 * {@code configureBindings()} itself (as {@code demoIntake} is here).
 */
public class RealRobot extends SwerveRobotContainer {

  private DemoIntake demoIntake;
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /** Auto routines by display name, in chooser order. Mirrored to Glass and the web UI. */
  private final LinkedHashMap<String, Command> autos = new LinkedHashMap<>();

  /**
   * The auto selected from the web UI (null until the user picks one). Written on the robot
   * thread by the {@code WebControl} select callback; read by {@link #getAutonomousCommand()}.
   */
  private volatile String webSelectedAuto;

  public RealRobot() {
    super(SwerveRobotContainer.selectProfile("frc.robot.RealRobotProfile"));

    autos.put("None", Commands.none());
    autos.put("BLine: Example Score (JSON)", AutoRoutines.exampleScore(drive));
    autos.put("BLine: Example Score (code)", AutoRoutines.exampleScoreInCode(drive));
    if (demoIntake != null) {
      autos.put("BLine: Pickup + Score", AutoRoutines.pickupAndScore(drive, demoIntake));
    }

    // Populate the Glass SendableChooser from the same ordered map (first entry = default).
    boolean first = true;
    for (Map.Entry<String, Command> e : autos.entrySet()) {
      if (first) { autoChooser.setDefaultOption(e.getKey(), e.getValue()); first = false; }
      else       { autoChooser.addOption(e.getKey(), e.getValue()); }
    }
    SmartDashboard.putData("Auto Mode", autoChooser);

    // Mirror the routines into the web UI's Driver Station panel (auto selector dropdown).
    webSelectedAuto = autos.keySet().iterator().next(); // "None"
    if (webControl != null) {
      webControl.bindAutos(
          autos.keySet().toArray(new String[0]),
          webSelectedAuto,
          name -> { if (autos.containsKey(name)) webSelectedAuto = name; });
    }
  }

  @Override
  protected void configureBindings() {
    super.configureBindings();

    controller.y().onTrue(TeleopRoutines.driveToHub(drive));

    if (!RobotBase.isSimulation()) return;

    drive.getDriveTrainSimulation().ifPresent(driveSim -> {
      demoIntake = new DemoIntake(driveSim, drive::getPose, drive.getField2d());

      controller.leftBumper().onTrue(demoIntake.extendCommand());
      controller.rightBumper().onTrue(demoIntake.retractCommand());
      controller.a().onTrue(demoIntake.fireCommand());
    });
  }

  @Override
  public Command getAutonomousCommand() {
    Command visual = super.getAutonomousCommand();
    if (visual != null) return visual;
    // In web UI mode the Driver Station panel's selector is authoritative; otherwise use
    // the Glass SendableChooser.
    if (webControl != null && webSelectedAuto != null) return autos.get(webSelectedAuto);
    return autoChooser.getSelected();
  }
}
