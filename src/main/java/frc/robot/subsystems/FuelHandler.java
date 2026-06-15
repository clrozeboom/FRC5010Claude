package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Supplier;
import org.frc5010.common.sim.SimRobotState;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation.IntakeSide;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

/**
 * The robot's Fuel-handling subsystem — a from-scratch replacement for the library's demo
 * intake, built on the team's own three devices: an {@link IntakeArm}, an {@link IntakeRoller},
 * and a {@link Shooter}.
 *
 * <p>It extends the library {@link SimRobotState}, which (for free) owns the IronMaple
 * {@link IntakeSimulation} that models Fuel collection, cleans up collected pieces each cycle,
 * and publishes the held/extended/scored counts to the web UI. This class adds the behaviour
 * specific to <em>this</em> robot:
 *
 * <ul>
 *   <li>{@link #deployCommand()} — swing the arm out, spin the roller, start collecting.</li>
 *   <li>{@link #retractCommand()} — swing the arm in, stop the roller, stop collecting.</li>
 *   <li>{@link #scoreCommand()} — spin the shooter up, wait until it's at speed, then launch
 *       one Fuel in the direction the robot is facing.</li>
 * </ul>
 *
 * <p>The mechanisms are <b>this subsystem's own</b>; {@link #close()} frees their motors and the
 * owning robot registers it for clean test teardown.
 */
public class FuelHandler extends SimRobotState implements AutoCloseable {

  // --- field geometry (2026 Rebuilt) ---
  private static final double BUMPER_HALF_M = Units.inchesToMeters(15);
  private static final double FIELD_WIDTH_M = 16.540988;
  private static final double ZONE_DEPTH_M = 3.952; // alliance scoring zone depth
  /** The hubs you score into, as (x, y, z) in meters. */
  private static final Translation3d BLUE_HUB = new Translation3d(4.5974, 4.034536, 1.5748);
  private static final Translation3d RED_HUB = new Translation3d(11.938, 4.034536, 1.5748);

  // --- launch parameters ---
  private static final int PRELOADED_FUEL = 8;
  private static final double LAUNCH_HEIGHT_M = 0.5;
  private static final double SHOT_ELEVATION_DEG = 55.0; // upward tilt of the shot
  private static final double GRAVITY = 9.80665;
  private static final double MIN_SPEED = 3.0;
  private static final double MAX_SPEED = 20.0;

  // The three devices this subsystem drives.
  private final IntakeArm arm = new IntakeArm();
  private final IntakeRoller roller = new IntakeRoller();
  private final Shooter shooter = new Shooter();

  // Number of Fuel scored in the hub. Written from the projectile callback (robot thread),
  // read by the web UI (HTTP thread) — AtomicInteger keeps that safe.
  private final AtomicInteger scoredFuel = new AtomicInteger(0);
  private final FieldObject2d fuelField; // null when no Field2d (no NT publishing)

  /**
   * @param driveSim     the physics drive-train (from {@code drive.getDriveTrainSimulation().get()})
   * @param poseSupplier supplies the current robot pose (for the launch origin and heading)
   * @param field2d      Field2d for drawing Fuel in Glass/AdvantageScope; {@code null} to skip
   */
  public FuelHandler(
      AbstractDriveTrainSimulation driveSim, Supplier<Pose2d> poseSupplier, Field2d field2d) {
    super(
        IntakeSimulation.OverTheBumperIntake(
            "Fuel", driveSim, Inches.of(24), Inches.of(12), IntakeSide.FRONT, 50),
        poseSupplier,
        BUMPER_HALF_M + Units.inchesToMeters(6),
        field2d != null ? field2d.getObject("Intake") : null);
    this.fuelField = field2d != null ? field2d.getObject("Fuel") : null;
    intakeSimulation.setGamePiecesCount(PRELOADED_FUEL); // start the match with a preload

    // Idle states so nothing sags or free-spins at startup. A deploy/retract/score command
    // temporarily takes over the relevant device, then these resume when it finishes.
    arm.setDefaultCommand(arm.goToAngle(IntakeArm.RETRACT_ANGLE));
    roller.setDefaultCommand(roller.goToSpeed(RPM.of(0)));
    shooter.setDefaultCommand(shooter.goToSpeed(RPM.of(0)));
  }

  // ------------------------------------------------------------------ commands

  /** Swing the arm out, spin the roller up, and start collecting Fuel. */
  public Command deployCommand() {
    return Commands.parallel(
            arm.goToAngle(IntakeArm.DEPLOY_ANGLE),
            roller.goToSpeed(IntakeRoller.INTAKE_RPM))
        .beforeStarting(() -> {
          intakeSimulation.startIntake();
          intakeExtended = true;
        })
        .withName("Deploy");
  }

  /** Swing the arm in, stop the roller, and stop collecting Fuel. */
  public Command retractCommand() {
    return Commands.parallel(
            arm.goToAngle(IntakeArm.RETRACT_ANGLE),
            roller.goToSpeed(RPM.of(0)))
        .beforeStarting(() -> {
          intakeSimulation.stopIntake();
          intakeExtended = false;
        })
        .withName("Retract");
  }

  /** Spin the shooter up and hold it at speed (use while you line up a shot). */
  public Command spinUpShooterCommand() {
    return shooter.goToSpeed(Shooter.SHOOT_RPM).withName("SpinUpShooter");
  }

  /** Stop the shooter (let it coast back down). */
  public Command stopShooterCommand() {
    return shooter.goToSpeed(RPM.of(0)).withName("StopShooter");
  }

  /** Launch one held Fuel right now in the direction the robot is facing. No-op if empty. */
  public Command fireCommand() {
    return Commands.runOnce(() -> launchFuel(poseSupplier.get()), this).withName("Fire");
  }

  /**
   * The full scoring action: spin the shooter up, wait until it reaches speed, fire one Fuel,
   * then let the shooter spin back down (its default command resumes when this command ends).
   */
  public Command scoreCommand() {
    return spinUpShooterCommand()
        .raceWith(
            Commands.waitUntil(shooter.isAtSpeed(Shooter.SHOOT_RPM, Shooter.RPM_TOLERANCE))
                .andThen(fireCommand()))
        .withName("Score");
  }

  // ------------------------------------------------------------------ status (for the LEDs)

  /** Current arm angle — exposed for tests and telemetry. */
  public Angle getArmAngle() {
    return arm.getAngle();
  }

  /** Whether the intake is currently deployed. */
  public boolean isIntakeExtended() {
    return isExtended();
  }

  /** Whether the roller is spinning (collecting). */
  public boolean isRollerSpinning() {
    return Math.abs(roller.getSpeed().in(RPM)) > 50;
  }

  /** Whether the shooter is spinning at all. */
  public boolean isShooterSpinning() {
    return Math.abs(shooter.getSpeed().in(RPM)) > 50;
  }

  /** Whether the shooter has reached firing speed. */
  public boolean isShooterAtSpeed() {
    return Math.abs(shooter.getSpeed().in(RPM) - Shooter.SHOOT_RPM.in(RPM))
        <= Shooter.RPM_TOLERANCE.in(RPM);
  }

  /** Number of Fuel currently held. */
  public int getHeldFuel() {
    return getHeldPieces();
  }

  /** Number of Fuel scored — overrides the base so the web UI shows it. */
  @Override
  protected int getScoredCount() {
    return scoredFuel.get();
  }

  // ------------------------------------------------------------------ internals

  @Override
  public void periodic() {
    super.periodic(); // base cleans up collected pieces and updates the intake overlay
    if (fuelField == null) {
      return;
    }
    // Draw every Fuel piece (resting and in flight) so the field view shows them.
    List<Pose2d> poses = new ArrayList<>();
    try {
      for (var piece : SimulatedArena.getInstance().gamePiecesOnField()) {
        if ("Fuel".equals(piece.getType())) {
          poses.add(piece.getPoseOnField());
        }
      }
      for (var proj : SimulatedArena.getInstance().gamePieceLaunched()) {
        if ("Fuel".equals(proj.getType())) {
          var p = proj.getPose3d();
          poses.add(new Pose2d(p.getX(), p.getY(), p.getRotation().toRotation2d()));
        }
      }
    } catch (Exception ignored) {
      // Arena not ready yet; skip this frame.
    }
    fuelField.setPoses(poses);
  }

  /** Whether {@code pose} is inside the current alliance's scoring zone. */
  private static boolean isInAllianceZone(Pose2d pose) {
    boolean blue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    return blue ? pose.getX() < ZONE_DEPTH_M : pose.getX() > FIELD_WIDTH_M - ZONE_DEPTH_M;
  }

  /**
   * Launches one Fuel in the direction the robot is facing. When the robot is inside its
   * scoring zone and aimed at the hub, the shot scores (and bumps {@link #scoredFuel}); aimed
   * elsewhere, it simply lands on the field.
   */
  private void launchFuel(Pose2d pose) {
    if (intakeSimulation.getGamePiecesAmount() <= 0) {
      return; // nothing to shoot
    }
    intakeSimulation.obtainGamePieceFromIntake(); // remove one from the held count

    double heading = pose.getRotation().getRadians(); // <-- we launch along the robot heading
    Translation2d launchPos = new Translation2d(
        pose.getX() + BUMPER_HALF_M * Math.cos(heading),
        pose.getY() + BUMPER_HALF_M * Math.sin(heading));

    boolean blue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    Translation3d hub = blue ? BLUE_HUB : RED_HUB;

    // Pick a launch speed that would reach the hub's distance/height at our fixed elevation,
    // using the projectile-motion formula v² = g·d² / (2·cos²θ·(d·tanθ − Δh)).
    double dx = hub.getX() - launchPos.getX();
    double dy = hub.getY() - launchPos.getY();
    double horizontal = Math.hypot(dx, dy);
    double dz = hub.getZ() - LAUNCH_HEIGHT_M;
    double speed = launchSpeed(horizontal, dz);

    RebuiltFuelOnFly fuel = new RebuiltFuelOnFly(
        launchPos,
        new Translation2d(),
        new ChassisSpeeds(),
        new Rotation2d(heading),            // azimuth = robot heading
        Meters.of(LAUNCH_HEIGHT_M),
        MetersPerSecond.of(speed),
        Degrees.of(SHOT_ELEVATION_DEG));

    // If we're in the zone, treat the hub as the target so a well-aimed shot scores.
    if (isInAllianceZone(pose)) {
      fuel.withTargetPosition(() -> hub)
          .withTargetTolerance(new Translation3d(0.6, 0.6, 0.5))
          .withHitTargetCallBack(scoredFuel::incrementAndGet);
    }
    fuel.enableBecomesGamePieceOnFieldAfterTouchGround();
    fuel.launch();
    SimulatedArena.getInstance().addGamePieceProjectile(fuel);
  }

  private static double launchSpeed(double horizontal, double dz) {
    double theta = Math.toRadians(SHOT_ELEVATION_DEG);
    double cos = Math.cos(theta);
    double denom = 2.0 * cos * cos * (horizontal * Math.tan(theta) - dz);
    if (denom <= 0 || horizontal < 0.01) {
      return MIN_SPEED;
    }
    double v = Math.sqrt(GRAVITY * horizontal * horizontal / denom);
    return Math.max(MIN_SPEED, Math.min(MAX_SPEED, v));
  }

  /** Frees the three motors. The owning robot registers this for test teardown. */
  @Override
  public void close() {
    arm.close();
    roller.close();
    shooter.close();
  }
}
