package org.frc5010.common.drive.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.frc5010.common.drive.swerve.akit.AkitSwerveDrive;

/**
 * Reusable Layer 4 visual-test command sequence for {@link AkitSwerveDrive}.
 *
 * <p>Runs five scripted steps that verify IronMaple physics is working correctly
 * in simulation. Each step prints {@code [VISUAL TEST] PASS/FAIL} to stdout and
 * a final {@code OVERALL: PASS/FAIL} summary at the end.
 *
 * <p>Usage in a robot project:
 * <pre>{@code
 * // In RobotContainer:
 * public Command getAutonomousCommand() {
 *     if (Boolean.getBoolean("visualTest")) {
 *         return SwerveVisualTest.build(drive, this::getAllianceStartPose);
 *     }
 *     return null;
 * }
 * }</pre>
 *
 * <p>Enable via {@code ./gradlew simulateJava -PvisualTest}. The Driver Station is
 * auto-enabled in autonomous by {@code Robot.simulationInit()} when that flag is set.
 *
 * <h3>Test steps</h3>
 * <ol>
 *   <li>Forward (+X) at 1.5 m/s for 2 s — robot must advance {@literal >} 0.5 m</li>
 *   <li>Strafe left (+Y) at 1.0 m/s for 2 s — robot must advance {@literal >} 0.2 m</li>
 *   <li>Rotate CCW at π rad/s for 2 s — heading must change {@literal >} 0.5 rad</li>
 *   <li>Alliance-direction drive for 2 s — must displace {@literal >} 0.3 m toward the
 *       opponent's wall (correct direction for the detected alliance)</li>
 *   <li>Drive into the blue wall at 1.5 m/s for 5 s — IronMaple physics body must be
 *       between 0 m and 1.5 m (wall contacted, not penetrated)</li>
 * </ol>
 */
public final class SwerveVisualTest {

  private SwerveVisualTest() {}

  /**
   * Builds the five-step visual test command sequence.
   *
   * @param drive             the swerve drive subsystem under test
   * @param allianceStartPose supplies the alliance-correct starting pose; called at
   *                          schedule time so the alliance is already resolved
   * @return a {@link Command} that runs the full sequence and prints results
   */
  public static Command build(AkitSwerveDrive drive, Supplier<Pose2d> allianceStartPose) {
    boolean[] allPassed = {true};
    double[]  xSnapshot = {0.0};  // step 4: x at start of alliance-direction drive

    return Commands.sequence(

        Commands.runOnce(() -> {
          System.out.println("[VISUAL TEST] ========================================");
          System.out.println("[VISUAL TEST] Layer 4 visual-test sequence starting...");
          System.out.println("[VISUAL TEST] ========================================");
          System.out.flush();
        }),

        // --- Teleport to alliance-correct starting pose --------------------------
        Commands.runOnce(() -> drive.resetSimulationPose(allianceStartPose.get()), drive),
        // One extra cycle so the odometry estimator propagates the reset.
        Commands.waitSeconds(0.05),

        // --- Step 1: Forward at 1.5 m/s for 2 s (Blue-frame +X) -----------------
        Commands.runOnce(() -> {
          System.out.println("[VISUAL TEST] Step 1: driving forward (+X) at 1.5 m/s for 2 s...");
          System.out.flush();
        }),
        Commands.run(
            () -> drive.runVelocityFieldRelative(new ChassisSpeeds(1.5, 0.0, 0.0)), drive
        ).withTimeout(2.0),
        Commands.runOnce(() -> {
          double x = drive.getPose().getX();
          boolean ok = x > 0.5;
          if (!ok) allPassed[0] = false;
          System.out.printf("[VISUAL TEST] Forward:    %-4s  x = %.3f m  (threshold > 0.50 m)%n",
              ok ? "PASS" : "FAIL", x);
          System.out.flush();
        }),

        Commands.runOnce(() -> drive.stop(), drive),
        Commands.waitSeconds(0.4),

        // --- Step 2: Strafe left at 1.0 m/s for 2 s (+Y) ------------------------
        Commands.runOnce(() -> {
          System.out.println("[VISUAL TEST] Step 2: strafing left (+Y) at 1.0 m/s for 2 s...");
          System.out.flush();
        }),
        Commands.run(
            () -> drive.runVelocityFieldRelative(new ChassisSpeeds(0.0, 1.0, 0.0)), drive
        ).withTimeout(2.0),
        Commands.runOnce(() -> {
          double y = drive.getPose().getY();
          boolean ok = y > 0.2;
          if (!ok) allPassed[0] = false;
          System.out.printf("[VISUAL TEST] Strafe:     %-4s  y = %.3f m  (threshold > 0.20 m)%n",
              ok ? "PASS" : "FAIL", y);
          System.out.flush();
        }),

        Commands.runOnce(() -> drive.stop(), drive),
        Commands.waitSeconds(0.4),

        // --- Step 3: Rotate CCW at pi rad/s for 2 s ------------------------------
        Commands.runOnce(() -> {
          System.out.println("[VISUAL TEST] Step 3: rotating CCW at pi rad/s for 2 s...");
          System.out.flush();
        }),
        Commands.run(
            () -> drive.runVelocityFieldRelative(new ChassisSpeeds(0.0, 0.0, Math.PI)), drive
        ).withTimeout(2.0),
        Commands.runOnce(() -> {
          double heading = drive.getRotation().getRadians();
          boolean ok = Math.abs(heading) > 0.5;
          if (!ok) allPassed[0] = false;
          System.out.printf("[VISUAL TEST] Rotate:     %-4s  heading = %.3f rad  (threshold |h| > 0.50 rad)%n",
              ok ? "PASS" : "FAIL", heading);
          System.out.flush();
        }),

        Commands.runOnce(() -> drive.stop(), drive),
        Commands.waitSeconds(0.4),

        // --- Step 4: Alliance-direction check ------------------------------------
        // Drive "alliance-forward" (toward the opponent's wall) for 2 s.
        // Blue: +X direction; Red: −X direction (Glass defaults to Red).
        // Displacement from the start of this step must be in the correct direction.
        Commands.runOnce(() -> {
          Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
          xSnapshot[0] = drive.getPose().getX();
          System.out.printf("[VISUAL TEST] Step 4: alliance-direction check (alliance=%s, x_start=%.3f)...%n",
              alliance.name(), xSnapshot[0]);
          System.out.flush();
        }),
        Commands.run(
            () -> {
              double vx = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                  ? -1.5 : 1.5;
              drive.runVelocityFieldRelative(new ChassisSpeeds(vx, 0.0, 0.0));
            }, drive
        ).withTimeout(2.0),
        Commands.runOnce(() -> {
          double xAfter = drive.getPose().getX();
          double displacement = xAfter - xSnapshot[0];
          boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
          boolean ok = isRed ? displacement < -0.3 : displacement > 0.3;
          if (!ok) allPassed[0] = false;
          System.out.printf("[VISUAL TEST] Alliance:   %-4s  disp = %+.3f m  (expected %s 0.30 m for %s)%n",
              ok ? "PASS" : "FAIL", displacement,
              isRed ? "< -" : ">", isRed ? "RED" : "BLUE");
          System.out.flush();
        }),

        Commands.runOnce(() -> drive.stop(), drive),
        Commands.waitSeconds(0.4),

        // --- Step 5: Field-boundary check ----------------------------------------
        // Drive into the blue wall (−X) at 1.5 m/s for 5 s.
        // Two ground-truth checks use IronMaple physics (not odometry, which drifts
        // from wheel spin while the robot is pressed against the wall):
        //   • physics x ≥ 0         — robot did NOT penetrate the wall
        //   • physics x ≤ 1.5       — robot DID reach the wall (not still in transit)
        Commands.runOnce(() -> {
          System.out.println("[VISUAL TEST] Step 5: field-boundary check (−X at 1.5 m/s for 5 s)...");
          System.out.flush();
        }),
        Commands.runOnce(() -> drive.stop(), drive),
        Commands.waitSeconds(0.3),
        Commands.run(
            () -> drive.runVelocityFieldRelative(new ChassisSpeeds(-1.5, 0.0, 0.0)), drive
        ).withTimeout(5.0),
        Commands.runOnce(() -> {
          double physX    = drive.getSimulatedPose().map(p -> p.getX()).orElse(-999.0);
          double actualVx = drive.getSimulatedChassisSpeeds()
              .map(s -> s.vxMetersPerSecond).orElse(Double.NaN);
          boolean reachedWall    = physX <= 1.5;
          boolean didntPenetrate = physX >= 0.0;
          boolean ok = reachedWall && didntPenetrate;
          if (!ok) allPassed[0] = false;
          System.out.printf(
              "[VISUAL TEST] Boundary:   %-4s  physics_x=%.3f m  actual_vx=%.3f m/s%n"
              + "[VISUAL TEST]             (expected: 0.0 <= x <= 1.5 m)%n",
              ok ? "PASS" : "FAIL", physX, actualVx);
          System.out.flush();
        }),

        // --- Final stop + summary -----------------------------------------------
        Commands.runOnce(() -> drive.stop(), drive),
        Commands.runOnce(() -> {
          System.out.println("[VISUAL TEST] ========================================");
          System.out.printf( "[VISUAL TEST] OVERALL: %s%n", allPassed[0] ? "PASS" : "FAIL");
          System.out.println("[VISUAL TEST] ========================================");
          System.out.flush();
        })
    );
  }
}
