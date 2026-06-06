package org.frc5010.common.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import java.util.function.Supplier;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

/**
 * Abstract base for sim-mode game-piece subsystems.
 *
 * <p>Owns the {@link IntakeSimulation} instance — registers it at construction,
 * cleans up obtained game pieces each cycle in {@link #periodic()}, and provides
 * generic {@link #extendCommand()} / {@link #retractCommand()} implementations.
 *
 * <p>When a {@link FieldObject2d} is supplied, renders the intake position on the
 * Glass Field2d widget each cycle: visible when extended, hidden when retracted.
 *
 * <p>Automatically binds state to the web UI (if active) via {@link WebControl#getInstance()}
 * so subclasses never need to inject or reference {@link WebControl} directly.
 *
 * <p>Subclasses supply game-specific behaviour by overriding {@link #getScoredCount()}
 * and adding firing / scoring logic. Access {@link #intakeSimulation},
 * {@link #intakeExtended}, and {@link #poseSupplier} directly from the subclass as needed.
 */
public abstract class SimRobotState extends SubsystemBase {

  protected final IntakeSimulation intakeSimulation;
  /** Set by {@link #extendCommand()} / {@link #retractCommand()}; read by the HTTP thread. */
  protected volatile boolean intakeExtended = false;
  /** Supplies the current robot pose; used for the Field2d overlay. Null when no overlay. */
  protected final Supplier<Pose2d> poseSupplier;
  private final double intakeForwardOffsetM;
  private final FieldObject2d intakeField;

  /** Constructor without Field2d intake overlay. */
  protected SimRobotState(IntakeSimulation intakeSimulation) {
    this(intakeSimulation, null, 0.0, null);
  }

  /**
   * Constructor with Field2d intake overlay.
   *
   * @param intakeSimulation     the IronMaple intake physics object
   * @param poseSupplier         supplies current robot pose for overlay positioning
   * @param intakeForwardOffsetM distance from robot centre to intake centre, metres
   * @param intakeField          FieldObject2d to render the intake graphic; null to skip
   */
  protected SimRobotState(
      IntakeSimulation intakeSimulation,
      Supplier<Pose2d> poseSupplier,
      double intakeForwardOffsetM,
      FieldObject2d intakeField) {
    this.intakeSimulation = intakeSimulation;
    this.poseSupplier = poseSupplier;
    this.intakeForwardOffsetM = intakeForwardOffsetM;
    this.intakeField = intakeField;
    intakeSimulation.register();
    WebControl.getInstance().ifPresent(wc ->
        wc.bindDemoState(this::getHeldPieces, this::isExtended, this::getScoredCount));
  }

  @Override
  public void periodic() {
    intakeSimulation.removeObtainedGamePieces(SimulatedArena.getInstance());
    if (intakeField != null) {
      if (intakeExtended) {
        Pose2d robot = poseSupplier.get();
        double theta = robot.getRotation().getRadians();
        intakeField.setPose(new Pose2d(
            robot.getX() + intakeForwardOffsetM * Math.cos(theta),
            robot.getY() + intakeForwardOffsetM * Math.sin(theta),
            robot.getRotation()));
      } else {
        intakeField.setPoses(List.of());
      }
    }
  }

  // ---- state — implementations are thread-safe for HTTP thread reads ----

  protected int getHeldPieces()  { return intakeSimulation.getGamePiecesAmount(); }
  protected boolean isExtended() { return intakeExtended; }
  /** Override to report game-specific scored count. Default returns 0. */
  protected int getScoredCount() { return 0; }

  // ---- generic intake commands ----

  public Command extendCommand() {
    return Commands.runOnce(() -> {
      intakeSimulation.startIntake();
      intakeExtended = true;
    }, this).withName("ExtendIntake");
  }

  public Command retractCommand() {
    return Commands.runOnce(() -> {
      intakeSimulation.stopIntake();
      intakeExtended = false;
    }, this).withName("RetractIntake");
  }
}
