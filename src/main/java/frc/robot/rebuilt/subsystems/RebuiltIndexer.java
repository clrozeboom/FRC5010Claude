package frc.robot.rebuilt.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import frc.robot.rebuilt.Constants;
import org.frc5010.common.mechanisms.MechanismVisuals3d;
import org.littletonrobotics.junction.Logger;

/**
 * Indexer subsystem — ported from the 2026 "Rebuilt" robot.
 *
 * <p>A {@code spindexer} (percent/duty motor that agitates Fuel) plus a
 * {@code transfer} flywheel (feeds Fuel up to the launcher). Both are modelled as logged
 * duty state — the source treats the spindexer as a {@code percent_motor} and the transfer
 * as a velocity flywheel; here the commanded duty captures the behaviour for the sim and
 * tests without standing up extra Phoenix sim devices.
 *
 * <p>The {@link IndexerState} machine drives the speeds. The launcher↔indexer coupling
 * (PREP+atGoal → FEED, PREP+spinning-up → CHURN, else IDLE) is wired by the container; the
 * indexer only owns its state and the {@code flywheelReadyForChurn} gate.
 */
public class RebuiltIndexer extends SubsystemBase implements AutoCloseable {

  // ── 3D visual mounts ──────────────────────────────────────────────────────
  // Spindexer: horizontal rotating disc at the robot centre, ~10" above the floor.
  private static final Pose3d SPINDEXER_MOUNT = new Pose3d(
      0, 0, Units.inchesToMeters(10), MechanismVisuals3d.YAW_PLANE);
  private static final double SPINDEXER_RADIUS_M = Units.inchesToMeters(7); // ~14" disc

  // Transfer: vertical roller below the turret pivot (-4.856", 4.863", ~9").
  private static final Pose3d TRANSFER_MOUNT = new Pose3d(
      Units.inchesToMeters(-4.856), Units.inchesToMeters(4.863),
      Units.inchesToMeters(9), Rotation3d.kZero);
  private static final double TRANSFER_RADIUS_M = Units.inchesToMeters(2); // ~4" roller

  /** Indexer state, ported from {@code IndexerCommands.IndexerState}. */
  public enum IndexerState {
    IDLE,
    CHURN,
    HARD_CHURN,
    FORCE,
    FEED
  }

  private IndexerState requested = IndexerState.IDLE;
  private IndexerState current = IndexerState.IDLE;

  private double spindexerSpeed = 0.0;
  private double transferSpeed = 0.0;

  /**
   * Gate for CHURN: the source only physically churns once the flywheel is at/above goal
   * ({@code requireFlywheelAtGoalForChurn}, default true). Wired by the container from the
   * launcher; defaults to always-ready so the indexer works standalone.
   */
  private BooleanSupplier flywheelReadyForChurn = () -> true;

  public RebuiltIndexer() {
    setName(Constants.INDEXER);
    setDefaultCommand(Commands.run(this::applyState, this).withName("Indexer/StateMachine"));
  }

  private void applyState() {
    current = requested;
    switch (current) {
      case FEED:
      case FORCE:
        spindexerSpeed = Constants.Indexer.SPINDEXER_SPEED;
        transferSpeed = Constants.Indexer.TRANSFER_SPEED;
        break;
      case CHURN:
        // Gentle reverse agitation while the flywheel spins up — only physically churns
        // once the flywheel is ready (otherwise hold, to avoid feeding too early).
        if (flywheelReadyForChurn.getAsBoolean()) {
          spindexerSpeed = -0.1;
          transferSpeed = Constants.Indexer.TRANSFER_CHURN;
        } else {
          spindexerSpeed = 0;
          transferSpeed = 0;
        }
        break;
      case HARD_CHURN:
        spindexerSpeed = -0.5;
        transferSpeed = Constants.Indexer.TRANSFER_CHURN;
        break;
      case IDLE:
      default:
        spindexerSpeed = 0;
        transferSpeed = 0;
        break;
    }
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Indexer/StateRequested", requested.name());
    Logger.recordOutput("Indexer/StateCurrent", current.name());
    Logger.recordOutput("Indexer/Spindexer", spindexerSpeed);
    Logger.recordOutput("Indexer/Transfer", transferSpeed);
    updateVisualization();
  }

  private void updateVisualization() {
    // Spindexer — horizontal disc; needle sweeps CCW for forward (positive duty), CW for reverse.
    double spindexerFrac = Math.max(-1, Math.min(1, spindexerSpeed));
    double spindexerNeedle = -Math.PI / 2 + spindexerFrac * Math.PI;
    var spindexerCenter = MechanismVisuals3d.planarPoint(SPINDEXER_MOUNT, 0, 0);
    var spindexerSegs = new ArrayList<>(MechanismVisuals3d.planarCircle(
        SPINDEXER_MOUNT, 0, 0, SPINDEXER_RADIUS_M, 20, "spindexer", "#9b59b6", 1));
    spindexerSegs.add(new MechanismVisuals3d.Segment("spindexer-needle", spindexerCenter,
        MechanismVisuals3d.planarOffset(SPINDEXER_MOUNT, spindexerCenter, spindexerNeedle,
            SPINDEXER_RADIUS_M * 0.85),
        spindexerFrac != 0 ? "#d4a6f7" : "#555555", 3));
    MechanismVisuals3d.publish("Spindexer", spindexerSegs);

    // Transfer roller — vertical disc below the turret; needle sweeps by duty cycle.
    double transferFrac = Math.max(-1, Math.min(1, transferSpeed));
    double transferNeedle = -Math.PI / 2 + transferFrac * Math.PI;
    var transferCenter = MechanismVisuals3d.planarPoint(TRANSFER_MOUNT, 0, 0);
    var transferSegs = new ArrayList<>(MechanismVisuals3d.planarCircle(
        TRANSFER_MOUNT, 0, 0, TRANSFER_RADIUS_M, 16, "transfer", "#e67e22", 1));
    transferSegs.add(new MechanismVisuals3d.Segment("transfer-needle", transferCenter,
        MechanismVisuals3d.planarOffset(TRANSFER_MOUNT, transferCenter, transferNeedle,
            TRANSFER_RADIUS_M * 0.85),
        transferFrac != 0 ? "#ffa055" : "#555555", 3));
    MechanismVisuals3d.publish("Transfer", transferSegs);
  }

  @Override
  public void close() {
    MechanismVisuals3d.remove("Spindexer");
    MechanismVisuals3d.remove("Transfer");
  }

  // ── requests + accessors ───────────────────────────────────────────────────

  public void setRequestedState(IndexerState state) {
    requested = state;
  }

  public IndexerState getRequestedState() {
    return requested;
  }

  public IndexerState getCurrentState() {
    return current;
  }

  public boolean isCurrent(IndexerState state) {
    return current == state;
  }

  /** Whether the indexer is actively feeding Fuel to the launcher (FEED or FORCE). */
  public boolean isFeeding() {
    return current == IndexerState.FEED || current == IndexerState.FORCE;
  }

  public double getSpindexerSpeed() {
    return spindexerSpeed;
  }

  public double getTransferSpeed() {
    return transferSpeed;
  }

  public void setFlywheelReadyForChurn(BooleanSupplier ready) {
    this.flywheelReadyForChurn = ready;
  }

  // ── command factories ──────────────────────────────────────────────────────

  public Command feedCommand() {
    return Commands.runOnce(() -> setRequestedState(IndexerState.FEED)).withName("Indexer/Feed");
  }

  public Command forceCommand() {
    return Commands.runOnce(() -> setRequestedState(IndexerState.FORCE)).withName("Indexer/Force");
  }

  public Command churnCommand() {
    return Commands.runOnce(() -> setRequestedState(IndexerState.CHURN)).withName("Indexer/Churn");
  }

  public Command hardChurnCommand() {
    return Commands.runOnce(() -> setRequestedState(IndexerState.HARD_CHURN))
        .withName("Indexer/HardChurn");
  }

  public Command idleCommand() {
    return Commands.runOnce(() -> setRequestedState(IndexerState.IDLE)).withName("Indexer/Idle");
  }
}
