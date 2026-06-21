package frc.robot.rebuilt.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import frc.robot.rebuilt.Constants;
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
public class RebuiltIndexer extends SubsystemBase {

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
