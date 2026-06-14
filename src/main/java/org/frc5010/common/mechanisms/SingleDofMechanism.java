package org.frc5010.common.mechanisms;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc5010.common.tuning.TunableGains;
import org.littletonrobotics.junction.Logger;

/**
 * Shared engine for the single-DOF position mechanisms ({@link Elevator}, {@link Arm},
 * {@link Pivot}): owns the IO/inputs lifecycle, the goal state machine, the trapezoid
 * profile + LQR loop (or onboard MotionMagic dispatch), live tuning, the enable/disable
 * transitions, and the disconnect alert. Subclasses provide units (the
 * native-per-rotation conversion), the LQR plant, gravity feedforward, visualization,
 * and the WPILib-units command/getter API.
 *
 * <p><b>Units:</b> "native" is the controller's unit — meters for elevators, radians
 * for arms/pivots. The IO boundary is mechanism rotations
 * (native = rotations × {@code nativePerRot}). Tunables are published in "display"
 * units — meters for elevators, <em>degrees</em> for arms/pivots — and the base
 * converts consistently in both directions.
 */
public abstract class SingleDofMechanism extends SubsystemBase implements AutoCloseable {

  /** Builds a {@link MechanismLqr} from weights in native units. */
  protected interface LqrFactory {
    /**
     * @param qelmsPosNative position error tolerance, native units
     * @param qelmsVelNative velocity error tolerance, native units/s
     * @param relmsVolts     control effort tolerance, volts
     * @return a freshly built LQR loop
     */
    MechanismLqr build(double qelmsPosNative, double qelmsVelNative, double relmsVolts);
  }

  /** What the mechanism's output is currently doing. */
  protected enum OutputMode {
    /** No command yet — output neutral. */
    IDLE,
    /** Tracking {@code goalNative} with the configured control style. */
    GOAL,
    /** A command (duty cycle / SysId / homing) is driving the IO directly. */
    VOLTAGE
  }

  /** Base wiring collected by the subclass constructor. */
  protected static class BaseParams {
    /** Mechanism name for telemetry/tuning tables. */
    public String name;
    /** Hardware IO (already selected for the current RobotMode). */
    public MechanismIO io;
    /** Native units per mechanism rotation (drum circumference, or 2π for radians). */
    public double nativePerRot;
    /** Display (tunable) units per native unit — 1 for meters, 180/π for degrees. */
    public double displayPerNative;
    /** Trapezoid profile constraints in native units (LQR style only). */
    public TrapezoidProfile.Constraints profileConstraints;
    /** LQR plant factory; null = PROFILED_PID style (onboard MotionMagic). */
    public LqrFactory lqrFactory;
    /** Initial LQR weights, display units. */
    public double initialQelmsPosDisplay;
    /** Initial LQR velocity weight, display units/s. */
    public double initialQelmsVelDisplay;
    /** Initial control effort, volts. */
    public double initialRelmsVolts;
    /** Initial onboard PID gains (PROFILED_PID style). */
    public double kP;
    /** Initial integral gain. */
    public double kI;
    /** Initial derivative gain. */
    public double kD;
    /** Gravity feedforward, volts (LQR style; onboard handles it in PROFILED_PID). */
    public double kGVolts;
    /** True to scale kG by cos(position) — arms; false = constant — elevators. */
    public boolean cosineGravity;
    /** True to drop the goal when the robot is disabled (mechanism stays put on enable). */
    public boolean clearGoalOnDisable;
  }

  /** The hardware IO, available to subclasses (SysId, homing). */
  protected final MechanismIO io;
  /** The logged inputs — subclasses read state from here (replay-safe). */
  protected final MechanismIOInputsAutoLogged inputs = new MechanismIOInputsAutoLogged();

  private final BaseParams params;
  private MechanismLqr lqr; // null in PROFILED_PID style
  private final TrapezoidProfile profile; // null in PROFILED_PID style
  private TrapezoidProfile.State profileState = new TrapezoidProfile.State();
  private final LqrTunables lqrTunables; // null in PROFILED_PID style
  private final TunableGains pidGains; // null in LQR style
  private final Alert disconnectedAlert;

  /** Current output mode — subclasses may set VOLTAGE/IDLE from their commands. */
  protected OutputMode mode = OutputMode.IDLE;
  /** Current goal in native units (valid when {@code mode == GOAL}). */
  protected double goalNative;
  private boolean wasEnabled = false;

  /**
   * Wires the shared engine.
   *
   * @param params base wiring from the subclass constructor
   */
  protected SingleDofMechanism(BaseParams params) {
    this.params = params;
    this.io = params.io;
    setName(params.name);
    boolean useLqr = params.lqrFactory != null;
    lqr = useLqr ? params.lqrFactory.build(
        params.initialQelmsPosDisplay / params.displayPerNative,
        params.initialQelmsVelDisplay / params.displayPerNative,
        params.initialRelmsVolts) : null;
    profile = useLqr ? new TrapezoidProfile(params.profileConstraints) : null;
    lqrTunables = useLqr
        ? new LqrTunables(params.name,
            params.initialQelmsPosDisplay, params.initialQelmsVelDisplay, params.initialRelmsVolts)
        : null;
    pidGains = useLqr ? null
        : new TunableGains(params.name, "pid", params.kP, params.kI, params.kD, 0);
    disconnectedAlert = new Alert(params.name + " TalonFX disconnected", AlertType.kError);
  }

  /** Current position in native units (from the replay-safe inputs). */
  protected double positionNative() {
    return inputs.positionRot * params.nativePerRot;
  }

  /** Current velocity in native units/s (from the replay-safe inputs). */
  protected double velocityNative() {
    return inputs.velocityRotPerSec * params.nativePerRot;
  }

  /** Rebuilds a mechanism's 3D segments at a given mount pose (real mount, then mirror). */
  @FunctionalInterface
  protected interface SegmentBuilder {
    /**
     * @param mount the mount pose to draw against
     * @return the mechanism's segments for that mount
     */
    java.util.List<MechanismVisuals3d.Segment> build(Pose3d mount);
  }

  /**
   * Appends an offset mirror of the mechanism when a follower is configured
   * ({@code followerCanId >= 0}): the same geometry redrawn at {@code offset} from the
   * resolved mount (mount-local frame), e.g. the far side of an elevator or a duplicated
   * arm on the same shaft. The follower is mechanically locked to the lead, so the mirror
   * tracks the live state every cycle. No-op when no follower is configured.
   *
   * @param segments      the mutable segment list being built for this cycle
   * @param mount         the mechanism's resolved mount pose
   * @param followerCanId the follower CAN ID (&lt; 0 = none)
   * @param offset        mirror position relative to the mount, mount-local meters
   * @param builder       rebuilds the mechanism's segments at the mirrored mount
   */
  protected void appendFollowerMirror(
      java.util.List<MechanismVisuals3d.Segment> segments, Pose3d mount, int followerCanId,
      Translation3d offset, SegmentBuilder builder) {
    if (followerCanId < 0) {
      return;
    }
    segments.addAll(builder.build(MechanismVisuals3d.offsetMount(mount, offset)));
  }

  /** Logs the goal each cycle in subclass-friendly units. */
  protected abstract void logGoal(double goalNative);

  /** Updates the Mechanism2d (or other) visualization each cycle. */
  protected abstract void updateVisualization();

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(params.name, inputs);
    disconnectedAlert.set(!inputs.connected);

    if (lqrTunables != null && lqrTunables.hasChanged()) {
      lqr = params.lqrFactory.build(
          lqrTunables.qelmsPosition() / params.displayPerNative,
          lqrTunables.qelmsVelocity() / params.displayPerNative,
          lqrTunables.relms());
      resetControlState();
    }
    if (pidGains != null && pidGains.hasChanged()) {
      io.setPidGains(pidGains.kP(), pidGains.kI(), pidGains.kD());
    }

    boolean enabled = DriverStation.isEnabled();
    if (enabled && !wasEnabled) {
      resetControlState(); // re-seed profile + observer at the current state on enable
    }
    if (!enabled && wasEnabled && params.clearGoalOnDisable && mode == OutputMode.GOAL) {
      mode = OutputMode.IDLE;
    }
    wasEnabled = enabled;

    if (!enabled) {
      // Explicit neutral: real hardware would self-neutral, but the simulated Talon
      // keeps executing its last request as long as DS packets stay fresh.
      resetProfileToCurrent();
      io.stop();
    } else if (mode == OutputMode.VOLTAGE) {
      resetProfileToCurrent(); // an external command owns the output
    } else if (mode == OutputMode.GOAL) {
      if (lqr != null) {
        profileState = profile.calculate(0.02, profileState,
            new TrapezoidProfile.State(goalNative, 0));
        double gravity = params.cosineGravity
            ? params.kGVolts * Math.cos(positionNative())
            : params.kGVolts;
        io.setVoltage(lqr.calculatePosition(
            positionNative(), profileState.position, profileState.velocity) + gravity);
      } else {
        io.runPosition(goalNative / params.nativePerRot);
      }
    } else {
      io.stop();
    }

    logGoal(mode == OutputMode.GOAL ? goalNative : positionNative());
    updateVisualization();
  }

  private void resetControlState() {
    resetProfileToCurrent();
    if (lqr != null) {
      lqr.reset(positionNative(), velocityNative());
    }
  }

  private void resetProfileToCurrent() {
    profileState = new TrapezoidProfile.State(positionNative(), velocityNative());
  }

  /** Command factory: track a goal in native units. Never finishes. */
  protected Command goalCommand(double targetNative, String commandName) {
    return Commands.run(() -> {
      mode = OutputMode.GOAL;
      goalNative = targetNative;
    }, this).withName(commandName);
  }

  /** Command: open-loop duty cycle (battery-compensated onboard). Neutral when it ends. */
  public Command setDutyCycle(double dutyCycle) {
    return Commands.run(() -> {
      mode = OutputMode.VOLTAGE;
      io.setDutyCycle(dutyCycle);
    }, this).finallyDo(this::exitVoltageMode).withName(getName() + " DutyCycle");
  }

  /** Marks the output as externally driven (SysId/homing) so periodic stands down. */
  protected void enterVoltageMode() {
    mode = OutputMode.VOLTAGE;
  }

  /** Returns the output to neutral after an external command finishes. */
  protected void exitVoltageMode() {
    mode = OutputMode.IDLE;
    io.stop();
  }

  /** Stops control and frees the CAN devices. For unit tests. */
  @Override
  public void close() {
    MechanismVisuals3d.remove(params.name);
    io.close();
  }
}
