package frc.robot.rebuilt.subsystems;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import java.util.concurrent.atomic.AtomicReference;
import org.frc5010.common.robot.Mode;
import org.frc5010.common.robot.RobotMode;
import org.littletonrobotics.junction.Logger;

/**
 * 2-state torque-current-FOC turret controller — ported from the source Rebuilt2026
 * {@code SmartTurretController}. Built for <b>fast tracking of a moving target</b>:
 *
 * <ul>
 *   <li><b>SEEKING</b> (large error): {@link MotionMagicExpoTorqueCurrentFOC} — smooth,
 *       physically-optimal long travel (exponential profile from the motor model).
 *   <li><b>TRACKING</b> (small error): {@link PositionTorqueCurrentFOC} with explicit velocity
 *       feedforward + position-error-directed kS — tracks a moving target with near-zero lag.
 *   <li>SEEK↔TRACK hysteresis prevents jitter.
 * </ul>
 *
 * <p>It owns its TalonFX directly (CANivore on the real robot) and runs {@link #step(double)} on a
 * <b>200&nbsp;Hz {@link Notifier}</b> — deliberately outside the synchronous-{@code periodic()}
 * convention of {@code org.frc5010.common.mechanisms} because turret tracking needs the high-rate
 * loop and torque-current control. {@link #setTarget} is called from the 20&nbsp;ms robot loop;
 * the handoff is thread-safe via an {@link AtomicReference} over an immutable {@link TurretTarget}.
 *
 * <p>In simulation the controller also integrates a {@link DCMotorSim} inside {@code step()} (so
 * the plant advances at the 200&nbsp;Hz control rate) and feeds the TalonFX sim state. Tests can
 * drive {@code step()} directly without the Notifier for deterministic results.
 */
public class SmartTurretController implements AutoCloseable {

  /** SEEKING (onboard expo profile) or TRACKING (position + velocity FF). */
  public enum TurretState {
    SEEKING,
    TRACKING
  }

  /** Immutable target from the 20 ms loop, consumed by the 200 Hz step loop. */
  public record TurretTarget(
      double positionMechRot, double velocityRadPerSec, double accelerationRadPerSecSq) {}

  private final SmartTurretConfig config;
  private final TalonFX talonFX;
  private final boolean sim;
  private final DCMotorSim plant; // sim only
  private final Notifier notifier;

  // Cached high-rate signals, refreshed each step for low-latency feedback.
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;

  private final MotionMagicExpoTorqueCurrentFOC seekingRequest =
      new MotionMagicExpoTorqueCurrentFOC(0).withSlot(0);
  private final PositionTorqueCurrentFOC trackingRequest =
      new PositionTorqueCurrentFOC(0).withSlot(1);
  private final NeutralOut neutralRequest = new NeutralOut();

  private final AtomicReference<TurretTarget> target =
      new AtomicReference<>(new TurretTarget(0, 0, 0));

  private volatile TurretState currentState = TurretState.SEEKING;
  private volatile boolean enabled = false;

  public SmartTurretController(SmartTurretConfig config) {
    this.config = config;
    this.talonFX = new TalonFX(config.canId, new CANBus(config.canBus));
    this.sim = RobotMode.get() == Mode.SIM;

    TalonFXConfiguration fx = new TalonFXConfiguration();
    // Work in mechanism rotations.
    fx.Feedback.SensorToMechanismRatio = config.gearRatio;

    // Slot0: SEEKING (MotionMagicExpoTorqueCurrentFOC).
    fx.Slot0.kP = config.seekingKP;
    fx.Slot0.kI = config.seekingKI;
    fx.Slot0.kD = config.seekingKD;
    fx.Slot0.kS = config.kS;
    fx.Slot0.kV = config.kV;
    fx.Slot0.kA = config.kA;

    // Slot1: TRACKING (PositionTorqueCurrentFOC). kS is injected per-cycle (position-error sign).
    fx.Slot1.kP = config.trackingKP;
    fx.Slot1.kI = config.trackingKI;
    fx.Slot1.kD = config.trackingKD;
    fx.Slot1.kS = 0;
    fx.Slot1.kV = config.kV;
    fx.Slot1.kA = config.kA;

    // Expo plant model (always Volts) + cruise cap.
    fx.MotionMagic.MotionMagicExpo_kV = config.resolvedExpoKV();
    fx.MotionMagic.MotionMagicExpo_kA = config.resolvedExpoKA();
    fx.MotionMagic.MotionMagicCruiseVelocity = config.maxVelocityMechRotPerSec;

    fx.TorqueCurrent.PeakForwardTorqueCurrent = config.peakTorqueCurrentAmps;
    fx.TorqueCurrent.PeakReverseTorqueCurrent = -config.peakTorqueCurrentAmps;
    fx.CurrentLimits.StatorCurrentLimitEnable = false;

    fx.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    fx.SoftwareLimitSwitch.ForwardSoftLimitThreshold = config.upperLimitRotations;
    fx.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    fx.SoftwareLimitSwitch.ReverseSoftLimitThreshold = config.lowerLimitRotations;

    talonFX.getConfigurator().apply(fx);
    talonFX.setPosition(config.startingPositionRot);

    positionSignal = talonFX.getPosition();
    velocitySignal = talonFX.getVelocity();
    BaseStatusSignal.setUpdateFrequencyForAll(250.0, positionSignal, velocitySignal);

    if (sim) {
      plant =
          new DCMotorSim(
              LinearSystemId.createDCMotorSystem(config.motorModel, config.moiKgM2, config.gearRatio),
              config.motorModel);
      plant.setState(config.startingPositionRot * 2 * Math.PI, 0);
      talonFX.getSimState().setRawRotorPosition(config.startingPositionRot * config.gearRatio);
    } else {
      plant = null;
    }

    notifier = new Notifier(() -> step(0.005));
    notifier.setName("SmartTurret");
  }

  /** Starts the 200 Hz control Notifier. (Tests call {@link #step} directly instead.) */
  public void start() {
    notifier.startPeriodic(0.005);
  }

  /**
   * Sets the turret target from the 20 ms loop. Enables the controller.
   *
   * @param position                desired mechanism angle (clamped to the soft limits)
   * @param velocityRadPerSec        velocity feedforward, mechanism rad/s
   * @param accelerationRadPerSecSq  acceleration feedforward, mechanism rad/s²
   */
  public void setTarget(Angle position, double velocityRadPerSec, double accelerationRadPerSecSq) {
    double posRot =
        MathUtil.clamp(
            position.in(Rotations), config.lowerLimitRotations, config.upperLimitRotations);
    target.set(new TurretTarget(posRot, velocityRadPerSec, accelerationRadPerSecSq));
    enabled = true;
    Logger.recordOutput("SmartTurret/GoalPositionRot", posRot);
  }

  /** Steps the controller (200 Hz on the Notifier, or directly in tests). */
  public void step(double dtSeconds) {
    if (sim) {
      updateSimPlant(dtSeconds);
    }
    BaseStatusSignal.refreshAll(positionSignal, velocitySignal);
    if (!enabled) {
      talonFX.setControl(neutralRequest);
      return;
    }

    TurretTarget t = target.get();
    double actualRot = positionSignal.getValueAsDouble();
    double signedError = t.positionMechRot() - actualRot;
    double error = Math.abs(signedError);

    // Hysteresis.
    if (currentState == TurretState.SEEKING) {
      if (error < config.seekingThresholdRotations) {
        currentState = TurretState.TRACKING;
      }
    } else if (error > config.seekingThresholdRotations + config.hysteresisBufferRotations) {
      currentState = TurretState.SEEKING;
    }

    switch (currentState) {
      case SEEKING -> talonFX.setControl(seekingRequest.withPosition(t.positionMechRot()));
      case TRACKING -> {
        double ksFf =
            error < config.trackingDeadbandRotations
                ? 0.0
                : Math.signum(signedError) * config.kS;
        talonFX.setControl(
            trackingRequest
                .withPosition(t.positionMechRot())
                .withVelocity(RadiansPerSecond.of(t.velocityRadPerSec()))
                .withFeedForward(ksFf));
      }
    }

    Logger.recordOutput("SmartTurret/State", currentState.name());
    Logger.recordOutput("SmartTurret/ActualPositionRot", actualRot);
    Logger.recordOutput("SmartTurret/PositionErrorRot", error);
  }

  private void updateSimPlant(double dt) {
    var simState = talonFX.getSimState();
    simState.setSupplyVoltage(RobotController.getBatteryVoltage());
    plant.setInputVoltage(simState.getMotorVoltage());
    plant.update(dt);
    simState.setRawRotorPosition(plant.getAngularPositionRotations() * config.gearRatio);
    simState.setRotorVelocity(
        plant.getAngularVelocityRadPerSec() / (2 * Math.PI) * config.gearRatio);
  }

  /** Current mechanism position (rotations), from the encoder. */
  public double getActualPositionRot() {
    return positionSignal.getValueAsDouble();
  }

  /** Current mechanism position as an angle. */
  public Angle getActualAngle() {
    return Rotations.of(getActualPositionRot());
  }

  /** Current mechanism velocity, rad/s. */
  public double getActualVelocityRadPerSec() {
    return velocitySignal.getValueAsDouble() * 2 * Math.PI;
  }

  /** Current target mechanism position (rotations). */
  public double getTargetRot() {
    return target.get().positionMechRot();
  }

  /** True while within {@code toleranceRot} of the target. */
  public boolean isAtTarget(double toleranceRot) {
    return Math.abs(getTargetRot() - getActualPositionRot()) <= toleranceRot;
  }

  public TurretState getState() {
    return currentState;
  }

  /** Disables output (neutral) and parks the target at the current position. */
  public void disable() {
    enabled = false;
    target.set(new TurretTarget(getActualPositionRot(), 0, 0));
    currentState = TurretState.SEEKING;
    talonFX.setControl(neutralRequest);
  }

  public SmartTurretConfig getConfig() {
    return config;
  }

  @Override
  public void close() {
    notifier.stop();
    notifier.close();
    talonFX.close();
  }
}
