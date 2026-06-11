package org.frc5010.common.mechanisms;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.frc5010.common.robot.Mode;
import org.frc5010.common.robot.RobotMode;
import org.frc5010.common.tuning.TunableGains;
import org.littletonrobotics.junction.Logger;

/**
 * TalonFX elevator with selectable control style and full REAL / SIM / REPLAY support.
 *
 * <p><b>Architecture</b> (same IO pattern as the swerve drive): hardware access goes
 * through {@link MechanismIO} with {@code @AutoLog} inputs; REAL uses
 * {@link MechanismIOTalonFX}, SIM uses {@link MechanismIOTalonFXSim} fed by a WPILib
 * {@link ElevatorSim}, REPLAY uses the no-op IO and reads inputs from the log. All
 * getters and triggers read the inputs, so subsystem logic is replay-safe.
 *
 * <p><b>Control styles</b> ({@code settings.controlStyle}):
 * <ul>
 *   <li>{@link ControlStyle#LQR} (default) — state-space {@code LinearSystemLoop}
 *       (LQR + Kalman + plant inversion) running synchronously in {@code periodic()}
 *       at 20 ms, fed by a trapezoid profile, plus a kG feedforward (gravity is not in
 *       the linear plant and LQR has no integrator). Tuned with physical tolerances
 *       under {@code /Tuning/<name>/lqr_*}; supports SysId-characterized plants.</li>
 *   <li>{@link ControlStyle#PROFILED_PID} — TalonFX onboard MotionMagic at 1 kHz with
 *       Slot0 kP/kI/kD/kS/kV/kG (Elevator_Static). Gains in mechanism (drum) rotations,
 *       tunable under {@code /Tuning/<name>/pid_*}.</li>
 * </ul>
 */
public class Elevator extends SubsystemBase {

  /** Robot-specific elevator parameters. Populate the fields, then construct {@link Elevator}. */
  public static class Settings {
    /** Mechanism name used for telemetry and tuning tables. */
    public String name = "Elevator";
    /** Closed-loop control style: LQR (default) or profiled PID. */
    public ControlStyle controlStyle = ControlStyle.LQR;
    /** CAN ID of the TalonFX. */
    public int canId;
    /** Motor physics model (count = motors on the gearbox). */
    public DCMotor motorModel = DCMotor.getKrakenX60(1);
    /** Gear reduction stages, rotor → mechanism (e.g. {4, 3} = 12:1). */
    public double[] gearReductionStages = {4, 3};
    /** Drum/sprocket circumference — carriage travel per drum rotation. */
    public Distance drumCircumference = Inches.of(5.5);
    /** Mass of the moving carriage (plus load). */
    public Mass carriageMass = Kilograms.of(6.0);
    /** Lowest carriage position (soft limit + sim hard limit). */
    public Distance minHeight = Meters.of(0);
    /** Highest carriage position (soft limit + sim hard limit). */
    public Distance maxHeight = Meters.of(1.5);
    /** Carriage position at robot power-on. */
    public Distance startingHeight = Meters.of(0);
    /**
     * Motion profile cruise velocity. Must be achievable: below
     * motorFreeSpeed / gearing × drumCircumference, or the profile runs away from the
     * mechanism and the controller saturates/overshoots chasing it.
     */
    public LinearVelocity maxVelocity = MetersPerSecond.of(0.9);
    /** Motion profile acceleration. */
    public LinearAcceleration maxAcceleration = MetersPerSecondPerSecond.of(2.0);
    /** Gravity feedforward (volts to hold the carriage) — from SysId or sim ramp. */
    public Voltage kG = Volts.of(0);
    /** Stator current limit. */
    public Current statorCurrentLimit = Amps.of(40);

    // --- LQR weights (live-tunable; these are the initial values) ---
    /** Position error tolerance. Smaller = more aggressive. */
    public Distance qelmsPosition = Inches.of(2);
    /** Velocity error tolerance. Smaller = more aggressive. */
    public LinearVelocity qelmsVelocity = MetersPerSecond.of(0.5);
    /** Control effort tolerance. Smaller = gentler. 12 V = full battery. */
    public Voltage relms = Volts.of(12);

    // --- PROFILED_PID gains (live-tunable; TalonFX onboard, units = drum rotations) ---
    /** Proportional gain, volts per drum rotation of error. */
    public double kP = 4;
    /** Integral gain. */
    public double kI = 0;
    /** Derivative gain. */
    public double kD = 0;
    /** Static friction feedforward, volts (PROFILED_PID only). */
    public double kS = 0;
    /** Velocity feedforward, volts per drum rotation/s (PROFILED_PID only — the LQR provides its own). */
    public double kV = 0;

    // --- Kalman filter trust (rarely changed) ---
    /** Model position standard deviation — how much you trust the plant model. */
    public Distance modelPositionTrust = Meters.of(0.05);
    /** Model velocity standard deviation. */
    public LinearVelocity modelVelocityTrust = MetersPerSecond.of(0.5);
    /** Encoder position standard deviation — how much you trust the sensor. */
    public Distance encoderPositionTrust = Meters.of(0.001);

    // --- Characterized plant (optional, LQR style — see docs/mechanisms.md) ---
    /**
     * Measured kV from a SysId run, volts per m/s. Leave 0 to use the physics-model
     * plant. When both kV and kA are set, the LQR plant is built from these measured
     * values instead of motor + gearing + {@link #carriageMass} — the real mass and
     * losses are implied by how the mechanism actually responded to voltage, so an
     * unknown or wrong carriage mass no longer matters to the controller.
     */
    public double characterizedKv = 0;
    /** Measured kA from a SysId run, volts per m/s². See {@link #characterizedKv}. */
    public double characterizedKa = 0;
  }

  private enum OutputMode {
    /** No command yet — output neutral. */
    IDLE,
    /** Tracking {@code goalMeters} with the configured control style. */
    GOAL,
    /** A command (duty cycle / SysId) is driving {@link MechanismIO#setVoltage} directly. */
    VOLTAGE
  }

  private final Settings settings;
  private final double gearing;
  private final double metersPerRot;
  private final MechanismIO io;
  private final MechanismIOInputsAutoLogged inputs = new MechanismIOInputsAutoLogged();
  private MechanismLqr lqr; // null in PROFILED_PID style; rebuilt on retune
  private final TrapezoidProfile profile;
  private TrapezoidProfile.State profileState = new TrapezoidProfile.State();
  private final LqrTunables lqrTunables; // null in PROFILED_PID style
  private final TunableGains pidGains; // null in LQR style
  private final SysIdRoutine sysIdRoutine;

  private OutputMode mode = OutputMode.IDLE;
  private double goalMeters;
  private boolean wasEnabled = false;

  private final Mechanism2d mech2d;
  private final MechanismLigament2d carriageLigament;
  private final MechanismLigament2d goalLigament;

  /**
   * Builds the elevator subsystem, its IO (per {@link RobotMode}), controller, and sim.
   *
   * @param settings robot-specific elevator parameters
   */
  public Elevator(Settings settings) {
    this.settings = settings;
    setName(settings.name);
    gearing = product(settings.gearReductionStages);
    metersPerRot = settings.drumCircumference.in(Meters);

    var config = new MechanismIOTalonFX.Config();
    config.canId = settings.canId;
    config.gearing = gearing;
    config.statorCurrentLimitAmps = settings.statorCurrentLimit.in(Amps);
    config.softLimitLowRot = settings.minHeight.in(Meters) / metersPerRot;
    config.softLimitHighRot = settings.maxHeight.in(Meters) / metersPerRot;
    config.startingPositionRot = settings.startingHeight.in(Meters) / metersPerRot;
    config.motionMagicCruiseRotPerSec = settings.maxVelocity.in(MetersPerSecond) / metersPerRot;
    config.motionMagicAccelRotPerSecSq =
        settings.maxAcceleration.in(MetersPerSecondPerSecond) / metersPerRot;
    config.kP = settings.kP;
    config.kI = settings.kI;
    config.kD = settings.kD;
    config.kS = settings.kS;
    config.kV = settings.kV;
    config.kG = settings.kG.in(Volts);
    config.gravityType = GravityTypeValue.Elevator_Static;

    io = switch (RobotMode.get()) {
      case REPLAY -> new MechanismIO() {};
      case SIM -> new MechanismIOTalonFXSim(config, elevatorSim());
      case REAL -> new MechanismIOTalonFX(config);
    };

    boolean useLqr = settings.controlStyle == ControlStyle.LQR;
    lqr = useLqr ? buildLqr(
        settings.qelmsPosition.in(Meters),
        settings.qelmsVelocity.in(MetersPerSecond),
        settings.relms.in(Volts)) : null;
    profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
        settings.maxVelocity.in(MetersPerSecond),
        settings.maxAcceleration.in(MetersPerSecondPerSecond)));
    lqrTunables = useLqr
        ? new LqrTunables(settings.name,
            settings.qelmsPosition.in(Meters),
            settings.qelmsVelocity.in(MetersPerSecond),
            settings.relms.in(Volts))
        : null;
    pidGains = useLqr ? null
        : new TunableGains(settings.name, "pid", settings.kP, settings.kI, settings.kD, 0);

    sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(1).per(Second), Volts.of(7), Seconds.of(10)),
        new SysIdRoutine.Mechanism(
            volts -> io.setVoltage(volts.in(Volts)),
            log -> log.motor(settings.name)
                .voltage(Volts.of(inputs.appliedVolts))
                .linearPosition(Meters.of(getHeightMeters()))
                .linearVelocity(MetersPerSecond.of(inputs.velocityRotPerSec * metersPerRot)),
            this));

    double maxM = settings.maxHeight.in(Meters);
    mech2d = new Mechanism2d(maxM, maxM * 1.2);
    carriageLigament = mech2d.getRoot(settings.name + "Root", maxM / 2, 0)
        .append(new MechanismLigament2d("carriage", settings.startingHeight.in(Meters), 90));
    goalLigament = mech2d.getRoot(settings.name + "GoalRoot", maxM / 2 + 0.05, 0)
        .append(new MechanismLigament2d("goal", settings.startingHeight.in(Meters), 90, 3,
            new edu.wpi.first.wpilibj.util.Color8Bit(edu.wpi.first.wpilibj.util.Color.kWhite)));
    SmartDashboard.putData(settings.name + "/mechanism", mech2d);
  }

  private MechanismSim elevatorSim() {
    var sim = new ElevatorSim(
        settings.motorModel,
        gearing,
        settings.carriageMass.in(Kilograms),
        metersPerRot / (2 * Math.PI),
        settings.minHeight.in(Meters),
        settings.maxHeight.in(Meters),
        true,
        settings.startingHeight.in(Meters));
    return new MechanismSim() {
      @Override
      public void setInputVoltage(double volts) {
        sim.setInputVoltage(volts);
      }

      @Override
      public void update(double dtSeconds) {
        sim.update(dtSeconds);
      }

      @Override
      public double getPositionRot() {
        return sim.getPositionMeters() / metersPerRot;
      }

      @Override
      public double getVelocityRotPerSec() {
        return sim.getVelocityMetersPerSecond() / metersPerRot;
      }
    };
  }

  private MechanismLqr buildLqr(double qelmsPosM, double qelmsVelMps, double relmsVolts) {
    return MechanismLqr.elevator(
        settings.motorModel,
        gearing,
        settings.carriageMass.in(Kilograms),
        metersPerRot / (2 * Math.PI),
        qelmsPosM,
        qelmsVelMps,
        relmsVolts,
        settings.modelPositionTrust.in(Meters),
        settings.modelVelocityTrust.in(MetersPerSecond),
        settings.encoderPositionTrust.in(Meters),
        settings.characterizedKv,
        settings.characterizedKa);
  }

  private static double product(double[] stages) {
    double total = 1.0;
    for (double stage : stages) {
      total *= stage;
    }
    return total;
  }

  private double getHeightMeters() {
    return inputs.positionRot * metersPerRot;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(settings.name, inputs);

    if (lqrTunables != null && lqrTunables.hasChanged()) {
      lqr = buildLqr(lqrTunables.qelmsPosition(), lqrTunables.qelmsVelocity(), lqrTunables.relms());
      resetControlState();
    }
    if (pidGains != null && pidGains.hasChanged()) {
      io.setPidGains(pidGains.kP(), pidGains.kI(), pidGains.kD());
    }

    boolean enabled = DriverStation.isEnabled();
    if (enabled && !wasEnabled) {
      resetControlState(); // re-seed profile + observer at the current state on enable
    }
    wasEnabled = enabled;

    if (!enabled || mode == OutputMode.VOLTAGE) {
      // Disabled (Talon neutrals itself) or an external command owns the output.
      resetProfileToCurrent();
    } else if (mode == OutputMode.GOAL) {
      if (lqr != null) {
        profileState = profile.calculate(0.02, profileState,
            new TrapezoidProfile.State(goalMeters, 0));
        double volts = lqr.calculatePosition(
            getHeightMeters(), profileState.position, profileState.velocity)
            + settings.kG.in(Volts);
        io.setVoltage(volts);
      } else {
        io.runPosition(goalMeters / metersPerRot);
      }
    } else {
      io.stop();
    }

    Logger.recordOutput(settings.name + "/GoalMeters",
        mode == OutputMode.GOAL ? goalMeters : getHeightMeters());
    carriageLigament.setLength(Math.max(0.02, getHeightMeters()));
    goalLigament.setLength(Math.max(0.02, mode == OutputMode.GOAL ? goalMeters : getHeightMeters()));
  }

  private void resetControlState() {
    resetProfileToCurrent();
    if (lqr != null) {
      lqr.reset(getHeightMeters(), inputs.velocityRotPerSec * metersPerRot);
    }
  }

  private void resetProfileToCurrent() {
    profileState = new TrapezoidProfile.State(
        getHeightMeters(), inputs.velocityRotPerSec * metersPerRot);
  }

  /** Command: drive the carriage to the given height. Never finishes. */
  public Command goToHeight(Distance height) {
    Logger.recordOutput(settings.name + "/CommandedHeightMeters", height.in(Meters));
    return Commands.run(() -> {
      mode = OutputMode.GOAL;
      goalMeters = height.in(Meters);
    }, this).withName(settings.name + " GoToHeight");
  }

  /** Command: open-loop duty cycle (e.g. for manual jog). Neutral when it ends. */
  public Command setDutyCycle(double dutyCycle) {
    return Commands.run(() -> {
      mode = OutputMode.VOLTAGE;
      io.setVoltage(dutyCycle * 12.0);
    }, this).finallyDo(() -> {
      mode = OutputMode.IDLE;
      io.stop();
    }).withName(settings.name + " DutyCycle");
  }

  /** Command: SysId routine for characterizing kG/kS/kV/kA, guarded by the travel limits. */
  public Command sysId() {
    Trigger nearTop = isAtHeight(settings.maxHeight, Inches.of(3));
    Trigger nearBottom = isAtHeight(settings.minHeight, Inches.of(3));
    return Commands.sequence(
            Commands.runOnce(() -> mode = OutputMode.VOLTAGE),
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(nearTop),
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(nearBottom),
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).until(nearTop),
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).until(nearBottom))
        .finallyDo(() -> {
          mode = OutputMode.IDLE;
          io.stop();
        })
        .withName(settings.name + " SysId");
  }

  /** Current carriage height (from the AdvantageKit inputs — replay-safe). */
  public Distance getHeight() {
    return Meters.of(getHeightMeters());
  }

  /** Current carriage velocity (from the AdvantageKit inputs — replay-safe). */
  public LinearVelocity getVelocity() {
    return MetersPerSecond.of(inputs.velocityRotPerSec * metersPerRot);
  }

  /** Trigger: true while the carriage is within {@code tolerance} of {@code height}. */
  public Trigger isAtHeight(Distance height, Distance tolerance) {
    return new Trigger(
        () -> Math.abs(getHeightMeters() - height.in(Meters)) <= tolerance.in(Meters));
  }

  /** The settings this mechanism was built with (start positions, limits, ...). */
  public Settings getSettings() {
    return settings;
  }

  /** Stops control and frees the CAN device. For unit tests. */
  public void close() {
    io.close();
  }
}
