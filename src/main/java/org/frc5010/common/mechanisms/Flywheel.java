package org.frc5010.common.mechanisms;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.frc5010.common.robot.RobotMode;
import org.frc5010.common.tuning.TunableGains;
import org.littletonrobotics.junction.Logger;

/**
 * TalonFX flywheel (shooter wheel, roller) with selectable control style and
 * REAL / SIM / REPLAY support. Same architecture as {@link Elevator} (see its javadoc).
 *
 * <p>LQR style: 1-state velocity {@code LinearSystemLoop} on the RIO — the loop's
 * plant-inversion feedforward replaces kV, so none is configured. PROFILED_PID style:
 * onboard VelocityVoltage where kV does most of the work (≈ 12 V ÷ free speed in
 * mechanism rot/s) and kP trims the residual.
 */
public class Flywheel extends SubsystemBase implements AutoCloseable {

  /** Robot-specific flywheel parameters. */
  public static class Settings {
    /** Mechanism name used for telemetry and tuning tables. */
    public String name = "Flywheel";
    /** Closed-loop control style: LQR (default) or profiled PID. */
    public ControlStyle controlStyle = ControlStyle.LQR;
    /** CAN ID of the TalonFX. */
    public int canId;
    /** CAN ID of a follower TalonFX on the same shaft; −1 = single motor. */
    public int followerCanId = -1;
    /** True if the follower is mounted opposing the lead motor. */
    public boolean followerOpposed = false;
    /** Drop the goal when the robot is disabled (stay spun down on re-enable). */
    public boolean clearGoalOnDisable = false;
    /** Motor physics model. */
    public DCMotor motorModel = DCMotor.getKrakenX60(1);
    /** Gear reduction stages, rotor → wheel (1.0 = direct drive). */
    public double[] gearReductionStages = {1.0};
    /** Flywheel diameter (used for the MOI estimate). */
    public Distance diameter = Inches.of(4);
    /** Flywheel mass. */
    public Mass mass = Kilograms.of(1.5);

    /**
     * Where this mechanism sits on the robot for the 3D isometric view — robot frame,
     * x forward, y left, z up, meters from robot center at floor level. The rotation
     * re-aims the wheel plane: identity (default) spins it in the robot's X-Z
     * side-view plane (a normal shooter wheel).
     */
    public edu.wpi.first.math.geometry.Pose3d visualPose3d =
        new edu.wpi.first.math.geometry.Pose3d(
            -0.2, 0, 0.6, edu.wpi.first.math.geometry.Rotation3d.kZero);
    /**
     * Optional parent for 3D coupling: when set, {@link #visualPose3d} becomes an offset
     * from this supplier's live attachment pose instead of an absolute robot-frame mount,
     * so the flywheel rides another mechanism's moving endpoint (e.g. an arm tip).
     */
    public java.util.function.Supplier<edu.wpi.first.math.geometry.Pose3d> visualParent = null;
    /**
     * Structural offset from the parent's endpoint to where this mechanism attaches,
     * expressed in the parent's attachment frame (the bracket/standoff carrying it off
     * the parent). Applied before {@link #visualPose3d} when {@link #visualParent} is
     * set; identity (default) mounts straight on the parent's endpoint.
     */
    public edu.wpi.first.math.geometry.Transform3d visualParentOffset =
        new edu.wpi.first.math.geometry.Transform3d();
    // --- LQR weights (live-tunable in RPM; these are the initial values) ---
    /** Velocity error tolerance. Smaller = more aggressive. */
    public AngularVelocity qelmsVelocity = RadiansPerSecond.of(8);
    /** Control effort tolerance. Smaller = gentler. 12 V = full battery. */
    public Voltage relms = Volts.of(12);

    // --- PROFILED_PID gains (live-tunable; TalonFX onboard VelocityVoltage, mechanism rot/s) ---
    /** Proportional gain, volts per wheel rotation/s of error. */
    public double kP = 0.1;
    /** Integral gain. */
    public double kI = 0;
    /** Derivative gain. */
    public double kD = 0;
    /** Static friction feedforward, volts (PROFILED_PID only). */
    public double kS = 0;
    /**
     * Velocity feedforward, volts per wheel rotation/s — essential for velocity PID
     * (≈ 12 / free speed in rot/s). PROFILED_PID only; the LQR provides its own.
     */
    public double kV = 0;

    // --- Kalman filter trust (rarely changed) ---
    /** Model velocity standard deviation. */
    public AngularVelocity modelVelocityTrust = RadiansPerSecond.of(3.0);
    /** Encoder velocity standard deviation. */
    public AngularVelocity encoderVelocityTrust = RadiansPerSecond.of(0.01);

    // --- Characterized plant (optional, LQR style — see docs/mechanisms.md) ---
    /**
     * Measured kV from a SysId run, volts per rotation/s (SysId tool set to Rotations).
     * Leave 0 to use the physics-model plant. When both kV and kA are set, the LQR
     * plant is built from these measured values instead of motor + gearing +
     * {@link #mass}/{@link #diameter} — the real inertia and losses are implied by how
     * the wheel actually responded to voltage, so an unknown MOI no longer matters to
     * the controller.
     */
    public double characterizedKv = 0;
    /** Measured kA from a SysId run, volts per rotation/s². See {@link #characterizedKv}. */
    public double characterizedKa = 0;

    /**
     * Use FOC commutation on all control requests (~15% more torque). Requires
     * Phoenix Pro; unlicensed devices fall back to non-FOC with a fault. Set false
     * for non-Pro teams.
     */
    public boolean enableFoc = true;
    /** Stator current limit. */
    public Current statorCurrentLimit = Amps.of(60);
  }

  private enum OutputMode { IDLE, GOAL, VOLTAGE }

  private final Settings settings;
  private final double gearing;
  private final MechanismIO io;
  private final MechanismIOInputsAutoLogged inputs = new MechanismIOInputsAutoLogged();
  private MechanismLqr lqr; // null in PROFILED_PID style
  private final LqrTunables lqrTunables; // null in PROFILED_PID style
  private final TunableGains pidGains; // null in LQR style
  private final SysIdRoutine sysIdRoutine;

  private OutputMode mode = OutputMode.IDLE;
  private double goalRadPerSec;
  private boolean wasEnabled = false;
  private final edu.wpi.first.wpilibj.Alert disconnectedAlert;

  /**
   * Builds the flywheel subsystem, its IO (per {@link RobotMode}), controller, and sim.
   *
   * @param settings robot-specific flywheel parameters
   */
  public Flywheel(Settings settings) {
    this.settings = settings;
    setName(settings.name);
    gearing = product(settings.gearReductionStages);

    var config = new MechanismIOTalonFX.Config();
    config.canId = settings.canId;
    config.enableFoc = settings.enableFoc;
    config.followerCanId = settings.followerCanId;
    config.followerOpposed = settings.followerOpposed;
    config.gearing = gearing;
    config.brakeMode = false; // coast — don't fight a spinning wheel
    config.statorCurrentLimitAmps = settings.statorCurrentLimit.in(Amps);
    config.kP = settings.kP;
    config.kI = settings.kI;
    config.kD = settings.kD;
    config.kS = settings.kS;
    config.kV = settings.kV;
    config.kG = 0;
    config.gravityType = GravityTypeValue.Elevator_Static;

    io = switch (RobotMode.get()) {
      case REPLAY -> new MechanismIO() {};
      case SIM -> new MechanismIOTalonFXSim(config, flywheelSim());
      case REAL -> new MechanismIOTalonFX(config);
    };

    boolean useLqr = settings.controlStyle == ControlStyle.LQR;
    lqr = useLqr ? buildLqr(
        settings.qelmsVelocity.in(RadiansPerSecond), settings.relms.in(Volts)) : null;
    lqrTunables = useLqr
        ? new LqrTunables(settings.name,
            0, // position weight unused for flywheel LQR
            settings.qelmsVelocity.in(RPM),
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
                .angularPosition(edu.wpi.first.units.Units.Rotations.of(inputs.positionRot))
                .angularVelocity(RadiansPerSecond.of(getVelocityRadPerSec())),
            this));

    disconnectedAlert = new edu.wpi.first.wpilibj.Alert(
        settings.name + " TalonFX disconnected", edu.wpi.first.wpilibj.Alert.AlertType.kError);
  }

  private double moiKgM2() {
    // Same uniform-rod estimate WPILib's SingleJointedArmSim.estimateMOI uses,
    // applied to the wheel diameter — matches the sim plant exactly.
    return settings.mass.in(Kilograms) * Math.pow(settings.diameter.in(Meters), 2) / 3.0;
  }

  private MechanismSim flywheelSim() {
    var sim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(settings.motorModel, moiKgM2(), gearing),
        settings.motorModel);
    return new MechanismSim() {
      // FlywheelSim has no position state; integrate it so the Talon's sim encoder
      // stays monotonic (status signals and SysId logging expect a position).
      private double positionRad = 0;

      @Override
      public void setInputVoltage(double volts) {
        sim.setInputVoltage(volts);
      }

      @Override
      public void update(double dtSeconds) {
        sim.update(dtSeconds);
        positionRad += sim.getAngularVelocityRadPerSec() * dtSeconds;
      }

      @Override
      public double getPositionRot() {
        return positionRad / (2 * Math.PI);
      }

      @Override
      public double getVelocityRotPerSec() {
        return sim.getAngularVelocityRadPerSec() / (2 * Math.PI);
      }
    };
  }

  private MechanismLqr buildLqr(double qelmsVelRadPerSec, double relmsVolts) {
    return MechanismLqr.flywheel(
        settings.motorModel,
        gearing,
        moiKgM2(),
        qelmsVelRadPerSec,
        relmsVolts,
        settings.modelVelocityTrust.in(RadiansPerSecond),
        settings.encoderVelocityTrust.in(RadiansPerSecond),
        // Settings take SysId's rotation units; the flywheel plant works in radians.
        settings.characterizedKv / (2 * Math.PI),
        settings.characterizedKa / (2 * Math.PI));
  }

  private static double product(double[] stages) {
    double total = 1.0;
    for (double stage : stages) {
      total *= stage;
    }
    return total;
  }

  private double getVelocityRadPerSec() {
    return inputs.velocityRotPerSec * 2 * Math.PI;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(settings.name, inputs);
    disconnectedAlert.set(!inputs.connected);

    if (lqrTunables != null && lqrTunables.hasChanged()) {
      // Tunables are published in RPM (the natural shooter unit); the loop runs in rad/s.
      lqr = buildLqr(lqrTunables.qelmsVelocity() * 2 * Math.PI / 60.0, lqrTunables.relms());
      lqr.reset(0, getVelocityRadPerSec());
    }
    if (pidGains != null && pidGains.hasChanged()) {
      io.setPidGains(pidGains.kP(), pidGains.kI(), pidGains.kD());
    }

    boolean enabled = DriverStation.isEnabled();
    if (enabled && !wasEnabled && lqr != null) {
      lqr.reset(0, getVelocityRadPerSec());
    }
    if (!enabled && wasEnabled && settings.clearGoalOnDisable && mode == OutputMode.GOAL) {
      mode = OutputMode.IDLE;
    }
    wasEnabled = enabled;

    if (!enabled) {
      io.stop(); // the simulated Talon doesn't self-neutral while DS packets stay fresh
    } else if (mode == OutputMode.VOLTAGE) {
      // An external command owns the output.
    } else if (mode == OutputMode.GOAL) {
      if (lqr != null) {
        io.setVoltage(lqr.calculateVelocity(getVelocityRadPerSec(), goalRadPerSec));
      } else {
        io.runVelocity(goalRadPerSec / (2 * Math.PI));
      }
    } else {
      io.stop();
    }

    Logger.recordOutput(settings.name + "/GoalRPM",
        RadiansPerSecond.of(mode == OutputMode.GOAL ? goalRadPerSec : getVelocityRadPerSec()).in(RPM));

    var mount = MechanismVisuals3d.resolveMount(
        settings.visualPose3d, settings.visualParent, settings.visualParentOffset);
    double radius = settings.diameter.in(Meters) / 2;
    // Speedometer dial: 0 speed points straight down, full speed points up. Positive
    // speed sweeps the needle CCW (up the right side), negative sweeps CW (up the left),
    // so sign and magnitude both read at a glance. Normalized to wheel free speed.
    double maxSpeed = settings.motorModel.freeSpeedRadPerSec / gearing;
    double frac = maxSpeed > 1e-6
        ? Math.max(-1, Math.min(1, getVelocityRadPerSec() / maxSpeed)) : 0;
    double needleRad = -Math.PI / 2 + frac * Math.PI;
    var center = MechanismVisuals3d.planarPoint(mount, 0, 0);
    var segments = new java.util.ArrayList<>(MechanismVisuals3d.planarCircle(
        mount, 0, 0, radius, 20, "dial", "#2e6e40", 1));
    segments.add(new MechanismVisuals3d.Segment("zero", // 0-speed mark at the bottom
        MechanismVisuals3d.planarOffset(mount, center, -Math.PI / 2, radius * 0.8),
        MechanismVisuals3d.planarOffset(mount, center, -Math.PI / 2, radius),
        "#8b949e", 2));
    segments.add(new MechanismVisuals3d.Segment("needle", center,
        MechanismVisuals3d.planarOffset(mount, center, needleRad, radius * 0.92),
        "#7ee787", 3));
    MechanismVisuals3d.publish(settings.name, segments);
  }

  /** Command: spin the wheel to the given velocity. Never finishes. */
  public Command goToSpeed(AngularVelocity speed) {
    Logger.recordOutput(settings.name + "/CommandedSpeedRPM", speed.in(RPM));
    return Commands.run(() -> {
      mode = OutputMode.GOAL;
      goalRadPerSec = speed.in(RadiansPerSecond);
    }, this).withName(settings.name + " GoToSpeed");
  }

  /**
   * Continuously commands the wheel to {@code speed} from a periodic loop (no command).
   * For owners that re-assert a moving setpoint every cycle, e.g. a launcher tracking a shot
   * solution.
   */
  public void track(AngularVelocity speed) {
    mode = OutputMode.GOAL;
    goalRadPerSec = speed.in(RadiansPerSecond);
  }

  /** Command: open-loop duty cycle (battery-compensated onboard). Coast when it ends. */
  public Command setDutyCycle(double dutyCycle) {
    return Commands.run(() -> {
      mode = OutputMode.VOLTAGE;
      io.setDutyCycle(dutyCycle);
    }, this).finallyDo(() -> {
      mode = OutputMode.IDLE;
      io.stop();
    }).withName(settings.name + " DutyCycle");
  }

  /** Command: SysId routine for characterizing kS/kV/kA (wheel spins freely). */
  public Command sysId() {
    return Commands.sequence(
            Commands.runOnce(() -> mode = OutputMode.VOLTAGE),
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward),
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward))
        .finallyDo(() -> {
          mode = OutputMode.IDLE;
          io.stop();
        })
        .withName(settings.name + " SysId");
  }

  /** Current wheel speed (from the AdvantageKit inputs — replay-safe). */
  public AngularVelocity getSpeed() {
    return RadiansPerSecond.of(getVelocityRadPerSec());
  }

  /** Trigger: true while the wheel is within {@code tolerance} of {@code speed} (ready to shoot). */
  public Trigger isAtSpeed(AngularVelocity speed, AngularVelocity tolerance) {
    return new Trigger(() -> Math.abs(getVelocityRadPerSec() - speed.in(RadiansPerSecond))
        <= tolerance.in(RadiansPerSecond));
  }

  /** Convenience trigger with a 100 RPM tolerance. */
  public Trigger isAtSpeed(AngularVelocity speed) {
    return isAtSpeed(speed, RPM.of(100));
  }

  /** The settings this mechanism was built with. */
  public Settings getSettings() {
    return settings;
  }

  /**
   * The live robot-frame pose where a child mechanism mounts: the wheel centre. Pass
   * {@code flywheel::attachmentPose} as another mechanism's {@code visualParent}.
   */
  public edu.wpi.first.math.geometry.Pose3d attachmentPose() {
    return MechanismVisuals3d.resolveMount(
        settings.visualPose3d, settings.visualParent, settings.visualParentOffset);
  }

  /** Stops control and frees the CAN device. For unit tests. */
  @Override
  public void close() {
    MechanismVisuals3d.remove(settings.name);
    io.close();
  }
}
