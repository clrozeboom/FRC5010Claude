package org.frc5010.common.mechanisms;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.frc5010.common.robot.RobotMode;
import org.frc5010.common.tuning.TunableGains;
import org.littletonrobotics.junction.Logger;

/**
 * TalonFX single-jointed arm with selectable control style and REAL / SIM / REPLAY
 * support. Same architecture as {@link Elevator} (see its javadoc): {@link MechanismIO}
 * replay bubble, SIM via {@link SingleJointedArmSim} feeding the Phoenix sim state.
 *
 * <p>LQR style: ARM-type plant (MOI = ⅓·m·L², matching the sim) with a trapezoid
 * profile and a kG·cos(θ) RIO-side feedforward. PROFILED_PID style: onboard
 * MotionMagic with Arm_Cosine gravity compensation. 0° = horizontal.
 */
public class Arm extends SubsystemBase {

  /** Robot-specific arm parameters. */
  public static class Settings {
    /** Mechanism name used for telemetry and tuning tables. */
    public String name = "Arm";
    /** Closed-loop control style: LQR (default) or profiled PID. */
    public ControlStyle controlStyle = ControlStyle.LQR;
    /** CAN ID of the TalonFX. */
    public int canId;
    /** Motor physics model. */
    public DCMotor motorModel = DCMotor.getKrakenX60(1);
    /** Gear reduction stages, rotor → mechanism (e.g. {10, 5} = 50:1). */
    public double[] gearReductionStages = {10, 5};
    /** Arm length, pivot to tip. */
    public Distance length = Meters.of(0.6);
    /** Arm mass (assumed uniform rod for MOI = ⅓·m·L²). */
    public Mass mass = Kilograms.of(4.0);
    /** Lower limit (0° = horizontal). */
    public Angle minAngle = Degrees.of(-30);
    /** Upper limit. */
    public Angle maxAngle = Degrees.of(210);
    /** Arm angle at robot power-on. */
    public Angle startingAngle = Degrees.of(0);
    /** Motion profile cruise velocity — keep below free speed ÷ gearing. */
    public AngularVelocity maxVelocity = DegreesPerSecond.of(180);
    /** Motion profile acceleration. */
    public AngularAcceleration maxAcceleration = DegreesPerSecondPerSecond.of(360);
    /** Gravity feedforward (volts to hold the arm horizontal) — from SysId or sim ramp. */
    public Voltage kG = Volts.of(0);
    /** Stator current limit. */
    public Current statorCurrentLimit = Amps.of(40);

    // --- LQR weights (live-tunable; these are the initial values) ---
    /** Position error tolerance. Smaller = more aggressive. */
    public Angle qelmsPosition = Degrees.of(1.5);
    /** Velocity error tolerance. Smaller = more aggressive. */
    public AngularVelocity qelmsVelocity = DegreesPerSecond.of(20);
    /** Control effort tolerance. Smaller = gentler. 12 V = full battery. */
    public Voltage relms = Volts.of(12);

    // --- PROFILED_PID gains (live-tunable; TalonFX onboard, units = mechanism rotations) ---
    /** Proportional gain, volts per arm rotation of error. */
    public double kP = 30;
    /** Integral gain. */
    public double kI = 0;
    /** Derivative gain. */
    public double kD = 0;
    /** Static friction feedforward, volts (PROFILED_PID only). */
    public double kS = 0;
    /** Velocity feedforward, volts per arm rotation/s (PROFILED_PID only — the LQR provides its own). */
    public double kV = 0;

    // --- Kalman filter trust (rarely changed) ---
    /** Model position standard deviation. */
    public Angle modelPositionTrust = Radians.of(0.015);
    /** Model velocity standard deviation. */
    public AngularVelocity modelVelocityTrust = RadiansPerSecond.of(0.17);
    /** Encoder position standard deviation. */
    public Angle encoderPositionTrust = Radians.of(0.001);

    // --- Characterized plant (optional, LQR style — see docs/mechanisms.md) ---
    /**
     * Measured kV from a SysId run, volts per rotation/s (SysId tool set to Rotations).
     * Leave 0 to use the physics-model plant. When both kV and kA are set, the LQR
     * plant is built from these measured values instead of motor + gearing +
     * {@link #mass}/{@link #length} — the real inertia and losses are implied by how
     * the arm actually responded to voltage, so an unknown mass no longer matters to
     * the controller. (kG still comes from SysId separately — gravity is not part of
     * the linear plant.)
     */
    public double characterizedKv = 0;
    /** Measured kA from a SysId run, volts per rotation/s². See {@link #characterizedKv}. */
    public double characterizedKa = 0;
  }

  private enum OutputMode { IDLE, GOAL, VOLTAGE }

  private final Settings settings;
  private final double gearing;
  private final MechanismIO io;
  private final MechanismIOInputsAutoLogged inputs = new MechanismIOInputsAutoLogged();
  private MechanismLqr lqr; // null in PROFILED_PID style
  private final TrapezoidProfile profile;
  private TrapezoidProfile.State profileState = new TrapezoidProfile.State();
  private final LqrTunables lqrTunables; // null in PROFILED_PID style
  private final TunableGains pidGains; // null in LQR style
  private final SysIdRoutine sysIdRoutine;

  private OutputMode mode = OutputMode.IDLE;
  private double goalRad;
  private boolean wasEnabled = false;

  private final Mechanism2d mech2d;
  private final MechanismLigament2d armLigament;
  private final MechanismLigament2d goalLigament;

  /**
   * Builds the arm subsystem, its IO (per {@link RobotMode}), controller, and sim.
   *
   * @param settings robot-specific arm parameters
   */
  public Arm(Settings settings) {
    this.settings = settings;
    setName(settings.name);
    gearing = product(settings.gearReductionStages);

    var config = new MechanismIOTalonFX.Config();
    config.canId = settings.canId;
    config.gearing = gearing;
    config.statorCurrentLimitAmps = settings.statorCurrentLimit.in(Amps);
    config.softLimitLowRot = settings.minAngle.in(Radians) / (2 * Math.PI);
    config.softLimitHighRot = settings.maxAngle.in(Radians) / (2 * Math.PI);
    config.startingPositionRot = settings.startingAngle.in(Radians) / (2 * Math.PI);
    config.motionMagicCruiseRotPerSec = settings.maxVelocity.in(RotationsPerSecond);
    config.motionMagicAccelRotPerSecSq =
        settings.maxAcceleration.in(RotationsPerSecond.per(Second));
    config.kP = settings.kP;
    config.kI = settings.kI;
    config.kD = settings.kD;
    config.kS = settings.kS;
    config.kV = settings.kV;
    config.kG = settings.kG.in(Volts);
    config.gravityType = GravityTypeValue.Arm_Cosine;

    io = switch (RobotMode.get()) {
      case REPLAY -> new MechanismIO() {};
      case SIM -> new MechanismIOTalonFXSim(config, armSim());
      case REAL -> new MechanismIOTalonFX(config);
    };

    boolean useLqr = settings.controlStyle == ControlStyle.LQR;
    lqr = useLqr ? buildLqr(
        settings.qelmsPosition.in(Radians),
        settings.qelmsVelocity.in(RadiansPerSecond),
        settings.relms.in(Volts)) : null;
    profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
        settings.maxVelocity.in(RadiansPerSecond),
        settings.maxAcceleration.in(RadiansPerSecond.per(Second))));
    lqrTunables = useLqr
        ? new LqrTunables(settings.name,
            settings.qelmsPosition.in(Radians),
            settings.qelmsVelocity.in(RadiansPerSecond),
            settings.relms.in(Volts))
        : null;
    pidGains = useLqr ? null
        : new TunableGains(settings.name, "pid", settings.kP, settings.kI, settings.kD, 0);

    sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(1).per(Second), Volts.of(3), Seconds.of(10)),
        new SysIdRoutine.Mechanism(
            volts -> io.setVoltage(volts.in(Volts)),
            log -> log.motor(settings.name)
                .voltage(Volts.of(inputs.appliedVolts))
                .angularPosition(Radians.of(getAngleRad()))
                .angularVelocity(RadiansPerSecond.of(getVelocityRadPerSec())),
            this));

    double lengthM = settings.length.in(Meters);
    mech2d = new Mechanism2d(lengthM * 2.5, lengthM * 2.5);
    armLigament = mech2d.getRoot(settings.name + "Root", lengthM * 1.25, lengthM * 1.25)
        .append(new MechanismLigament2d("arm", lengthM, settings.startingAngle.in(Degrees)));
    goalLigament = mech2d.getRoot(settings.name + "GoalRoot", lengthM * 1.25, lengthM * 1.25)
        .append(new MechanismLigament2d("goal", lengthM, settings.startingAngle.in(Degrees), 3,
            new edu.wpi.first.wpilibj.util.Color8Bit(edu.wpi.first.wpilibj.util.Color.kWhite)));
    SmartDashboard.putData(settings.name + "/mechanism", mech2d);
  }

  private MechanismSim armSim() {
    var sim = new SingleJointedArmSim(
        settings.motorModel,
        gearing,
        SingleJointedArmSim.estimateMOI(settings.length.in(Meters), settings.mass.in(Kilograms)),
        settings.length.in(Meters),
        settings.minAngle.in(Radians),
        settings.maxAngle.in(Radians),
        true,
        settings.startingAngle.in(Radians));
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
        return sim.getAngleRads() / (2 * Math.PI);
      }

      @Override
      public double getVelocityRotPerSec() {
        return sim.getVelocityRadPerSec() / (2 * Math.PI);
      }
    };
  }

  private MechanismLqr buildLqr(double qelmsPosRad, double qelmsVelRadPerSec, double relmsVolts) {
    double moi = settings.mass.in(Kilograms) * Math.pow(settings.length.in(Meters), 2) / 3.0;
    return MechanismLqr.arm(
        settings.motorModel,
        gearing,
        moi,
        qelmsPosRad,
        qelmsVelRadPerSec,
        relmsVolts,
        settings.modelPositionTrust.in(Radians),
        settings.modelVelocityTrust.in(RadiansPerSecond),
        settings.encoderPositionTrust.in(Radians),
        // Settings take SysId's rotation units; the arm plant works in radians.
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

  private double getAngleRad() {
    return inputs.positionRot * 2 * Math.PI;
  }

  private double getVelocityRadPerSec() {
    return inputs.velocityRotPerSec * 2 * Math.PI;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(settings.name, inputs);

    if (lqrTunables != null && lqrTunables.hasChanged()) {
      // Tunables are published in rotations; the loop runs in radians.
      lqr = buildLqr(
          lqrTunables.qelmsPosition() * 2 * Math.PI,
          lqrTunables.qelmsVelocity() * 2 * Math.PI,
          lqrTunables.relms());
      resetControlState();
    }
    if (pidGains != null && pidGains.hasChanged()) {
      io.setPidGains(pidGains.kP(), pidGains.kI(), pidGains.kD());
    }

    boolean enabled = DriverStation.isEnabled();
    if (enabled && !wasEnabled) {
      resetControlState();
    }
    wasEnabled = enabled;

    if (!enabled || mode == OutputMode.VOLTAGE) {
      resetProfileToCurrent();
    } else if (mode == OutputMode.GOAL) {
      if (lqr != null) {
        profileState = profile.calculate(0.02, profileState,
            new TrapezoidProfile.State(goalRad, 0));
        double volts = lqr.calculatePosition(getAngleRad(), profileState.position, profileState.velocity)
            + settings.kG.in(Volts) * Math.cos(getAngleRad());
        io.setVoltage(volts);
      } else {
        io.runPosition(goalRad / (2 * Math.PI));
      }
    } else {
      io.stop();
    }

    Logger.recordOutput(settings.name + "/GoalDegrees",
        Math.toDegrees(mode == OutputMode.GOAL ? goalRad : getAngleRad()));
    armLigament.setAngle(Math.toDegrees(getAngleRad()));
    goalLigament.setAngle(Math.toDegrees(mode == OutputMode.GOAL ? goalRad : getAngleRad()));
  }

  private void resetControlState() {
    resetProfileToCurrent();
    if (lqr != null) {
      lqr.reset(getAngleRad(), getVelocityRadPerSec());
    }
  }

  private void resetProfileToCurrent() {
    profileState = new TrapezoidProfile.State(getAngleRad(), getVelocityRadPerSec());
  }

  /** Command: drive the arm to the given angle. Never finishes. */
  public Command goToAngle(Angle angle) {
    Logger.recordOutput(settings.name + "/CommandedAngleDegrees", angle.in(Degrees));
    return Commands.run(() -> {
      mode = OutputMode.GOAL;
      goalRad = angle.in(Radians);
    }, this).withName(settings.name + " GoToAngle");
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
    Trigger nearMax = isAtAngle(settings.maxAngle, Degrees.of(8));
    Trigger nearMin = isAtAngle(settings.minAngle, Degrees.of(8));
    return Commands.sequence(
            Commands.runOnce(() -> mode = OutputMode.VOLTAGE),
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(nearMax),
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(nearMin),
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).until(nearMax),
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).until(nearMin))
        .finallyDo(() -> {
          mode = OutputMode.IDLE;
          io.stop();
        })
        .withName(settings.name + " SysId");
  }

  /** Current arm angle (from the AdvantageKit inputs — replay-safe). */
  public Angle getAngle() {
    return Radians.of(getAngleRad());
  }

  /** Trigger: true while the arm is within {@code tolerance} of {@code angle}. */
  public Trigger isAtAngle(Angle angle, Angle tolerance) {
    return new Trigger(
        () -> Math.abs(getAngleRad() - angle.in(Radians)) <= tolerance.in(Radians));
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
