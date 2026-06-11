package org.frc5010.common.mechanisms;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import yams.gearing.MechanismGearing;
import yams.math.LQRConfig;

/**
 * {@link LQRConfig} with a corrected Kalman filter for the 2-state ARM and ELEVATOR
 * plants.
 *
 * <p><b>Why this exists:</b> WPILib's {@code createElevatorSystem} /
 * {@code createSingleJointedArmSystem} return plants with <em>two</em> outputs
 * (position and velocity), but only position is measured. YAMS 2026.4.10.3's
 * {@code LQRConfig.getKalmanFilter} passes the unsliced 2-output plant with a
 * 1-dimensional measurement-noise vector, which makes the native DARE solver read a
 * garbage R matrix — failing with "R was not symmetric" or, worse, silently producing
 * useless gains. YAMS main fixes this with {@code plant.slice(0)}; this subclass
 * applies the same fix until a release ships it. FLYWHEEL (1-state) is unaffected and
 * delegates to the parent.
 *
 * <p>Always construct mechanism LQR configs through the static factories here so
 * live retuning ({@code LQRController.updateConfig}) also goes through the fixed path.
 */
public class MechanismLqrConfig extends LQRConfig {

  /** Model std-devs for the 2-state plants; null for FLYWHEEL. */
  private final Vector<N2> modelStdDevs;
  /** Measurement (position) std-dev for the 2-state plants; null for FLYWHEEL. */
  private final Vector<N1> encoderStdDevs;
  /** Measured kV in the loop's native units; 0 = use the physics-model plant. */
  private double characterizedKv;
  /** Measured kA in the loop's native units; 0 = use the physics-model plant. */
  private double characterizedKa;

  private MechanismLqrConfig(DCMotor motor, MechanismGearing gearing, MomentOfInertia moi,
      Vector<N2> modelStdDevs, Vector<N1> encoderStdDevs) {
    super(motor, gearing, moi);
    this.modelStdDevs = modelStdDevs;
    this.encoderStdDevs = encoderStdDevs;
  }

  /**
   * Builds an ARM-type LQR config (also correct for gravity-free pivots/turrets).
   *
   * @param motor   motor model
   * @param gearing rotor → mechanism gearing
   * @param moi     moment of inertia about the joint
   * @param qelmsPosition position error tolerance
   * @param qelmsVelocity velocity error tolerance
   * @param modelPositionTrust model position std-dev
   * @param modelVelocityTrust model velocity std-dev
   * @param encoderPositionTrust encoder position std-dev
   * @param relms control effort tolerance
   * @return arm LQR config with the corrected Kalman filter
   */
  public static MechanismLqrConfig arm(DCMotor motor, MechanismGearing gearing,
      MomentOfInertia moi, Angle qelmsPosition, AngularVelocity qelmsVelocity,
      Angle modelPositionTrust, AngularVelocity modelVelocityTrust,
      Angle encoderPositionTrust, Voltage relms) {
    var config = new MechanismLqrConfig(motor, gearing, moi,
        VecBuilder.fill(modelPositionTrust.in(Radians), modelVelocityTrust.in(RadiansPerSecond)),
        VecBuilder.fill(encoderPositionTrust.in(Radians)));
    config.withArm(qelmsPosition, qelmsVelocity,
        modelPositionTrust, modelVelocityTrust, encoderPositionTrust);
    config.withRelms(relms);
    return config;
  }

  /**
   * Builds an ELEVATOR-type LQR config (linear units, meters).
   *
   * @param motor   motor model
   * @param gearing rotor → drum gearing
   * @param qelmsPosition position error tolerance
   * @param qelmsVelocity velocity error tolerance
   * @param modelPositionTrust model position std-dev
   * @param modelVelocityTrust model velocity std-dev
   * @param encoderPositionTrust encoder position std-dev
   * @param mass carriage mass
   * @param drumRadius drum/sprocket radius
   * @param relms control effort tolerance
   * @return elevator LQR config with the corrected Kalman filter
   */
  public static MechanismLqrConfig elevator(DCMotor motor, MechanismGearing gearing,
      MomentOfInertia moi, Distance qelmsPosition, LinearVelocity qelmsVelocity,
      Distance modelPositionTrust, LinearVelocity modelVelocityTrust,
      Distance encoderPositionTrust, Mass mass, Distance drumRadius, Voltage relms) {
    var config = new MechanismLqrConfig(motor, gearing, moi,
        VecBuilder.fill(modelPositionTrust.in(Meters), modelVelocityTrust.in(MetersPerSecond)),
        VecBuilder.fill(encoderPositionTrust.in(Meters)));
    config.withElevator(qelmsPosition, qelmsVelocity, modelPositionTrust,
        modelVelocityTrust, encoderPositionTrust, mass, drumRadius);
    config.withRelms(relms);
    return config;
  }

  /**
   * Builds a FLYWHEEL-type LQR config (1-state; unaffected by the upstream bug,
   * provided here so all mechanisms construct configs the same way).
   *
   * @param motor   motor model
   * @param gearing rotor → wheel gearing
   * @param moi     flywheel moment of inertia
   * @param qelmsVelocity velocity error tolerance
   * @param modelVelocityTrust model velocity std-dev
   * @param encoderVelocityTrust encoder velocity std-dev
   * @param relms control effort tolerance
   * @return flywheel LQR config
   */
  public static MechanismLqrConfig flywheel(DCMotor motor, MechanismGearing gearing,
      MomentOfInertia moi, AngularVelocity qelmsVelocity,
      AngularVelocity modelVelocityTrust, AngularVelocity encoderVelocityTrust,
      Voltage relms) {
    var config = new MechanismLqrConfig(motor, gearing, moi, null, null);
    config.withFlyWheel(qelmsVelocity, modelVelocityTrust, encoderVelocityTrust);
    config.withRelms(relms);
    return config;
  }

  /**
   * Switches the LQR plant from the physics model (mass/MOI/gearing + ideal motor) to a
   * model <em>identified from a SysId test on the real mechanism</em>
   * ({@code LinearSystemId.identifyPositionSystem} / {@code identifyVelocitySystem}).
   *
   * <p><b>Why:</b> the LQR is only as good as its plant. The physics-model plant assumes
   * the CAD mass/MOI is right and the motor is lossless; measured kV/kA capture the real
   * system — friction, gearbox efficiency, the battery sag your robot actually has. When
   * kV/kA come from SysId, parameters like mass don't need to be known at all: they are
   * implied by how the mechanism actually responded to voltage.
   *
   * @param kv measured velocity gain, volts per unit velocity in the <b>loop's native
   *           units</b>: V/(m/s) for elevators, V/(rad/s) for arms/pivots/flywheels.
   *           The wrapper settings accept SysId-friendly units and convert.
   * @param ka measured acceleration gain, volts per unit acceleration (same unit base)
   * @return this config for chaining
   */
  public MechanismLqrConfig withCharacterizedGains(double kv, double ka) {
    this.characterizedKv = kv;
    this.characterizedKa = ka;
    return this;
  }

  @Override
  public LinearSystem<?, ?, ?> getSystem() {
    if (characterizedKv > 0 && characterizedKa > 0) {
      return switch (getType()) {
        case FLYWHEEL -> LinearSystemId.identifyVelocitySystem(characterizedKv, characterizedKa);
        case ARM, ELEVATOR ->
            LinearSystemId.identifyPositionSystem(characterizedKv, characterizedKa);
      };
    }
    return super.getSystem();
  }

  @Override
  @SuppressWarnings("unchecked")
  public KalmanFilter<?, ?, ?> getKalmanFilter(LinearSystem<?, ?, ?> plant) {
    if (getType() == LQRType.FLYWHEEL) {
      return super.getKalmanFilter(plant);
    }
    // ARM / ELEVATOR: only position is measured — slice the plant to output 0 so the
    // observer's dimensions match the 1-dim measurement noise vector.
    LinearSystem<N2, N1, N1> slicedPlant =
        (LinearSystem<N2, N1, N1>) ((LinearSystem<N2, N1, N2>) plant).slice(0);
    return new KalmanFilter<N2, N1, N1>(
        Nat.N2(),
        Nat.N1(),
        slicedPlant,
        modelStdDevs,
        encoderStdDevs,
        getPeriod().in(Seconds));
  }
}
