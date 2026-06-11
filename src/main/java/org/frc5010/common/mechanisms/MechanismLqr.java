package org.frc5010.common.mechanisms;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

/**
 * State-space LQR for a single-DOF mechanism: WPILib {@link LinearSystemLoop}
 * (LQR gain + Kalman filter + plant-inversion feedforward) running on the RIO at
 * 20 ms, called synchronously from the subsystem's {@code periodic()}.
 *
 * <p>All values are in the mechanism's <b>native units</b> — meters for elevators,
 * radians for arms/pivots, rad/s for flywheels. Subsystems convert from mechanism
 * rotations at the boundary.
 *
 * <p>The plant comes from either the physics model (motor + gearing + mass/MOI) or,
 * when {@code characterizedKv/kA > 0}, from SysId measurements via
 * {@code LinearSystemId.identifyPositionSystem} — see docs/mechanisms.md
 * "Characterized plants". WPILib's position plants expose two outputs (position and
 * velocity) but only position is measured, so the Kalman filter is built on
 * {@code plant.slice(0)}.
 */
public class MechanismLqr {

  private final boolean positionLoop;
  private final LinearSystemLoop<N2, N1, N1> positionLoopImpl; // null for flywheels
  private final LinearSystemLoop<N1, N1, N1> velocityLoopImpl; // null for position mechanisms
  private static final double DT = 0.02;

  private MechanismLqr(LinearSystemLoop<N2, N1, N1> position, LinearSystemLoop<N1, N1, N1> velocity) {
    this.positionLoop = position != null;
    this.positionLoopImpl = position;
    this.velocityLoopImpl = velocity;
  }

  /**
   * Builds an elevator LQR (meters).
   *
   * @param motor motor model
   * @param gearing rotor-to-mechanism reduction
   * @param massKg carriage mass
   * @param drumRadiusMeters drum/sprocket radius
   * @param qelmsPos position error tolerance, m
   * @param qelmsVel velocity error tolerance, m/s
   * @param relmsVolts control effort tolerance, V
   * @param modelPosTrust model position std-dev, m
   * @param modelVelTrust model velocity std-dev, m/s
   * @param encoderTrust encoder position std-dev, m
   * @param characterizedKv measured kV (V per m/s) — 0 = use the physics model
   * @param characterizedKa measured kA (V per m/s²)
   * @return position LQR in meters
   */
  public static MechanismLqr elevator(DCMotor motor, double gearing, double massKg,
      double drumRadiusMeters, double qelmsPos, double qelmsVel, double relmsVolts,
      double modelPosTrust, double modelVelTrust, double encoderTrust,
      double characterizedKv, double characterizedKa) {
    LinearSystem<N2, N1, N2> plant = (characterizedKv > 0 && characterizedKa > 0)
        ? LinearSystemId.identifyPositionSystem(characterizedKv, characterizedKa)
        : LinearSystemId.createElevatorSystem(motor, massKg, drumRadiusMeters, gearing);
    return new MechanismLqr(positionLoop(plant, qelmsPos, qelmsVel, relmsVolts,
        modelPosTrust, modelVelTrust, encoderTrust), null);
  }

  /**
   * Builds an arm/pivot LQR (radians).
   *
   * @param motor motor model
   * @param gearing rotor-to-mechanism reduction
   * @param moiKgM2 moment of inertia about the joint
   * @param qelmsPos position error tolerance, rad
   * @param qelmsVel velocity error tolerance, rad/s
   * @param relmsVolts control effort tolerance, V
   * @param modelPosTrust model position std-dev, rad
   * @param modelVelTrust model velocity std-dev, rad/s
   * @param encoderTrust encoder position std-dev, rad
   * @param characterizedKv measured kV (V per rad/s) — 0 = use the physics model
   * @param characterizedKa measured kA (V per rad/s²)
   * @return position LQR in radians
   */
  public static MechanismLqr arm(DCMotor motor, double gearing, double moiKgM2,
      double qelmsPos, double qelmsVel, double relmsVolts,
      double modelPosTrust, double modelVelTrust, double encoderTrust,
      double characterizedKv, double characterizedKa) {
    LinearSystem<N2, N1, N2> plant = (characterizedKv > 0 && characterizedKa > 0)
        ? LinearSystemId.identifyPositionSystem(characterizedKv, characterizedKa)
        : LinearSystemId.createSingleJointedArmSystem(motor, moiKgM2, gearing);
    return new MechanismLqr(positionLoop(plant, qelmsPos, qelmsVel, relmsVolts,
        modelPosTrust, modelVelTrust, encoderTrust), null);
  }

  /**
   * Builds a flywheel velocity LQR (rad/s).
   *
   * @param motor motor model
   * @param gearing rotor-to-mechanism reduction
   * @param moiKgM2 flywheel moment of inertia
   * @param qelmsVel velocity error tolerance, rad/s
   * @param relmsVolts control effort tolerance, V
   * @param modelVelTrust model velocity std-dev, rad/s
   * @param encoderTrust encoder velocity std-dev, rad/s
   * @param characterizedKv measured kV (V per rad/s) — 0 = use the physics model
   * @param characterizedKa measured kA (V per rad/s²)
   * @return velocity LQR in rad/s
   */
  public static MechanismLqr flywheel(DCMotor motor, double gearing, double moiKgM2,
      double qelmsVel, double relmsVolts, double modelVelTrust, double encoderTrust,
      double characterizedKv, double characterizedKa) {
    LinearSystem<N1, N1, N1> plant = (characterizedKv > 0 && characterizedKa > 0)
        ? LinearSystemId.identifyVelocitySystem(characterizedKv, characterizedKa)
        : LinearSystemId.createFlywheelSystem(motor, moiKgM2, gearing);
    var observer = new KalmanFilter<>(Nat.N1(), Nat.N1(), plant,
        VecBuilder.fill(modelVelTrust), VecBuilder.fill(encoderTrust), DT);
    var regulator = new LinearQuadraticRegulator<>(plant,
        VecBuilder.fill(qelmsVel), VecBuilder.fill(relmsVolts), DT);
    var loop = new LinearSystemLoop<>(plant, regulator, observer, 12.0, DT);
    return new MechanismLqr(null, loop);
  }

  @SuppressWarnings("unchecked")
  private static LinearSystemLoop<N2, N1, N1> positionLoop(LinearSystem<N2, N1, N2> plant,
      double qelmsPos, double qelmsVel, double relmsVolts,
      double modelPosTrust, double modelVelTrust, double encoderTrust) {
    // Only position is measured: slice the 2-output plant to output 0 for the
    // observer and the loop, so the Kalman dimensions match the measurement.
    LinearSystem<N2, N1, N1> sliced = (LinearSystem<N2, N1, N1>) plant.slice(0);
    var observer = new KalmanFilter<>(Nat.N2(), Nat.N1(), sliced,
        VecBuilder.fill(modelPosTrust, modelVelTrust), VecBuilder.fill(encoderTrust), DT);
    var regulator = new LinearQuadraticRegulator<>(sliced,
        VecBuilder.fill(qelmsPos, qelmsVel), VecBuilder.fill(relmsVolts), DT);
    return new LinearSystemLoop<>(sliced, regulator, observer, 12.0, DT);
  }

  /**
   * Re-seeds the loop at the current state (call when enabling or after retuning).
   *
   * @param positionNative current position (ignored for flywheels)
   * @param velocityNative current velocity
   */
  public void reset(double positionNative, double velocityNative) {
    if (positionLoop) {
      positionLoopImpl.reset(VecBuilder.fill(positionNative, velocityNative));
    } else {
      velocityLoopImpl.reset(VecBuilder.fill(velocityNative));
    }
  }

  /**
   * One 20 ms iteration of a position loop.
   *
   * @param measuredPos measured position, native units
   * @param profilePos  motion-profile position setpoint
   * @param profileVel  motion-profile velocity setpoint
   * @return output voltage (clamped to ±12 V)
   */
  public double calculatePosition(double measuredPos, double profilePos, double profileVel) {
    positionLoopImpl.setNextR(profilePos, profileVel);
    positionLoopImpl.correct(VecBuilder.fill(measuredPos));
    positionLoopImpl.predict(DT);
    return positionLoopImpl.getU(0);
  }

  /**
   * One 20 ms iteration of a velocity loop.
   *
   * @param measuredVel measured velocity, native units
   * @param setpointVel target velocity
   * @return output voltage (clamped to ±12 V)
   */
  public double calculateVelocity(double measuredVel, double setpointVel) {
    velocityLoopImpl.setNextR(setpointVel);
    velocityLoopImpl.correct(VecBuilder.fill(measuredVel));
    velocityLoopImpl.predict(DT);
    return velocityLoopImpl.getU(0);
  }
}
