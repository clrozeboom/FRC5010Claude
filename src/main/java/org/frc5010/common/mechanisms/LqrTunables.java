package org.frc5010.common.mechanisms;

import org.frc5010.common.tuning.TunableDouble;

/**
 * Live-tunable LQR weights for a YAMS mechanism, published under
 * {@code /Tuning/<table>/} in NetworkTables (see {@link TunableDouble}).
 *
 * <p>LQR is tuned with <em>tolerances</em>, not PID gains:
 * <ul>
 *   <li>{@code qelmsPosition} — how much position error you tolerate
 *       (meters for elevators, rotations for arms/pivots). Smaller = more aggressive.</li>
 *   <li>{@code qelmsVelocity} — how much velocity error you tolerate
 *       (m/s or rotations/s). Smaller = more aggressive. For flywheels this is
 *       the only Q weight (use {@code qelmsVelocity}; position is ignored).</li>
 *   <li>{@code relms} — how much control effort (volts) you allow. Smaller =
 *       less aggressive. 12 V (a full battery) is the standard starting point.</li>
 * </ul>
 *
 * <p>When {@link #hasChanged()} returns true, rebuild the {@code LQRConfig} and call
 * {@code LQRController.updateConfig(...)} — the wrappers in this package do this
 * automatically in {@code periodic()}.
 */
public class LqrTunables {

  private final TunableDouble qelmsPosition;
  private final TunableDouble qelmsVelocity;
  private final TunableDouble relms;

  /**
   * Creates the three LQR weight tunables.
   *
   * @param table         NT table name, typically the mechanism name (e.g. "Elevator")
   * @param qelmsPosition initial position error tolerance (meters or rotations)
   * @param qelmsVelocity initial velocity error tolerance (m/s or rotations/s)
   * @param relms         initial control effort tolerance in volts
   */
  public LqrTunables(String table, double qelmsPosition, double qelmsVelocity, double relms) {
    this.qelmsPosition = new TunableDouble(table, "lqr_qelmsPosition", qelmsPosition);
    this.qelmsVelocity = new TunableDouble(table, "lqr_qelmsVelocity", qelmsVelocity);
    this.relms = new TunableDouble(table, "lqr_relms", relms);
  }

  /** Current position error tolerance (meters or rotations). */
  public double qelmsPosition() {
    return qelmsPosition.get();
  }

  /** Current velocity error tolerance (m/s or rotations/s). */
  public double qelmsVelocity() {
    return qelmsVelocity.get();
  }

  /** Current control effort tolerance in volts. */
  public double relms() {
    return relms.get();
  }

  /**
   * Returns true if ANY weight changed since the last call.
   * Evaluates all three so each changed flag resets — call once per loop.
   */
  public boolean hasChanged() {
    boolean qp = qelmsPosition.hasChanged();
    boolean qv = qelmsVelocity.hasChanged();
    boolean r = relms.hasChanged();
    return qp || qv || r;
  }
}
