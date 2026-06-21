package frc.robot.rebuilt.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.rebuilt.HubTracker;
import org.littletonrobotics.junction.Logger;

/**
 * Driver-display subsystem that surfaces the {@link HubTracker} shift schedule for the
 * dashboard. Logs the current shift, time remaining in it, and whether the robot's alliance
 * hub is active right now — the strategy read-out the source robot's {@code HubStatus}
 * provided (the rumble-on-shift-change is stubbed in the source and omitted here).
 */
public class HubStatus extends SubsystemBase {

  public HubStatus() {
    setName("HubStatus");
  }

  @Override
  public void periodic() {
    var shift = HubTracker.getCurrentShift();
    Logger.recordOutput("HubStatus/CurrentShift", shift.map(Enum::name).orElse("NONE"));
    Logger.recordOutput(
        "HubStatus/TimeRemainingInShift",
        HubTracker.timeRemainingInCurrentShift().map(t -> t.in(Seconds)).orElse(-1.0));
    Logger.recordOutput("HubStatus/HubActive", HubTracker.isActive());
    Logger.recordOutput("HubStatus/MatchTime", HubTracker.getMatchTime());
    Logger.recordOutput(
        "HubStatus/AutoWinner",
        HubTracker.getAutoWinner().map(DriverStation.Alliance::name).orElse("UNKNOWN"));
  }

  /** Whether the robot's alliance hub is currently scorable. */
  public boolean isHubActive() {
    return HubTracker.isActive();
  }
}
