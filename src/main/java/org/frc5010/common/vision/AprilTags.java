// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

/**
 * Single shared holder for the active AprilTag field layout, so field-geometry helpers and
 * {@code VisionFactory} all resolve tag poses against <b>one</b> layout and never diverge.
 *
 * <p><b>The active {@link org.frc5010.common.profiles.RobotProfile} is the authority on the
 * field.</b> {@link org.frc5010.common.profiles.SwerveRobotContainer} calls
 * {@link #setAprilTagFieldLayout} with {@code profile.getAprilTagFieldLayout()} before it
 * builds the drive and vision, so this holder reflects the profile's chosen variant (e.g. a
 * profile that overrides {@code getAprilTagFieldLayout()} to compete on a non-default field
 * variant). The static default below is only a fallback for code that runs without a profile
 * (some unit tests): WPILib's current-season default ({@code kDefaultField}).
 *
 * <pre>
 *   AprilTagFieldLayout layout = AprilTags.aprilTagFieldLayout;
 *   Optional&lt;Pose3d&gt; tagPose = layout.getTagPose(26);
 * </pre>
 *
 * @see AprilTagFieldLayout
 */
public final class AprilTags {

  /** Active layout — set from the profile by the container; defaults to WPILib's default field. */
  public static AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  private AprilTags() {}

  /**
   * Sets the active field layout. Called by {@code SwerveRobotContainer} from the profile, and
   * available directly for a practice/lab field. Call before anything reads field constants or
   * builds vision.
   */
  public static void setAprilTagFieldLayout(AprilTagFieldLayout layout) {
    aprilTagFieldLayout = layout;
  }
}
