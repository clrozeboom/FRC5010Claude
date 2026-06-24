package org.frc5010.common.profiles;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

/**
 * A named autonomous routine entry for the auto chooser.
 *
 * <p>Pass a list of these to {@link SwerveRobotContainer#registerAutos} from
 * {@link SwerveRobotContainer#buildAutos()} to register all autos in one call.
 *
 * @param name       display name shown in the chooser and web UI dropdown
 * @param factory    called once per selection change to build the autonomous command
 * @param blueStart  blue-alliance starting pose, or {@code null} to use the profile default
 */
public record AutoEntry(String name, Supplier<Command> factory, Pose2d blueStart) {}
