// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final class VisionConstants {

        // Standard deviation baselines, for 1 meter distance and 1 tag
        // (Adjusted automatically based on distance and # of tags)
        public static double linearStdDevBaseline = 0.02; // Meters
        public static double angularStdDevBaseline = 0.06; // Radians

        public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(); // Transform from camera to robot frame

        public static final String CAMERA_NAME = "photonvision"; // Name of the camera in the PhotonVision dashboard
    }

    public final class AprilTagFieldLayout {

        // Map integer ID to its specific field offset
        private static final Map<Integer, Pose3d> TAG_OFFSETS = Map.of(
            1, new Pose3d(0.0, 2, 0.0, new Rotation3d()), // Example offset for tag ID 1
            2, new Pose3d(1.0, 0.0, 0.0, new Rotation3d()), // Example offset for tag ID 2
            3, new Pose3d(2.0, 0.0, 0.0, new Rotation3d())  // Example offset for tag ID 3
        );

        public static Optional<Pose3d> getOffset(int tagId) {
            return Optional.ofNullable(TAG_OFFSETS.get(tagId));

        }
    }
}
