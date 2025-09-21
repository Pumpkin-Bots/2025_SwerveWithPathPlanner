package frc.robot;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;

/**
 * Contains various field dimensions and useful reference points. All units are
 * in meters and poses are defined to have a blue alliance origin.
 */
public final class FieldConstants {
    // Prevent instantiation
    private FieldConstants() {
        throw new UnsupportedOperationException("This is a utility class and cannot be instantiated");
    }

    private static AprilTagFieldLayout fieldLayout  = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    // Example method to get the pose of a specific AprilTag by its ID
    public static Optional<Pose3d> getTagPose(int tagID) {
        Optional<Pose3d> tagPose = fieldLayout.getTagPose(tagID);

        if (tagPose.isPresent()) {
            Pose3d pose = tagPose.get();
            double x = pose.getX(); // X coordinate in meters
            double y = pose.getY(); // Y coordinate in meters
            double zRotationDegrees = Math.toDegrees(pose.getRotation().getZ());
            //System.out.println("Target Tag ID " + tagID + ": - X: " + x + ", Y: " + y);
            System.out.println("Target Rotation degrees: " + zRotationDegrees + " Target Tag ID " + tagID + ": - X: " + x + ", Y: " + y);
        } else {
            System.out.println("Tag ID " + tagID + " not found.");
        }
        return tagPose;
    }

    public static boolean fieldElementIsBlueAlliance(int apriltagID) {
        return apriltagID > 11 && apriltagID < 23;
    }

}
