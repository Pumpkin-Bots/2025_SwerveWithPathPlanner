package frc.robot;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;

/**
 * Contains various field dimensions and useful reference points. All units are
 * in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {
    private AprilTagFieldLayout fieldLayout  = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    // Example method to get the pose of a specific AprilTag by its ID
    public Optional<Pose3d> getTagPose(int tagID) {
        Optional<Pose3d> tagPose = fieldLayout.getTagPose(tagID);

        if (tagPose.isPresent()) {
            Pose3d pose = tagPose.get();
            double x = pose.getX(); // X coordinate in meters
            double y = pose.getY(); // Y coordinate in meters
            double zRotationDegrees = Math.toDegrees(pose.getRotation().getZ());
            System.out.println("Rotation degrees: " + zRotationDegrees);

            System.out.println("Tag ID " + tagID + " - X: " + x + ", Y: " + y);
        } else {
            System.out.println("Tag ID " + tagID + " not found.");
        }
        return tagPose;
    }
}