package org.team1126.robot.util;

// Java
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.team1126.robot.subsystems.Swerve;

public final class TagLateralSupplier {

    private final AprilTagFieldLayout layout;
    private final Swerve swerve; // your subsystem exposing getPose()

    public TagLateralSupplier(Swerve swerve, AprilTagFieldLayout layout) {
        this.swerve = swerve;
        this.layout = layout;
    }

    // Returns normalized left/right: 0.0 = far left, 0.5 = center, 1.0 = far right (relative to tag)
    // spanMeters is half-range; e.g., spanMeters=1.5 clamps lateral offset to [-1.5, +1.5].
    public DoubleSupplier normalizedLateralToTag(int tagId, double spanMeters) {
        return () -> {
            Pose2d robot = swerve.getPose();
            Optional<Pose3d> tagPose3d = layout.getTagPose(tagId);
            if (tagPose3d.isEmpty()) return 0.5; // unknown tag -> center as neutral

            Pose2d tag = tagPose3d.get().toPose2d();

            // Transform robot into tag frame
            Transform2d robotInTag = new Transform2d(tag, robot);
            double lateralY = robotInTag.getY(); // left/right relative to tag X axis

            // Clamp to [-span, +span] and normalize to [0, 1]
            double clamped = Math.max(-spanMeters, Math.min(spanMeters, lateralY));
            double normalized = (clamped + spanMeters) / (2.0 * spanMeters);
            return normalized; // 0..1
        };
    }
}

