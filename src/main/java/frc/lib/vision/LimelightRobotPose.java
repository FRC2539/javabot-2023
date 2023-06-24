package frc.lib.vision;

import java.util.OptionalInt;

import edu.wpi.first.math.geometry.Pose3d;

public class LimelightRobotPose {
    public final Pose3d estimatedPose;
    public final double timestampSeconds;
    public final OptionalInt tagID;

    public LimelightRobotPose(Pose3d estimatedPose, double timestampSeconds, int tagID) {
        this(estimatedPose, timestampSeconds, OptionalInt.of(tagID));
    }

    private LimelightRobotPose(Pose3d estimatedPose, double timestampSeconds, OptionalInt tagID) {
        this.estimatedPose = estimatedPose;
        this.timestampSeconds = timestampSeconds;
        this.tagID = tagID;
    }

    public LimelightRobotPose(Pose3d estimatedPose, double timestampSeconds) {
        this(estimatedPose, timestampSeconds, OptionalInt.empty());
    }
}
