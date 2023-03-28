package frc.lib.vision;

import edu.wpi.first.math.geometry.Pose3d;

public class LimelightRobotPose {
    public final Pose3d estimatedPose;
    public final double timestampSeconds;

    public LimelightRobotPose(Pose3d estimatedPose, double timestampSeconds) {
        this.estimatedPose = estimatedPose;
        this.timestampSeconds = timestampSeconds;
    }
}
