package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logging.LoggablePose;
import frc.robot.Constants.VisionConstants;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
    private final String GLOBAL_SHUTTER_CAMERA = "Global_Shutter_Camera";

    PhotonCamera camera = new PhotonCamera(GLOBAL_SHUTTER_CAMERA);

    boolean hasTargets = false;
    PhotonTrackedTarget currentTarget = null;
    int fiducialId = -1;
    double ambiguity = 0.0;
    double timestamp = 0.0;
    Pose2d estimatedRobotPose = new Pose2d();

    LoggablePose poseEstimateLogger = new LoggablePose("/VisionSubsystem/Pose");

    public boolean hasTarget() {
        return hasTargets;
    }

    public double getTimestamp() {
        return timestamp;
    }

    public Pose2d getPoseEstimate() {
        return estimatedRobotPose;
    }

    @Override
    public void periodic() {
        var latestResult = camera.getLatestResult();

        hasTargets = latestResult.hasTargets();

        if (!hasTargets) return;

        currentTarget = latestResult.getBestTarget();
        fiducialId = currentTarget.getFiducialId();
        ambiguity = currentTarget.getPoseAmbiguity();
        timestamp = latestResult.getTimestampSeconds();

        Transform3d targetTransform = currentTarget.getBestCameraToTarget();
        Optional<Pose3d> currentTargetFieldPose = VisionConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(fiducialId);

        if (currentTargetFieldPose.isPresent()) {
            // Swap y and x to correct for our camera's current mounting position (temp fix)
            Transform3d correctedTransform = new Transform3d(
                    new Translation3d(targetTransform.getY(), targetTransform.getX(), targetTransform.getZ()),
                    new Rotation3d());
            estimatedRobotPose = currentTargetFieldPose
                    .get()
                    .transformBy(correctedTransform.inverse())
                    .toPose2d();

            poseEstimateLogger.set(estimatedRobotPose);
        }
    }
}
