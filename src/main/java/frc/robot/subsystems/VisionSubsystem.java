package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logging.LoggablePose;
import frc.robot.Constants.FieldConstants;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
    private final String GLOBAL_SHUTTER_CAMERA = "Global_Shutter_Camera";

    PhotonCamera camera = new PhotonCamera(GLOBAL_SHUTTER_CAMERA);

    PhotonTrackedTarget currentTarget = null;

    // Data to be updated on every loop
    Pose3d globalRobotPose = new Pose3d();
    Pose3d robotRelativePose = new Pose3d();
    boolean hasTargets = false;
    int fiducialId = -1;
    double ambiguity = 0.0;
    double timestamp = 0.0;

    LoggablePose poseEstimateLogger = new LoggablePose("/VisionSubsystem/Pose");

    public boolean hasTargets() {
        return hasTargets;
    }

    public double getTimestamp() {
        return timestamp;
    }

    public Pose3d getGlobalPoseEstimate() {
        return globalRobotPose;
    }

    public Pose3d getRelativePoseEstimate() {
        return robotRelativePose;
    }

    @Override
    public void periodic() {
        // updateUsingPhotonvision();
        updateUsingLimelight();
    }

    private void updateUsingLimelight() {
        hasTargets = getEntry("tv").getInteger(0) == 1;

        fiducialId = (int) getEntry("fid").getInteger(0);
        timestamp = Timer.getFPGATimestamp() - 0.001 * getEntry("tl").getInteger(0);

        var camtran = getEntry("camtran").getDoubleArray(new double[] {});

        Transform3d targetTransform = new Transform3d(
                new Translation3d(camtran[0], camtran[1], camtran[2]),
                new Rotation3d(camtran[3], camtran[4], camtran[5]));

        robotRelativePose = new Pose3d().transformBy(targetTransform);

        Optional<Pose3d> currentTargetFieldPose = FieldConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(fiducialId);

        if (currentTargetFieldPose.isPresent()) {
            globalRobotPose = currentTargetFieldPose.get().transformBy(targetTransform.inverse());

            poseEstimateLogger.set(globalRobotPose);
        }
    }

    private void updateUsingPhotonvision() {
        var latestResult = camera.getLatestResult();

        hasTargets = latestResult.hasTargets();

        if (!hasTargets) return;

        currentTarget = latestResult.getBestTarget();
        fiducialId = currentTarget.getFiducialId();
        ambiguity = currentTarget.getPoseAmbiguity();
        timestamp = latestResult.getTimestampSeconds();

        Transform3d targetTransform = currentTarget.getBestCameraToTarget();

        robotRelativePose = new Pose3d().transformBy(targetTransform);

        Optional<Pose3d> currentTargetFieldPose = FieldConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(fiducialId);

        if (currentTargetFieldPose.isPresent()) {
            globalRobotPose = currentTargetFieldPose.get().transformBy(targetTransform.inverse());

            poseEstimateLogger.set(globalRobotPose);
        }
    }

    private NetworkTableEntry getEntry(String key) {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry(key);
    }
}
