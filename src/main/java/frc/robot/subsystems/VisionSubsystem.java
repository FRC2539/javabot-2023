package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logging.LoggableDouble;
import frc.lib.logging.LoggablePose;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
    private final String GLOBAL_SHUTTER_CAMERA = "Global_Shutter_Camera";

    PhotonCamera camera = new PhotonCamera(GLOBAL_SHUTTER_CAMERA);

    PhotonTrackedTarget currentTarget = null;

    // Data to be updated on every loop
    Pose3d aprilTagPose = new Pose3d();
    Transform3d cameraToTarget = new Transform3d();
    boolean hasTargets = false;
    int fiducialId = -1;
    double ambiguity = 0.0;
    double timestamp = 0.0;

    public LoggablePose poseEstimateLogger = new LoggablePose("/VisionSubsystem/Pose");

    LoggableDouble ambiguityLogger = new LoggableDouble("/VisionSubsystem/Ambiguity");

    public boolean hasTargets() {
        return hasTargets;
    }

    public double getAmbiguity() {
        return ambiguity;
    }

    public double getTimestamp() {
        return timestamp;
    }

    public Pose3d getAprilTagFieldPose() {
        return aprilTagPose;
    }

    public Transform3d getCameraToTarget() {
        return cameraToTarget;
    }

    public Pose2d getRobotPoseEstimate() {
        var targetToRobot = getCameraToTarget().inverse().plus(VisionConstants.cameraToRobot);

        var robotPoseEstimate =
                getAprilTagFieldPose().transformBy(targetToRobot).toPose2d();

        return robotPoseEstimate;
    }

    @Override
    public void periodic() {
        updateUsingPhotonvision();
        // updateUsingLimelight();
    }

    private void updateUsingPhotonvision() {
        var latestResult = camera.getLatestResult();

        hasTargets = latestResult.hasTargets();

        if (!hasTargets) return;

        currentTarget = latestResult.getBestTarget();
        fiducialId = currentTarget.getFiducialId();
        ambiguity = currentTarget.getPoseAmbiguity();
        timestamp = latestResult.getTimestampSeconds();

        ambiguityLogger.set(ambiguity);

        cameraToTarget = currentTarget.getBestCameraToTarget();

        Optional<Pose3d> currentTargetFieldPose = FieldConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(fiducialId);

        if (currentTargetFieldPose.isPresent()) {
            aprilTagPose = currentTargetFieldPose.get();

            // poseEstimateLogger.set(aprilTagPose);
        }
    }

    // private void updateUsingLimelight() {
    //     hasTargets = getEntry("tv").getInteger(0) == 1;

    //     if (!hasTargets) return;

    //     fiducialId = (int) getEntry("fid").getInteger(0);
    //     timestamp = Timer.getFPGATimestamp() - 0.001 * getEntry("tl").getInteger(0);

    //     var camtran = getEntry("camtran").getDoubleArray(new double[] {});

    //     Transform3d targetTransform = new Transform3d(
    //             new Translation3d(camtran[0], camtran[1], camtran[2]),
    //             new Rotation3d(camtran[3], camtran[4], camtran[5]));

    //     robotRelativePose = new Pose3d().transformBy(targetTransform);

    //     Optional<Pose3d> currentTargetFieldPose = FieldConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(fiducialId);

    //     if (currentTargetFieldPose.isPresent()) {
    //         globalRobotPose = currentTargetFieldPose.get().transformBy(targetTransform.inverse());

    //         poseEstimateLogger.set(globalRobotPose);
    //     }
    // }

    // private NetworkTableEntry getEntry(String key) {
    //     return NetworkTableInstance.getDefault().getTable("limelight").getEntry(key);
    // }
}
