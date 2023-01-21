package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logging.LoggableDouble;
import frc.lib.logging.LoggableInteger;
import frc.lib.logging.LoggablePose;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
    private final String GLOBAL_SHUTTER_CAMERA = "Global_Shutter_Camera";

    PhotonCamera camera = new PhotonCamera(GLOBAL_SHUTTER_CAMERA);

    private PhotonVisionResult photonVisionResult = new PhotonVisionResult(false, -1, 0, 0, new Transform3d(), new Pose3d());
    private LimelightResult limelightResult;

    // PhotonTrackedTarget currentTarget = null;

    // Data to be updated on every loop
    // Pose3d aprilTagPose = new Pose3d();
    // Transform3d cameraToTarget = new Transform3d();
    // boolean hasTargets = false;
    // int fiducialId = -1;
    // double ambiguity = 0.0;
    // double timestamp = 0.0;

    // LoggablePose poseEstimateLogger = new LoggablePose("/VisionSubsystem/Pose");
    // LoggableInteger fiducialIdLogger = new LoggableInteger("/VisionSubsystem/Fiducial Id");
    // LoggableDouble ambiguityLogger = new LoggableDouble("/VisionSubsystem/Ambiguity");

    public boolean photonHasTargets() {
        return photonVisionResult.hasTargets;
    }

    public double getPhotonAmbiguity() {
        return photonVisionResult.ambiguity;
    }

    public double getPhotonTimestamp() {
        return photonVisionResult.timestamp;
    }

    public Pose3d getPhotonAprilTagFieldPose() {
        return photonVisionResult.aprilTagPose;
    }

    public Transform3d getPhotonCameraToTarget() {
        return photonVisionResult.cameraToTarget;
    }

    public Pose2d getPhotonRobotPoseEstimate() {
        var targetToCamera = getPhotonCameraToTarget().inverse();

        var robotPoseEstimate = getPhotonAprilTagFieldPose()
                .transformBy(targetToCamera)
                .transformBy(VisionConstants.cameraToRobot)
                .toPose2d();

        return robotPoseEstimate;
    }

    public Pose2d getPhotonRobotRelativeTargetPose(Pose2d robotPose) {
        var robotPose3d = new Pose3d(robotPose);

        return robotPose3d
                .transformBy(VisionConstants.robotToCamera)
                .transformBy(getPhotonCameraToTarget())
                .toPose2d();
    }

    @Override
    public void periodic() {
        updateUsingPhotonvision();
        // updateUsingLimelight();
    }

    private void updateUsingPhotonvision() {
        var latestResult = camera.getLatestResult();

        var hasTargets = latestResult.hasTargets();

        if (!hasTargets) return;

        PhotonTrackedTarget currentTarget = latestResult.getBestTarget();
        var fiducialId = currentTarget.getFiducialId();
        var ambiguity = currentTarget.getPoseAmbiguity();
        var timestamp = latestResult.getTimestampSeconds();

        // ambiguityLogger.set(ambiguity);
        // fiducialIdLogger.set(fiducialId);

        var cameraToTarget = currentTarget.getBestCameraToTarget();

        Optional<Pose3d> currentTargetFieldPose = FieldConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(fiducialId);

        Pose3d aprilTagPose = null;

        if (currentTargetFieldPose.isPresent()) {
            aprilTagPose = currentTargetFieldPose.get();

            // poseEstimateLogger.set(aprilTagPose);
        }

        photonVisionResult =
                new PhotonVisionResult(hasTargets, fiducialId, ambiguity, timestamp, cameraToTarget, aprilTagPose);
    }

    private void updateUsingLimelight() {
        var hasTargets = getEntry("tv").getInteger(0) == 1;

        if (!hasTargets) return;

        var fiducialId = (int) getEntry("fid").getInteger(0);
        var timestamp = Timer.getFPGATimestamp() - 0.001 * getEntry("tl").getInteger(0);

        var camtran = getEntry("camtran").getDoubleArray(new double[] {});

        // Transform3d targetTransform = new Transform3d(
        //         new Translation3d(camtran[0], camtran[1], camtran[2]),
        //         new Rotation3d(camtran[3], camtran[4], camtran[5]));

        // robotRelativePose = new Pose3d().transformBy(targetTransform);

        Optional<Pose3d> currentTargetFieldPose = FieldConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(fiducialId);

        if (currentTargetFieldPose.isPresent()) {
            // globalRobotPose = currentTargetFieldPose.get().transformBy(targetTransform.inverse());

            // poseEstimateLogger.set(globalRobotPose);
        }
    }

    // public static Pose3d allianceFlip(Pose3d pose) {
    //     if (DriverStation.getAlliance() == Alliance.Red) {
    //       return new Pose2d(
    //           fieldLength - pose.getX(),
    //           pose.getY(),
    //           new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
    //     } else {
    //       return pose;
    //     }
    //   }

    private NetworkTableEntry getEntry(String key) {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry(key);
    }

    public record PhotonVisionResult(
                boolean hasTargets,
                int fiducialId,
                double ambiguity,
                double timestamp,
                Transform3d cameraToTarget,
                Pose3d aprilTagPose) {
    }

    public record LimelightResult(
                boolean hasTargets,
                int fiducialId,
                double ambiguity,
                double timestamp,
                Transform3d cameraToTarget,
                Pose3d aprilTagPose) {
    }
}
