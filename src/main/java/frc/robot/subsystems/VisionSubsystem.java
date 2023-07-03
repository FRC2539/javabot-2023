package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logging.Logger;
import frc.lib.math.MathUtils;
import frc.lib.vision.BackLimelight;
import frc.lib.vision.FrontLimelight;
import frc.lib.vision.LimelightRobotPose;
import frc.robot.Constants.FieldConstants;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
    public double translationStdDevCoefficient = 0.3;
    private final double rotationStdDevCoefficient = 0.9;

    // private PhotonCamera camera;
    // private PhotonPoseEstimator photonPoseEstimator;

    private BackLimelight backLimelight = new BackLimelight();

    private FrontLimelight frontLimelight = new FrontLimelight();

    private double lastApriltagUpdateTimestamp = Timer.getFPGATimestamp();

    private SwerveDriveSubsystem swerveDriveSubsystem;

    public VisionSubsystem(SwerveDriveSubsystem swerveDriveSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;

        getBackLimelight().setMode(BackLimelight.Mode.APRILTAG);
        
        getFrontLimelight().setMode(FrontLimelight.Mode.ML);

        setDefaultCommand(defaultLimelightCommand());

        backLimelight.update();
        frontLimelight.update();

        // Initialize photonvision
        // camera = new PhotonCamera(VisionConstants.photonCameraName);
        // photonPoseEstimator = new PhotonPoseEstimator(
        //         FieldConstants.APRIL_TAG_FIELD_LAYOUT,
        //         PoseStrategy.MULTI_TAG_PNP,
        //         camera,
        //         VisionConstants.photonRobotToCamera);

        // photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void periodic() {
        backLimelight.update();
        frontLimelight.update();

        var startTimeMS = Timer.getFPGATimestamp() * 1000;

        /* Use limelight apriltag estimate to update robot pose estimator */
        var backApriltagEstimate = backLimelight.getRobotPoseEstimate();

        if (backApriltagEstimate.isPresent()) {
            addVisionPoseEstimate(backApriltagEstimate.get());

            Logger.log(
                    "/VisionSubsystem/BackApriltagPose",
                    backApriltagEstimate.get().estimatedPose.toPose2d());

            lastApriltagUpdateTimestamp = Timer.getFPGATimestamp();
        }

        /* Use photonvision apriltag estimate to update robot pose estimator */
        // FrontApriltagEstimate = calculatePhotonVisionEstimate();

        // if (FrontApriltagEstimate.isPresent()) {
        //     addVisionPoseEstimate(FrontApriltagEstimate.get());

        //     Logger.log(
        //             "/VisionSubsystem/FrontApriltagPose",
        //             FrontApriltagEstimate.get().estimatedPose.toPose2d());

        //     lastApriltagUpdateTimestamp = Timer.getFPGATimestamp();
        // }

        // Send the time since the last apriltag update to the dashboard
        Logger.log("/VisionSubsystem/Last Update", Timer.getFPGATimestamp() - lastApriltagUpdateTimestamp);

        Logger.log("/VisionSubsystem/Back Vision Mode", backLimelight.getMode().name());
        Logger.log("/VisionSubsystem/Front Vision Mode", frontLimelight.getMode().name());

        Logger.log("/VisionSubsystem/LoopDuration", Timer.getFPGATimestamp() * 1000 - startTimeMS);
    }

    public BackLimelight getBackLimelight() {
        return backLimelight;
    }

    public FrontLimelight getFrontLimelight() {
        return frontLimelight;
    }

    private void addVisionPoseEstimate(EstimatedRobotPose estimate) {
        if (!isValidPose(estimate.estimatedPose)) return;
        
        var estimatedPose = estimate.estimatedPose.toPose2d();

        double averageDistance = 0;

        for (PhotonTrackedTarget target : estimate.targetsUsed) {
            averageDistance += target.getBestCameraToTarget().getTranslation().getNorm();
        }

        averageDistance /= estimate.targetsUsed.size();

        swerveDriveSubsystem.addVisionPoseEstimate(
                estimatedPose, estimate.timestampSeconds, calculateVisionStdDevs(averageDistance));
    }

    private void addVisionPoseEstimate(LimelightRobotPose estimate) {
        if (!isValidPose(estimate.estimatedPose)) return;

        var estimatedPose = estimate.estimatedPose.toPose2d();

        if (estimate.tagID.isEmpty()) return;

        var aprilTagPose =
                FieldConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(estimate.tagID.getAsInt());

        if (aprilTagPose.isPresent()) {
            var distanceFromPrimaryTag =
                    aprilTagPose.get().getTranslation().getDistance(estimate.estimatedPose.getTranslation());

            swerveDriveSubsystem.addVisionPoseEstimate(
                    estimatedPose, estimate.timestampSeconds, calculateVisionStdDevs(distanceFromPrimaryTag));
        }
    }

    private Matrix<N3, N1> calculateVisionStdDevs(double distance) {
        var translationStdDev = translationStdDevCoefficient * Math.pow(distance, 2);
        var rotationStdDev = rotationStdDevCoefficient * Math.pow(distance, 2);

        return VecBuilder.fill(translationStdDev, translationStdDev, rotationStdDev);
    }

    public Command resetPoseWithApriltag() {
        return run(() -> {
            Optional<LimelightRobotPose> output = backLimelight.getRobotPoseEstimate();

            if (output.isPresent()) {

                swerveDriveSubsystem.setPose(output.get().estimatedPose.toPose2d());
            }
        });
    }

    // @Deprecated
    // private Optional<EstimatedRobotPose> calculatePhotonVisionEstimate() {
    //     var botpose = photonPoseEstimator.update();

    //     if (botpose.isEmpty() || !isValidPose(botpose.get().estimatedPose)) return Optional.empty();

    //     return botpose;
    // }

    private boolean isValidPose(Pose3d pose) {
        boolean isWithinField = MathUtils.isInRange(pose.getY(), -5, FieldConstants.fieldWidth + 5)
                && MathUtils.isInRange(pose.getX(), -5, FieldConstants.fieldLength + 5)
                && MathUtils.isInRange(pose.getZ(), 0, 5);

        boolean isNearRobot = swerveDriveSubsystem
                        .getPose()
                        .getTranslation()
                        .getDistance(pose.getTranslation().toTranslation2d())
                < 1.4;

        return isWithinField && isNearRobot;
    }

    public Command defaultLimelightCommand() {
        return runOnce(() -> {
            backLimelight.setMode(BackLimelight.Mode.RETROREFLECTIVEALIGN);
            frontLimelight.setMode(FrontLimelight.Mode.ML);
        });
    }

    public Command setBackLimelightModeCommand(BackLimelight.Mode limelightMode) {
        return startEnd(() -> getBackLimelight().setMode(limelightMode), () -> {});
    }

    public Command customLimelightModeCommand() {
        return startEnd(() -> {}, () -> {});
    }
}
