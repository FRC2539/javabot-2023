package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logging.LoggedReceiver;
import frc.lib.logging.Logger;
import frc.lib.math.MathUtils;
import frc.lib.vision.LimelightRobotPose;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
    private final double translationStdDevCoefficient = 0.3;
    private final double rotationStdDevCoefficient = 0.9;

    // private PhotonCamera camera;
    // private PhotonPoseEstimator photonPoseEstimator;

    private LimelightMode backLimelightMode = LimelightMode.APRILTAG;
    private LimelightMode frontLimelightMode = LimelightMode.APRILTAG;

    // Back limelight
    private LoggedReceiver backLimelightHasTargetReceiver = Logger.receive("/limelight/tv", 0);
    private LoggedReceiver backLimelightTXReceiver = Logger.receive("/limelight/tx", 0.0);
    private LoggedReceiver backLimelightTYReceiver = Logger.receive("/limelight/ty", 0.0);
    private LoggedReceiver backLimelightAreaReceiver = Logger.receive("/limelight/ta", 0.0);
    private LoggedReceiver backLimelightApriltagIDReceiver = Logger.receive("/limelight/tid", -1);
    private LoggedReceiver backBotposeRedReceiver = Logger.receive("/limelight/botpose_wpired", new double[] {});
    private LoggedReceiver backBotposeBlueReceiver = Logger.receive("/limelight/botpose_wpiblue", new double[] {});
    private LoggedReceiver backLimelightPipelineReceiver = Logger.receive("/limelight/getpipe", 0);

    // Front limelight
    private LoggedReceiver frontLimelightHasTargetReceiver = Logger.receive("/limelight-ml/tv", 0);
    private LoggedReceiver frontLimelightTXReceiver = Logger.receive("/limelight-ml/tx", 0.0);
    private LoggedReceiver frontLimelightTYReceiver = Logger.receive("/limelight-ml/ty", 0.0);
    private LoggedReceiver frontLimelightPipelineReceiver = Logger.receive("/limelight-ml/getpipe", 0);

    // Note: Front is intake side, Back is arm side
    private Optional<LimelightRobotPose> BackApriltagEstimate = Optional.empty();
    private Optional<EstimatedRobotPose> FrontApriltagEstimate = Optional.empty();
    private Optional<LimelightRawAngles> BackRetroreflectiveAngles = Optional.empty();
    private Optional<LimelightRawAngles> FrontMLAngles = Optional.empty();

    private double lastApriltagUpdateTimestamp = Timer.getFPGATimestamp();

    private SwerveDriveSubsystem swerveDriveSubsystem;

    public VisionSubsystem(SwerveDriveSubsystem swerveDriveSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;

        setBackLimelightMode(backLimelightMode);

        setDefaultCommand(defaultLimelightCommand());

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
        var startTimeMS = Timer.getFPGATimestamp() * 1000;

        /* Use limelight apriltag estimate to update robot pose estimator */
        BackApriltagEstimate = calculateLLApriltagEstimate();

        if (BackApriltagEstimate.isPresent()) {
            addVisionPoseEstimate(BackApriltagEstimate.get());

            Logger.log(
                    "/VisionSubsystem/BackApriltagPose",
                    BackApriltagEstimate.get().estimatedPose.toPose2d());

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

        /* Estimate the location of the retroreflective target */
        BackRetroreflectiveAngles = calculateBackRetroreflectiveAngles();

        /* Estimate the location of a game piece detected by ML */
        FrontMLAngles = calculateFrontMLAngles();

        // Send the time since the last apriltag update to the dashboard
        Logger.log("/VisionSubsystem/Last Update", Timer.getFPGATimestamp() - lastApriltagUpdateTimestamp);

        Logger.log("/VisionSubsystem/Back Vision Mode", backLimelightMode.name());
        Logger.log("/VisionSubsystem/Front Vision Mode", frontLimelightMode.name());

        Logger.log("/VisionSubsystem/LoopDuration", Timer.getFPGATimestamp() * 1000 - startTimeMS);
    }

    public void setBackLimelightMode(LimelightMode limelightMode) {
        // No need to update the mode if we are already on the correct pipeline
        if (this.backLimelightMode == limelightMode) return;

        this.backLimelightMode = limelightMode;

        NetworkTableInstance.getDefault()
                .getTable("limelight")
                .getEntry("pipeline")
                .setNumber(limelightMode.pipelineNumber);
    }

    public void setFrontLimelightMode(LimelightMode limelightMode) {
        // No need to update the mode if we are already on the correct pipeline
        if (this.frontLimelightMode == limelightMode) return;

        this.frontLimelightMode = limelightMode;

        NetworkTableInstance.getDefault()
                .getTable("limelight-ml")
                .getEntry("pipeline")
                .setNumber(limelightMode.pipelineNumber);
    }

    public LimelightMode getBackLimelightMode() {
        return this.backLimelightMode;
    }

    public LimelightMode getFrontLimelightMode() {
        return this.frontLimelightMode;
    }

    public boolean isBackLimelightAtPipeline() {
        return backLimelightPipelineReceiver.getInteger() == backLimelightMode.pipelineNumber;
    }

    public boolean isFrontLimelightAtPipeline() {
        return frontLimelightPipelineReceiver.getInteger() == frontLimelightMode.pipelineNumber;
    }

    public boolean backLimelightHasTarget() {
        return backLimelightHasTargetReceiver.getInteger() == 1;
    }

    public boolean frontLimelightHasTarget() {
        return frontLimelightHasTargetReceiver.getInteger() == 1;
    }

    private boolean hasBackApriltagEstimate() {
        return backLimelightApriltagIDReceiver.getInteger() != -1;
    }

    public boolean hasFrontApriltagEstimate() {
        return FrontApriltagEstimate.isPresent();
    }

    private void addVisionPoseEstimate(EstimatedRobotPose estimate) {
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
        var estimatedPose = estimate.estimatedPose.toPose2d();

        var aprilTagPose =
                FieldConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose((int) backLimelightApriltagIDReceiver.getInteger());

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

    public Optional<EstimatedRobotPose> getFrontApriltagEstimate() {
        return FrontApriltagEstimate;
    }

    public Optional<LimelightRobotPose> getBackApriltagEstimate() {
        return BackApriltagEstimate;
    }

    public Command resetPoseWithApriltag() {
        return run(() -> {
            if (getBackLimelightMode() != LimelightMode.APRILTAG) return;

            // gets the botpose array from the limelight and a timestamp
            double[] botposeArray = DriverStation.getAlliance() == Alliance.Red
                    ? backBotposeRedReceiver.getDoubleArray()
                    : backBotposeBlueReceiver.getDoubleArray(); // double[] {x, y, z, roll, pitch, yaw, latency}

            // if botpose exists and the limelight has an april tag, it adds the pose to our kalman filter
            if (hasBackApriltagEstimate() && botposeArray.length == 7) {
                Pose3d botPose = new Pose3d(
                                botposeArray[0],
                                botposeArray[1],
                                botposeArray[2],
                                new Rotation3d(
                                        Math.toRadians(botposeArray[3]),
                                        Math.toRadians(botposeArray[4]),
                                        Math.toRadians(botposeArray[5])))
                        .transformBy(VisionConstants.limelightCameraToRobot);

                swerveDriveSubsystem.setPose(botPose.toPose2d());
            }
        });
    }

    private Optional<LimelightRawAngles> calculateBackRetroreflectiveAngles() {
        if (!backLimelightHasTarget()
                || (backLimelightMode != LimelightMode.RETROREFLECTIVEMID
                        && backLimelightMode != LimelightMode.RETROREFLECTIVEHIGH
                        && backLimelightMode != LimelightMode.CONE)) return Optional.empty();

        double limelightTX = backLimelightTXReceiver.getDouble();
        double limelightTY = backLimelightTYReceiver.getDouble();
        double limelightTA = backLimelightAreaReceiver.getDouble();

        // Store raw limelight angles
        return Optional.of(new LimelightRawAngles(limelightTX, limelightTY, limelightTA));
    }

    private Optional<LimelightRawAngles> calculateFrontMLAngles() {
        if (!frontLimelightHasTarget() || frontLimelightMode != LimelightMode.ML) return Optional.empty();

        double limelightTX = frontLimelightTXReceiver.getDouble();
        double limelightTY = frontLimelightTYReceiver.getDouble();

        // Store raw limelight angles
        return Optional.of(new LimelightRawAngles(limelightTX, limelightTY));
    }

    public Optional<LimelightRawAngles> getBackRetroreflectiveAngles() {
        return BackRetroreflectiveAngles;
    }

    public Optional<LimelightRawAngles> getFrontMLAngles() {
        return FrontMLAngles;
    }

    public boolean hasBackRetroreflectiveAngles() {
        return BackRetroreflectiveAngles.isPresent();
    }

    public boolean hasFrontMLAngles() {
        return FrontMLAngles.isPresent();
    }

    // private Optional<EstimatedRobotPose> calculatePhotonVisionEstimate() {
    //     var botpose = photonPoseEstimator.update();

    //     if (botpose.isEmpty() || !isValidPose(botpose.get().estimatedPose)) return Optional.empty();

    //     return botpose;
    // }

    private Optional<LimelightRobotPose> calculateLLApriltagEstimate() {
        if (getBackLimelightMode() != LimelightMode.APRILTAG) return Optional.empty();

        // gets the botpose array from the limelight and a timestamp
        double[] botposeArray = DriverStation.getAlliance() == Alliance.Red
                ? backBotposeRedReceiver.getDoubleArray()
                : backBotposeBlueReceiver.getDoubleArray(); // double[] {x, y, z, roll, pitch, yaw, latency}

        // if botpose exists and the limelight has an april tag, it adds the pose to our kalman filter
        if (hasBackApriltagEstimate() && botposeArray.length == 7) {
            Pose3d botPose = new Pose3d(
                            botposeArray[0],
                            botposeArray[1],
                            botposeArray[2],
                            new Rotation3d(
                                    Math.toRadians(botposeArray[3]),
                                    Math.toRadians(botposeArray[4]),
                                    Math.toRadians(botposeArray[5])))
                    .transformBy(VisionConstants.limelightCameraToRobot);

            double timestamp = Timer.getFPGATimestamp() - botposeArray[6] / 1000.0;

            if (!isValidPose(botPose)) return Optional.empty();

            return Optional.of(new LimelightRobotPose(botPose, timestamp));
        } else {
            return Optional.empty();
        }
    }

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
            setBackLimelightMode(LimelightMode.APRILTAG);
            setFrontLimelightMode(LimelightMode.ML);
        });
    }

    public Command setLimelightModeCommand(LimelightMode limelightMode) {
        return startEnd(() -> setBackLimelightMode(limelightMode), () -> {});
    }

    public Command customLimelightModeCommand() {
        return startEnd(() -> {}, () -> {});
    }

    public enum LimelightMode {
        // Back limelight
        APRILTAG(0),
        RETROREFLECTIVEMID(1),
        RETROREFLECTIVEHIGH(2),
        CONE(3),

        // Front limelight
        ML(0);

        public int pipelineNumber;

        private LimelightMode(int pipelineNumber) {
            this.pipelineNumber = pipelineNumber;
        }
    }

    public record LimelightRawAngles(double tx, double ty, double ta) {
        public LimelightRawAngles(double tx, double ty) {
            this(ty, tx, 0.0);
        }
    }
}
