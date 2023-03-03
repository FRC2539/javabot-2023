package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logging.LoggedReceiver;
import frc.lib.logging.Logger;
import frc.lib.math.MathUtils;
import frc.lib.vision.TimestampedPose;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class VisionSubsystem extends SubsystemBase {
    private PhotonCamera camera = new PhotonCamera(VisionConstants.photonCameraName);
    private PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(
            FieldConstants.APRIL_TAG_FIELD_LAYOUT,
            PoseStrategy.MULTI_TAG_PNP,
            camera,
            VisionConstants.photonRobotToCamera);

    private LimelightMode limelightMode = LimelightMode.APRILTAG;

    private LoggedReceiver limelightHasTargetReceiver = Logger.receive("/limelight/tv", 0);
    private LoggedReceiver limelightTXReceiver = Logger.receive("/limelight/tx", 0.0);
    private LoggedReceiver limelightTYReceiver = Logger.receive("/limelight/ty", 0.0);
    private LoggedReceiver limelightApriltagIDReceiver = Logger.receive("/limelight/tid", -1);
    private LoggedReceiver botposeRedReceiver = Logger.receive("/limelight/botpose_wpired", new double[] {});
    private LoggedReceiver botposeBlueReceiver = Logger.receive("/limelight/botpose_wpiblue", new double[] {});

    private Optional<TimestampedPose> LLApriltagEstimate = Optional.empty();
    private Optional<EstimatedRobotPose> photonVisionEstimate = Optional.empty();
    private Optional<Pose2d> validLLFieldRelativeRetroreflectiveEstimate = Optional.empty();
    private Optional<Pose2d> LLFieldRelativeRetroreflectiveEstimate = Optional.empty();
    private Optional<Pose2d> LLMLFieldPoseEstimate = Optional.empty();
    private Optional<Pose2d> validLLMLFieldPoseEstimate = Optional.empty();
    private Optional<LimelightRawAngles> LLRawAngles = Optional.empty();

    private double lastApriltagUpdateTimestamp = Timer.getFPGATimestamp();

    private BiConsumer<Pose2d, Double> addVisionMeasurement;
    private Supplier<Pose2d> robotPoseSupplier;

    private static boolean visionDisabled = false;

    public VisionSubsystem(BiConsumer<Pose2d, Double> addVisionMeasurement, Supplier<Pose2d> robotPoseSupplier) {
        this.addVisionMeasurement = addVisionMeasurement;
        this.robotPoseSupplier = robotPoseSupplier;
        setLimelightMode(limelightMode);

        setDefaultCommand(defaultLimelightCommand());
    }

    @Override
    public void periodic() {
        var startTimeMS = Timer.getFPGATimestamp() * 1000;

        /* Use limelight apriltag estimate to update robot pose estimator */
        LLApriltagEstimate = calculateLLApriltagEstimate();

        if (LLApriltagEstimate.isPresent()) {
            addVisionPoseEstimate(LLApriltagEstimate.get());

            Logger.log(
                    "/VisionSubsystem/LLApriltagPose",
                    LLApriltagEstimate.get().estimatedPose.toPose2d());

            lastApriltagUpdateTimestamp = Timer.getFPGATimestamp();
        }

        /* Use photonvision apriltag estimate to update robot pose estimator */
        photonVisionEstimate = calculatePhotonVisionEstimate();

        if (photonVisionEstimate.isPresent()) {
            addVisionPoseEstimate(photonVisionEstimate.get());

            Logger.log(
                    "/VisionSubsystem/photonVisionPose",
                    photonVisionEstimate.get().estimatedPose.toPose2d());

            lastApriltagUpdateTimestamp = Timer.getFPGATimestamp();
        }

        /* Estimate the location of the retroreflective target */
        LLFieldRelativeRetroreflectiveEstimate = calculateLLFieldRelativeRetroreflectiveEstimate();

        if (LLFieldRelativeRetroreflectiveEstimate.isPresent()) {
            validLLFieldRelativeRetroreflectiveEstimate = LLFieldRelativeRetroreflectiveEstimate;

            Logger.log("/VisionSubsystem/LLRobotRelativeRetroreflective", LLFieldRelativeRetroreflectiveEstimate.get());
        }

        /* Estimate the location of a game piece detected by ML */
        LLMLFieldPoseEstimate = calculateLLMLFieldPoseEstimate();

        if (LLMLFieldPoseEstimate.isPresent()) {
            validLLMLFieldPoseEstimate = LLMLFieldPoseEstimate;

            Logger.log("/VisionSubsystem/LLMLFieldPoseEstimate", LLMLFieldPoseEstimate.get());
        }

        // Send the time since the last apriltag update to the dashboard
        Logger.log("/VisionSubsystem/Last Update", Timer.getFPGATimestamp() - lastApriltagUpdateTimestamp);

        Logger.log("/VisionSubsystem/Vision Mode", limelightMode.name());

        Logger.log("/VisionSubsystem/LoopDuration", Timer.getFPGATimestamp() * 1000 - startTimeMS);
    }

    public void setLimelightMode(LimelightMode limelightMode) {
        // No need to update the mode if we are already on the correct pipeline
        if (this.limelightMode == limelightMode) return;

        this.limelightMode = limelightMode;

        NetworkTableInstance.getDefault()
                .getTable("limelight")
                .getEntry("pipeline")
                .setNumber(limelightMode.pipelineNumber);
    }

    public LimelightMode getLimelightMode() {
        return this.limelightMode;
    }

    private boolean limelightHasTarget() {
        return limelightHasTargetReceiver.getInteger() == 1;
    }

    public Optional<LimelightRawAngles> getLimelightRawAngles() {
        return LLRawAngles;
    }

    private boolean limelightHasApriltag() {
        return limelightApriltagIDReceiver.getInteger() != -1;
    }

    private void addVisionPoseEstimate(EstimatedRobotPose estimate) {
        addVisionMeasurement.accept(estimate.estimatedPose.toPose2d(), estimate.timestampSeconds);
    }

    private void addVisionPoseEstimate(TimestampedPose estimate) {
        addVisionMeasurement.accept(estimate.estimatedPose.toPose2d(), estimate.timestampSeconds);
    }

    public boolean hasPhotonVisionEstimate() {
        return photonVisionEstimate.isPresent();
    }

    public Optional<EstimatedRobotPose> getPhotonVisionEstimate() {
        return photonVisionEstimate;
    }

    private Optional<EstimatedRobotPose> calculatePhotonVisionEstimate() {
        if (visionDisabled) return Optional.empty();

        var botpose = photonPoseEstimator.update();

        if (botpose.isEmpty() || !isValidPose(botpose.get().estimatedPose)) return Optional.empty();

        return botpose;
    }

    public boolean hasLLFieldRelativeRetroflectiveEstimate() {
        return LLFieldRelativeRetroreflectiveEstimate.isPresent();
    }

    public Optional<Pose2d> getLLFieldRelativeRetroflectiveEstimate() {
        return LLFieldRelativeRetroreflectiveEstimate;
    }

    public Optional<Pose2d> getValidLLRetroreflectiveEstimate() {
        return validLLFieldRelativeRetroreflectiveEstimate;
    }

    /**
     * @return The pose of the retroreflective target using the robot's pose to make it field relative
     */
    private Optional<Pose2d> calculateLLFieldRelativeRetroreflectiveEstimate() {
        if (visionDisabled) return Optional.empty();

        if (!limelightHasTarget()
                || (limelightMode != LimelightMode.RETROREFLECTIVEMID
                        && limelightMode != LimelightMode.RETROREFLECTIVEHIGH)) {

            LLRawAngles = Optional.empty();

            return Optional.empty();
        }

        double limelightTX = limelightTXReceiver.getDouble();
        double limelightTY = limelightTYReceiver.getDouble();

        // Store raw limelight angles
        LLRawAngles = Optional.of(new LimelightRawAngles(limelightTX, limelightTY));

        double retroreflectiveHeight;

        if (limelightMode == LimelightMode.RETROREFLECTIVEMID
                && limelightTY < VisionConstants.retroreflectiveAngleThreshold) {
            retroreflectiveHeight = VisionConstants.lowerRetroreflectiveHeight;
        } else {
            retroreflectiveHeight = VisionConstants.upperRetroreflectiveHeight;
        }

        var distance = (retroreflectiveHeight - VisionConstants.limelightRobotToCamera.getZ())
                / Math.tan(VisionConstants.limelightRobotToCamera.getRotation().getY() + Math.toRadians(limelightTY));

        var robotPose = robotPoseSupplier.get();

        Translation2d cameraToRetroreflective =
                new Translation2d(distance, Rotation2d.fromDegrees(-limelightTX).plus(robotPose.getRotation()));

        Translation2d cameraToRobot = new Translation2d(
                VisionConstants.limelightCameraToRobot.getX(), VisionConstants.limelightCameraToRobot.getY());

        Translation2d relativeTarget = cameraToRetroreflective.minus(cameraToRobot);

        Pose2d fieldPose = new Pose2d(robotPose.getTranslation().minus(relativeTarget), new Rotation2d());

        return Optional.of(fieldPose);
    }

    public boolean hasLLMLFieldPoseEstimate() {
        return LLMLFieldPoseEstimate.isPresent();
    }

    public Optional<Pose2d> getLLMLFieldPoseEstimate() {
        return LLMLFieldPoseEstimate;
    }

    public Supplier<Pose2d> getMLPoseSupplier() {
        return () -> validLLMLFieldPoseEstimate.get();
    }

    /**
     * @return The pose of the machine learning target using the robot's pose to make it field relative
     */
    private Optional<Pose2d> calculateLLMLFieldPoseEstimate() {
        if (visionDisabled) return Optional.empty();

        if (!limelightHasTarget() || limelightMode != LimelightMode.ML) return Optional.empty();

        // Use bottom edge if possible
        double limelightTX = limelightTXReceiver.getDouble();
        double limelightTY = limelightTYReceiver.getDouble();

        // Store raw limelight angles
        LLRawAngles = Optional.of(new LimelightRawAngles(limelightTX, limelightTY));

        // Cones and cubes are always picked up from the floor
        double targetHeight = 0;

        var distance = (targetHeight - VisionConstants.limelightRobotToCamera.getZ())
                / Math.tan(VisionConstants.limelightRobotToCamera.getRotation().getY() + Math.toRadians(limelightTY));

        var robotPose = robotPoseSupplier.get();

        Translation2d cameraToMLTarget =
                new Translation2d(distance, Rotation2d.fromDegrees(-limelightTX).plus(robotPose.getRotation()));

        Translation2d cameraToRobot = new Translation2d(
                VisionConstants.limelightCameraToRobot.getX(), VisionConstants.limelightCameraToRobot.getY());

        Translation2d relativeTarget = cameraToMLTarget.minus(cameraToRobot);

        Pose2d fieldPose = new Pose2d(robotPose.getTranslation().minus(relativeTarget), new Rotation2d());

        return Optional.of(fieldPose);
    }

    public boolean hasLLApriltagEstimate() {
        return LLApriltagEstimate.isPresent();
    }

    public Optional<TimestampedPose> getLLApriltagEstimate() {
        return LLApriltagEstimate;
    }

    private Optional<TimestampedPose> calculateLLApriltagEstimate() {
        if (visionDisabled || getLimelightMode() != LimelightMode.APRILTAG) return Optional.empty();

        // gets the botpose array from the limelight and a timestamp
        double[] botposeArray = DriverStation.getAlliance() == Alliance.Red
                ? botposeRedReceiver.getDoubleArray()
                : botposeBlueReceiver.getDoubleArray(); // double[] {x, y, z, roll, pitch, yaw}

        // if botpose exists and the limelight has an april tag, it adds the pose to our kalman filter
        if (limelightHasApriltag() && botposeArray.length == 7) {
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

            return Optional.of(new TimestampedPose(botPose, timestamp));
        } else {
            return Optional.empty();
        }
    }

    private boolean isValidPose(Pose3d pose) {
        return MathUtils.isInRange(pose.getY(), -5, FieldConstants.fieldWidth + 5)
                && MathUtils.isInRange(pose.getX(), -5, FieldConstants.fieldLength + 5)
                && MathUtils.isInRange(pose.getZ(), 0, 5);
    }

    public Command defaultLimelightCommand() {
        return runOnce(() -> setLimelightMode(LimelightMode.APRILTAG));
    }

    public Command setLimelightModeCommand(LimelightMode limelightMode) {
        return startEnd(() -> setLimelightMode(limelightMode), () -> {});
    }

    public Command customLimelightModeCommand() {
        return startEnd(() -> {}, () -> {});
    }

    public enum LimelightMode {
        APRILTAG(0),
        RETROREFLECTIVEMID(1),
        RETROREFLECTIVEHIGH(2),
        ML(3);

        public int pipelineNumber;

        private LimelightMode(int pipelineNumber) {
            this.pipelineNumber = pipelineNumber;
        }
    }

    public record LimelightRawAngles(double tx, double ty) {}
}
