package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logging.LoggedReceiver;
import frc.lib.logging.Logger;
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
            PoseStrategy.AVERAGE_BEST_TARGETS,
            camera,
            VisionConstants.photonRobotToCamera);

    private LimelightMode limelightMode = LimelightMode.APRILTAG;

    private LoggedReceiver limelightHasTargetReceiver = Logger.receive("/limelight/tv", 0);
    private LoggedReceiver limelightTXReceiver = Logger.receive("tx", 0);
    private LoggedReceiver limelightTYReceiver = Logger.receive("ty", 0);
    private LoggedReceiver limelightApriltagIDReceiver = Logger.receive("tid", -1);
    private LoggedReceiver limelightLatencyReceiver = Logger.receive("tl", 0);
    private LoggedReceiver botposeReceiver = Logger.receive("botpose", new double[] {});

    private Optional<EstimatedRobotPose> LLApriltagEstimate = Optional.empty();
    private DoubleArrayPublisher LLApriltagPosePublisher = NetworkTableInstance.getDefault()
            .getTable("VisionSubsystem")
            .getDoubleArrayTopic("LLApriltagPose")
            .publish();

    private Optional<EstimatedRobotPose> LLRetroreflectiveEstimate = Optional.empty();
    private DoubleArrayPublisher LLRetroreflectivePosePublisher = NetworkTableInstance.getDefault()
            .getTable("VisionSubsystem")
            .getDoubleArrayTopic("LLRetroreflectivePose")
            .publish();

    private Optional<EstimatedRobotPose> photonVisionEstimate = Optional.empty();
    private DoubleArrayPublisher photonVisionPosePublisher = NetworkTableInstance.getDefault()
            .getTable("VisionSubsystem")
            .getDoubleArrayTopic("photonVisionPose")
            .publish();

    private BiConsumer<Pose2d, Double> addVisionMeasurement;
    private Supplier<Pose2d> robotPoseSupplier;

    public VisionSubsystem(BiConsumer<Pose2d, Double> addVisionMeasurement, Supplier<Pose2d> robotPoseSupplier) {
        this.addVisionMeasurement = addVisionMeasurement;
        this.robotPoseSupplier = robotPoseSupplier;
        setLimelightMode(limelightMode);
    }

    @Override
    public void periodic() {
        LLApriltagEstimate = calculateLLApriltagEstimate();
        if (LLApriltagEstimate.isPresent()) {
            addVisionPoseEstimate(LLApriltagEstimate.get());
            publishPoseEstimate(LLApriltagPosePublisher, LLApriltagEstimate.get());
        }

        LLRetroreflectiveEstimate = calculateLLRetroreflectiveEstimate();
        if (LLRetroreflectiveEstimate.isPresent()) {
            addVisionPoseEstimate(LLRetroreflectiveEstimate.get());
            publishPoseEstimate(LLRetroreflectivePosePublisher, LLApriltagEstimate.get());
        }

        photonVisionEstimate = calculatePhotonVisionEstimate();
        if (photonVisionEstimate.isPresent()) {
            addVisionPoseEstimate(photonVisionEstimate.get());
            publishPoseEstimate(photonVisionPosePublisher, photonVisionEstimate.get());
        }
    }

    public void setLimelightMode(LimelightMode limelightMode) {
        this.limelightMode = limelightMode;

        Logger.log("/limelight/pipeline", limelightMode.pipelineNumber);
    }

    public LimelightMode getLimelightMode() {
        return this.limelightMode;
    }

    private boolean limelightHasTarget() {
        return limelightHasTargetReceiver.getLogValue().getInteger() == 1;
    }

    private boolean limelightHasApriltag() {
        return limelightApriltagIDReceiver.getLogValue().getInteger() != -1;
    }

    private void addVisionPoseEstimate(EstimatedRobotPose estimate) {
        addVisionMeasurement.accept(estimate.estimatedPose.toPose2d(), estimate.timestampSeconds);
    }

    private void publishPoseEstimate(DoubleArrayPublisher publisher, EstimatedRobotPose estimate) {
        publisher.accept(new double[] {
            estimate.estimatedPose.getX(),
            estimate.estimatedPose.getX(),
            estimate.estimatedPose.getRotation().getZ()
        });
    }

    public boolean hasPhotonVisionEstimate() {
        return photonVisionEstimate.isPresent();
    }

    public Optional<EstimatedRobotPose> getPhotonVisionEstimate() {
        return photonVisionEstimate;
    }

    private Optional<EstimatedRobotPose> calculatePhotonVisionEstimate() {
        // Set the reference pose to the current estimated pose from the swerve drive subsystem
        photonPoseEstimator.setReferencePose(robotPoseSupplier.get());
        return photonPoseEstimator.update();
    }

    public boolean hasLLRetroreflectiveEstimate() {
        return LLRetroreflectiveEstimate.isPresent();
    }

    public Optional<EstimatedRobotPose> getLLRetroreflectiveEstimate() {
        return LLRetroreflectiveEstimate;
    }

    private Optional<EstimatedRobotPose> calculateLLRetroreflectiveEstimate() {
        // this probably doesnt work yet
        return Optional.empty();

        // if (!limelightHasTarget()) return Optional.empty();

        // var timestamp = Timer.getFPGATimestamp() - limelightLatencySubscriber.get() / 1000.0;

        // var distance = (VisionConstants.retroreflectiveHeight - VisionConstants.limelightHeight)
        //         / Math.tan(VisionConstants.limelightCameraToRobot.getRotation().getY() +
        // limelightTYSubscriber.get());

        // Pose3d poseEstimate = new Pose3d(new Pose2d(new Translation2d(distance, limelightTXSubscriber.get()),
        // Rotation2d.fromDegrees(180)));

        // return Optional.of(new EstimatedRobotPose(poseEstimate, timestamp));
    }

    public boolean hasLLApriltagEstimate() {
        return LLApriltagEstimate.isPresent();
    }

    public Optional<EstimatedRobotPose> getLLApriltagEstimate() {
        return LLApriltagEstimate;
    }

    private Optional<EstimatedRobotPose> calculateLLApriltagEstimate() {
        if (getLimelightMode() != LimelightMode.APRILTAG) return Optional.empty();

        // gets the botpose array from the limelight and a timestamp
        double[] botpose = botposeReceiver.getLogValue().getDoubleArray(); // double[] {x, y, z, roll, pitch, yaw}
        double timestamp = Timer.getFPGATimestamp()
                - limelightLatencyReceiver.getLogValue().getDouble() / 1000.0;

        // if botpose exists and the limelight has an april tag, it adds the pose to our kalman filter
        if (limelightHasApriltag() && botpose.length == 6) {
            Pose3d botPose = new Pose3d(
                    botpose[0],
                    botpose[1],
                    botpose[2],
                    new Rotation3d(Math.toRadians(botpose[3]), Math.toRadians(botpose[4]), Math.toRadians(botpose[5])));
            Pose3d convertedBotpose = botPose.relativeTo(
                    new Pose3d(-FieldConstants.fieldLength / 2, -FieldConstants.fieldWidth / 2, 0, new Rotation3d()));
            return Optional.of(new EstimatedRobotPose(convertedBotpose, timestamp));
        } else {
            return Optional.empty();
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

    public enum LimelightMode {
        APRILTAG(0),
        RETROREFLECTIVE(1);

        public int pipelineNumber;

        private LimelightMode(int pipelineNumber) {
            this.pipelineNumber = pipelineNumber;
        }
    }
}
