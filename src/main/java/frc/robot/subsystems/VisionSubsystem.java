package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    private Optional<EstimatedRobotPose> photonEstimatedRobotPose = Optional.empty();

    private LimelightMode limelightMode = LimelightMode.APRILTAG;

    private NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    private IntegerPublisher pipelinePublisher =
            limelightTable.getIntegerTopic("pipeline").publish();
    private IntegerSubscriber limelightHasTargetSubscriber =
            limelightTable.getIntegerTopic("tv").subscribe(0);
    private DoubleSubscriber limelightTXSubscriber =
            limelightTable.getDoubleTopic("tx").subscribe(0);
    private DoubleSubscriber limelightTYSubscriber =
            limelightTable.getDoubleTopic("ty").subscribe(0);
    private DoubleSubscriber limelightApriltagIDSubscriber =
            limelightTable.getDoubleTopic("tid").subscribe(-1);
    private DoubleSubscriber limelightLatencySubscriber =
            limelightTable.getDoubleTopic("tl").subscribe(0);
    private DoubleArraySubscriber botposeSubscriber =
            limelightTable.getDoubleArrayTopic("botpose").subscribe(new double[] {});
    private DoubleArrayPublisher convertedBotposePublisher = NetworkTableInstance.getDefault()
            .getTable("VisionSubsystem")
            .getDoubleArrayTopic("LLPose")
            .publish();

    private BiConsumer<Pose2d, Double> addVisionMeasurement;
    private Supplier<Pose2d> robotPoseSupplier;

    public VisionSubsystem(BiConsumer<Pose2d, Double> addVisionMeasurement, Supplier<Pose2d> robotPoseSupplier) {
        this.addVisionMeasurement = addVisionMeasurement;
        this.robotPoseSupplier = robotPoseSupplier;
    }

    public void setLimelightMode(LimelightMode limelightMode) {
        this.limelightMode = limelightMode;

        pipelinePublisher.accept(limelightMode.pipelineNumber);
    }

    public LimelightMode getLimelightMode() {
        return this.limelightMode;
    }

    public boolean limelightHasTarget() {
        return limelightHasTargetSubscriber.get() == 1;
    }

    public Pose2d getLimelightTargetRetroreflectivePoseEstimate() {
        var distance = (VisionConstants.retroreflectiveHeight - VisionConstants.limelightHeight)
                / Math.tan(VisionConstants.limelightCameraToRobot.getRotation().getY() + limelightTYSubscriber.get());

        return new Pose2d(new Translation2d(distance, limelightTXSubscriber.get()), Rotation2d.fromDegrees(180));
    }

    public boolean photonHasTargets() {
        return photonEstimatedRobotPose.isPresent();
    }

    public double getPhotonTimestamp() {
        return photonEstimatedRobotPose.get().timestampSeconds;
    }

    public Pose2d getPhotonRobotPoseEstimate() {
        return photonEstimatedRobotPose.get().estimatedPose.toPose2d();
    }

    @Override
    public void periodic() {
        // Set the reference pose to the current estimated pose from the swerve drive subsystem
        photonPoseEstimator.setReferencePose(robotPoseSupplier.get());
        photonEstimatedRobotPose = photonPoseEstimator.update();

        if (photonEstimatedRobotPose.isPresent()) {
            EstimatedRobotPose estimatedRobotPose = photonEstimatedRobotPose.get();
            addVisionMeasurement.accept(
                    estimatedRobotPose.estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds);
        }

        var botpose = botposeSubscriber.get();

        if (limelightHasTarget() && botpose.length > 0) {
            var botPose =
                    new Pose3d(botpose[0], botpose[1], botpose[2], new Rotation3d(botpose[3], botpose[4], botpose[5]));
            var convertedBotpose = botPose.transformBy(new Transform3d(
                    new Translation3d(FieldConstants.fieldLength / 2, FieldConstants.fieldWidth / 2, 0),
                    new Rotation3d()));
            convertedBotposePublisher.set(new double[] {
                convertedBotpose.getX(),
                convertedBotpose.getY(),
                convertedBotpose.getRotation().getZ()
            });
        }

        // Basic
        // Get distance from limelight from ty
        // Get angle from tx
        // Make target pose estimate, assume correct placement angle is 180

        // Advanced
        // Calculate robot relative target pose from botpose from ll (apriltags)
        //

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
