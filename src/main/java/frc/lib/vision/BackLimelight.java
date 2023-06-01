package frc.lib.vision;
import frc.lib.logging.Logger;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.logging.LoggedReceiver;
import frc.lib.vision.CameraInterfaces.*;
import frc.robot.Constants.VisionConstants;

public class BackLimelight implements ApriltagEstimator, Retroreflective, RetroreflectiveArray {
    private final double FOVX = 29.8 * 2;
    private final double FOVY = 24.85 * 2;
    
    private LoggedReceiver hasTargetReceiver = Logger.receive("/limelight/tv", 0);
    private LoggedReceiver TXReceiver = Logger.receive("/limelight/tx", 0.0);
    private LoggedReceiver TYReceiver = Logger.receive("/limelight/ty", 0.0);
    private LoggedReceiver areaReceiver = Logger.receive("/limelight/ta", 0.0);
    private LoggedReceiver apriltagIDReceiver = Logger.receive("/limelight/tid", -1);
    private LoggedReceiver botposeRedReceiver = Logger.receive("/limelight/botpose_wpired", new double[] {});
    private LoggedReceiver botposeBlueReceiver = Logger.receive("/limelight/botpose_wpiblue", new double[] {});
    private LoggedReceiver pipelineReceiver = Logger.receive("/limelight/getpipe", 0);

    private LoggedReceiver Tx0Receiver = Logger.receive("/limelight/tx0", 0.0);
    private LoggedReceiver Ty0Receiver = Logger.receive("/limelight/ty0", 0.0);
    private LoggedReceiver Ta0Receiver = Logger.receive("/limelight/ta0", 0.0);

    private LoggedReceiver Tx1Receiver = Logger.receive("/limelight/tx1", 0.0);
    private LoggedReceiver Ty1Receiver = Logger.receive("/limelight/ty1", 0.0);
    private LoggedReceiver Ta1Receiver = Logger.receive("/limelight/ta1", 0.0);

    private LoggedReceiver Tx2Receiver = Logger.receive("/limelight/tx2", 0.0);
    private LoggedReceiver Ty2Receiver = Logger.receive("/limelight/ty2", 0.0);
    private LoggedReceiver Ta2Receiver = Logger.receive("/limelight/ta2", 0.0);
    
    private Mode limelightMode = Mode.APRILTAG;

    private Optional<LimelightRobotPose> apriltagEstimate = Optional.empty();
    private Optional<LimelightRawAngles> retroreflectiveAngles = Optional.empty();
    private ArrayList<LimelightRawAngles> allRetroreflectiveAngles = new ArrayList<LimelightRawAngles>();

    public BackLimelight() {
        setMode(Mode.APRILTAG);
    }

    public enum Mode {
        APRILTAG(0),
        RETROREFLECTIVEMID(1),
        RETROREFLECTIVEHIGH(2),
        CONE(3);

        public int pipelineNumber;

        private Mode(int pipelineNumber) {
            this.pipelineNumber = pipelineNumber;
        }
    }

    public void update() {
        apriltagEstimate = calculateApriltagEstimate();
        retroreflectiveAngles = calculateLimelightRawAngles();
        allRetroreflectiveAngles = calculateAllBackRetroreflectiveAngles();
    }

    /** this does NOT account for stuff being really wacky. use your own filtering to treat this */

    private Optional<LimelightRobotPose> calculateApriltagEstimate() {
        if (limelightMode != Mode.APRILTAG) return Optional.empty();

        // double[] {x, y, z, roll, pitch, yaw, latency}
        double[] botposeArray = DriverStation.getAlliance() == Alliance.Red
                ? botposeRedReceiver.getDoubleArray()
                : botposeBlueReceiver.getDoubleArray(); 

        // if botpose exists and the limelight has an april tag, it returns it

        if (apriltagIDReceiver.getInteger() != -1 && botposeArray.length == 7) {
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

            return Optional.of(new LimelightRobotPose(botPose, timestamp, (int) apriltagIDReceiver.getInteger()));
        } else {
            return Optional.empty();
        }
    }

    public Optional<LimelightRobotPose> getRobotPoseEstimate() {
        return apriltagEstimate;
    }

    private Optional<LimelightRawAngles> calculateLimelightRawAngles() {
        if (!(hasTargetReceiver.getInteger() == 1)
                || (limelightMode != Mode.RETROREFLECTIVEMID
                        && limelightMode != Mode.RETROREFLECTIVEHIGH
                        && limelightMode != Mode.CONE)) return Optional.empty();

        double limelightTX = TXReceiver.getDouble();
        double limelightTY = TYReceiver.getDouble();
        double limelightTA = areaReceiver.getDouble();

        // Store raw limelight angles
        return Optional.of(new LimelightRawAngles(limelightTX, limelightTY, limelightTA));
    }

    public Optional<LimelightRawAngles> getLimelightRawAngles() {
        return retroreflectiveAngles;
    }

    private ArrayList<LimelightRawAngles> calculateAllBackRetroreflectiveAngles() {
        if (!isAtPipeline()) return new ArrayList<LimelightRawAngles>();
        
        if (!(hasTargetReceiver.getInteger() == 1)
                || (limelightMode != Mode.RETROREFLECTIVEMID
                        && limelightMode != Mode.RETROREFLECTIVEHIGH
                        && limelightMode != Mode.CONE)) return new ArrayList<LimelightRawAngles>();

        var outputArray = new ArrayList<LimelightRawAngles>();

        int MY_THRESHOLD = 1; // in percent (0-100)

        if (Ta0Receiver.getDouble() >= MY_THRESHOLD) {
            outputArray.add(new LimelightRawAngles(
                    Tx0Receiver.getDouble() * FOVX / 2, Ty0Receiver.getDouble() * FOVY / 2));
        }

        if (Ta1Receiver.getDouble() >= MY_THRESHOLD) {
            outputArray.add(new LimelightRawAngles(
                    Tx1Receiver.getDouble() * FOVX / 2, Ty1Receiver.getDouble() * FOVY / 2));
        }

        if (Ta2Receiver.getDouble() >= MY_THRESHOLD) {
            outputArray.add(new LimelightRawAngles(
                    Tx2Receiver.getDouble() * FOVX / 2, Ty2Receiver.getDouble() * FOVY / 2));
        }

        // Store raw limelight angles
        return outputArray;
    }

    public ArrayList<LimelightRawAngles> getLimelightRawAnglesArray() {
        return allRetroreflectiveAngles;
    }

    private boolean isAtPipeline() {
        return pipelineReceiver.getInteger() == limelightMode.pipelineNumber;
    }

    public void setMode(Mode limelightMode) {
        if (limelightMode == this.limelightMode) return;

        NetworkTableInstance.getDefault()
                .getTable("limelight")
                .getEntry("pipeline")
                .setNumber(limelightMode.pipelineNumber);

        this.limelightMode = limelightMode;
    }

    public Mode getMode() {
        return limelightMode;
    }
}
