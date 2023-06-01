package frc.lib.vision;

import java.util.ArrayList;
import java.util.Optional;

public class CameraInterfaces {
    public static interface GenericCamera {
        void update();
    }

    public static interface Retroreflective extends GenericCamera {
        Optional<LimelightRawAngles> getLimelightRawAngles();

        default boolean hasLimelightRawAngles() {
            return getLimelightRawAngles().isPresent();
        }
    }

    public static interface MachineLearning extends GenericCamera {
        // yes i know this class is the same as the retroreflective one, its a semantics thing

        Optional<LimelightRawAngles> getMLRawAngles();

        default boolean hasMLRawAngles() {
            return getMLRawAngles().isPresent();
        }
    }

    public static interface RetroreflectiveArray extends GenericCamera {
        ArrayList<LimelightRawAngles> getLimelightRawAnglesArray();

        default boolean hasLimelightRawAnglesArray() {
            return !getLimelightRawAnglesArray().isEmpty();
        }
    }

    public static interface ApriltagEstimator extends GenericCamera {
        Optional<LimelightRobotPose> getRobotPoseEstimate();

        default boolean hasRobotPoseEstimate() {
            return getRobotPoseEstimate().isPresent();
        }
    }
}
