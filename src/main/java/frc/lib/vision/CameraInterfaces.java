package frc.lib.vision;

import java.util.ArrayList;
import java.util.Optional;

public class CameraInterfaces {
    public static interface GenericCamera {
        void update();
    }

    public static interface Retroreflective extends GenericCamera {
        Optional<LimelightRawAngles> getLimelightRawAngles();
    }

    public static interface MachineLearning extends GenericCamera {
        // yes i know this class is the same as the retroreflective one, its a semantics thing

        Optional<LimelightRawAngles> getMLRawAngles();
    }

    public static interface RetroreflectiveArray extends GenericCamera {
        ArrayList<LimelightRawAngles> getLimelightRawAnglesArray();
    }

    public static interface ApriltagEstimator extends GenericCamera {
        Optional<LimelightRobotPose> getRobotPoseEstimate();
    }
}
