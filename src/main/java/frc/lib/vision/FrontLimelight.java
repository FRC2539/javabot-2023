package frc.lib.vision;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.lib.logging.LoggedReceiver;
import frc.lib.logging.Logger;
import java.util.Optional;

public class FrontLimelight implements CameraInterfaces.MachineLearning {
    private Optional<LimelightRawAngles> backRetroreflectiveAngles = Optional.empty();

    private LoggedReceiver frontLimelightHasTargetReceiver = Logger.receive("/limelight-ml/tv", 0);
    private LoggedReceiver frontLimelightTXReceiver = Logger.receive("/limelight-ml/tx", 0.0);
    private LoggedReceiver frontLimelightTYReceiver = Logger.receive("/limelight-ml/ty", 0.0);
    private LoggedReceiver frontLimelightPipelineReceiver = Logger.receive("/limelight-ml/getpipe", 0);

    public FrontLimelight() {
        NetworkTableInstance.getDefault()
                .getTable("limelight-ml")
                .getEntry("pipeline")
                .setNumber(0); // 0 is the pipeline for ML
    }

    public void update() {
        backRetroreflectiveAngles = calculateAngles();
    }

    public Optional<LimelightRawAngles> calculateAngles() {
        if (frontLimelightHasTargetReceiver.getInteger() != 1 || frontLimelightPipelineReceiver.getInteger() != 0) {
            return Optional.empty();
        }

        double limelightTX = frontLimelightTXReceiver.getDouble();
        double limelightTY = frontLimelightTYReceiver.getDouble();

        return Optional.of(new LimelightRawAngles(limelightTX, limelightTY));
    }

    public Optional<LimelightRawAngles> getMLRawAngles() {
        return backRetroreflectiveAngles;
    }
}
