package frc.lib.logging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Constants;

public class LoggablePose {
    DoubleArrayPublisher publisher;
    DoubleArrayLogEntry logger;
    boolean override;

    /**
     * @param path The full name of the array, e.g. "/MySubsystem/MyThing"
     * @param defaultValue
     * @param override
     */
    public LoggablePose(String path, boolean override) {
        publisher = NetworkTableInstance.getDefault().getDoubleArrayTopic(path).publish();
        logger = new DoubleArrayLogEntry(DataLogManager.getLog(), path);
        this.override = override;
    }

    public LoggablePose(String path) {
        this(path, !Constants.competitionMode);
    }

    public void set(Pose2d value) {
        var valueAsArray = toDoubleArray(value);

        if (override) publisher.set(valueAsArray);

        logger.append(valueAsArray);
    }

    public void set(Pose3d value) {
        var valueAsArray = toDoubleArray(value);

        if (override) publisher.set(valueAsArray);

        logger.append(valueAsArray);
    }

    private double[] toDoubleArray(Pose2d pose) {
        var doubleArray =
                new double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()};
        return doubleArray;
    }

    private double[] toDoubleArray(Pose3d pose) {
        var doubleArray = new double[] {
            pose.getX(),
            pose.getY(),
            pose.getY(),
            pose.getRotation().getX(),
            pose.getRotation().getY(),
            pose.getRotation().getZ()
        };
        return doubleArray;
    }
}
