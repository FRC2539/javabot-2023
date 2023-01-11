package frc.lib.logging;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Constants;

public class LoggableChassisSpeeds {
    DoubleArrayPublisher publisher;
    DoubleArrayLogEntry logger;
    boolean override;

    /**
     * @param path The full name of the double, e.g. "/MySubsystem/MyThing"
     * @param override
     */
    public LoggableChassisSpeeds(String path, boolean override) {
        publisher = NetworkTableInstance.getDefault().getDoubleArrayTopic(path).publish();
        logger = new DoubleArrayLogEntry(DataLogManager.getLog(), path);
        this.override = override;
    }

    public LoggableChassisSpeeds(String path) {
        this(path, !Constants.competitionMode);
    }

    public void set(ChassisSpeeds value) {
        var valueAsArray = toDoubleArray(value);

        if (override) publisher.set(valueAsArray);

        logger.append(valueAsArray);
    }

    private double[] toDoubleArray(ChassisSpeeds value) {
        var doubleArray = new double[] {value.vxMetersPerSecond, value.vyMetersPerSecond, value.omegaRadiansPerSecond};
        return doubleArray;
    }
}
