package frc.lib.logging;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Constants;

public class LoggableDoubleArray {
    DoubleArrayTopic topic;
    DoubleArrayPublisher publisher;
    DoubleArraySubscriber subscriber;
    DoubleArrayLogEntry logger;
    double[] defaultValue;
    boolean override;

    /**
     * @param path The full name of the array, e.g. "/MySubsystem/MyThing"
     * @param override
     * @param defaultValue
     */
    public LoggableDoubleArray(String path, boolean override, double[] defaultValue) {
        this.defaultValue = defaultValue;

        topic = NetworkTableInstance.getDefault().getDoubleArrayTopic(path);
        logger = new DoubleArrayLogEntry(DataLogManager.getLog(), path);
        this.override = override;
    }

    public LoggableDoubleArray(String path, double[] defaultValue) {
        this(path, !Constants.competitionMode, defaultValue);
    }

    public LoggableDoubleArray(String path, boolean override) {
        this(path, override, new double[] {});
    }

    public LoggableDoubleArray(String path) {
        this(path, !Constants.competitionMode, new double[] {});
    }

    public void set(double[] value) {
        // Lazily create a publisher
        if (publisher == null) publisher = topic.publish();

        if (override) publisher.set(value);

        logger.append(value);
    }

    public double[] get() {
        // Lazily create a subscriber
        if (subscriber == null) subscriber = topic.subscribe(defaultValue);

        var value = subscriber.get();
        logger.append(value);

        return value;
    }
}
