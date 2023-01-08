package frc.lib.logging;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Constants;

public class LoggableDouble {
    DoubleTopic topic;
    DoublePublisher publisher;
    DoubleSubscriber subscriber;
    DoubleLogEntry logger;
    double defaultValue;
    boolean override;

    /**
     * @param path The full name of the double, e.g. "/MySubsystem/MyThing"
     * @param defaultValue
     * @param override
     */
    public LoggableDouble(String path, double defaultValue, boolean override) {
        this.defaultValue = defaultValue;

        topic = NetworkTableInstance.getDefault().getDoubleTopic(path);
        logger = new DoubleLogEntry(DataLogManager.getLog(), path);
        this.override = override;
    }

    public LoggableDouble(String path, double defaultValue) {
        this(path, defaultValue, !Constants.competitionMode);
    }

    public LoggableDouble(String path, boolean override) {
        this(path, 0.0, override);
    }

    public LoggableDouble(String path) {
        this(path, 0.0, !Constants.competitionMode);
    }

    public void set(double value) {
        // Lazily create a publisher
        if (publisher == null) publisher = topic.publish();

        if (override) publisher.set(value);

        logger.append(value);
    }

    public double get() {
        // Lazily create a subscriber
        if (subscriber == null) subscriber = topic.subscribe(defaultValue);

        var value = subscriber.get();
        logger.append(value);

        return value;
    }
}
