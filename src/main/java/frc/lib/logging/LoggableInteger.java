package frc.lib.logging;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Constants;

public class LoggableInteger {
    IntegerTopic topic;
    IntegerPublisher publisher;
    IntegerSubscriber subscriber;
    IntegerLogEntry logger;
    long defaultValue;
    boolean override;

    /**
     * @param path The full name of the double, e.g. "/MySubsystem/MyThing"
     * @param defaultValue
     * @param override
     */
    public LoggableInteger(String path, long defaultValue, boolean override) {
        this.defaultValue = defaultValue;

        topic = NetworkTableInstance.getDefault().getIntegerTopic(path);
        logger = new IntegerLogEntry(DataLogManager.getLog(), path);

        this.override = override;
    }

    public LoggableInteger(String path, long defaultValue) {
        this(path, defaultValue, !Constants.competitionMode);
    }

    public LoggableInteger(String path, boolean override) {
        this(path, 0, override);
    }

    public LoggableInteger(String path) {
        this(path, 0, !Constants.competitionMode);
    }

    public void set(long value) {
        // Lazily create a publisher
        if (publisher == null) publisher = topic.publish();

        if (override) publisher.set(value);

        logger.append(value);
    }

    public long get() {
        // Lazily create a subscriber
        if (subscriber == null) subscriber = topic.subscribe(defaultValue);

        var value = subscriber.get();
        logger.append(value);

        return value;
    }
}
