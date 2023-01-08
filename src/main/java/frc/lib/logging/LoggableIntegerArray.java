package frc.lib.logging;

import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.IntegerArrayTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.IntegerArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Constants;

public class LoggableIntegerArray {
    IntegerArrayTopic topic;
    IntegerArrayPublisher publisher;
    IntegerArraySubscriber subscriber;
    IntegerArrayLogEntry logger;
    long[] defaultValue;
    boolean override;

    /**
     * @param path The full name of the array, e.g. "/MySubsystem/MyThing"
     * @param defaultValue
     * @param override
     */
    public LoggableIntegerArray(String path, long[] defaultValue, boolean override) {
        this.defaultValue = defaultValue;

        topic = NetworkTableInstance.getDefault().getIntegerArrayTopic(path);
        logger = new IntegerArrayLogEntry(DataLogManager.getLog(), path);

        this.override = override;
    }

    public LoggableIntegerArray(String path, long[] defaultValue) {
        this(path, defaultValue, !Constants.competitionMode);
    }

    public LoggableIntegerArray(String path, boolean override) {
        this(path, new long[] {}, override);
    }

    public LoggableIntegerArray(String path) {
        this(path, new long[] {}, !Constants.competitionMode);
    }

    public void set(long[] value) {
        // Lazily create a publisher
        if (publisher == null) publisher = topic.publish();

        if (override) publisher.set(value);

        logger.append(value);
    }

    public long[] get() {
        // Lazily create a subscriber
        if (subscriber == null) subscriber = topic.subscribe(defaultValue);

        var value = subscriber.get();
        logger.append(value);

        return value;
    }
}
