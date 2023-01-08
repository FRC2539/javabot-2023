package frc.lib.logging;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringArraySubscriber;
import edu.wpi.first.networktables.StringArrayTopic;
import edu.wpi.first.util.datalog.StringArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Constants;

public class LoggableStringArray {
    StringArrayTopic topic;
    StringArrayPublisher publisher;
    StringArraySubscriber subscriber;
    StringArrayLogEntry logger;
    String[] defaultValue;
    boolean override;

    /**
     * @param path The full name of the double, e.g. "/MySubsystem/MyThing"
     * @param defaultValue
     * @param override
     */
    public LoggableStringArray(String path, String[] defaultValue, boolean override) {
        this.defaultValue = defaultValue;

        topic = NetworkTableInstance.getDefault().getStringArrayTopic(path);
        logger = new StringArrayLogEntry(DataLogManager.getLog(), path);

        this.override = override;
    }

    public LoggableStringArray(String path, String[] defaultValue) {
        this(path, defaultValue, !Constants.competitionMode);
    }

    public LoggableStringArray(String path, boolean override) {
        this(path, new String[] {}, override);
    }

    public LoggableStringArray(String path) {
        this(path, new String[] {}, !Constants.competitionMode);
    }

    public void set(String[] value) {
        // Lazily create a publisher
        if (publisher == null) publisher = topic.publish();

        if (override) publisher.set(value);

        logger.append(value);
    }

    public String[] get() {
        // Lazily create a subscriber
        if (subscriber == null) subscriber = topic.subscribe(defaultValue);

        var value = subscriber.get();
        logger.append(value);

        return value;
    }
}
