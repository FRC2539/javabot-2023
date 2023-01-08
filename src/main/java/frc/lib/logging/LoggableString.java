package frc.lib.logging;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Constants;

public class LoggableString {
    StringTopic topic;
    StringPublisher publisher;
    StringSubscriber subscriber;
    StringLogEntry logger;
    String defaultValue;
    boolean override;

    /**
     * @param path The full name of the double, e.g. "/MySubsystem/MyThing"
     * @param defaultValue
     * @param override
     */
    public LoggableString(String path, String defaultValue, boolean override) {
        this.defaultValue = defaultValue;

        topic = NetworkTableInstance.getDefault().getStringTopic(path);
        logger = new StringLogEntry(DataLogManager.getLog(), path);

        this.override = override;
    }

    public LoggableString(String path, String defaultValue) {
        this(path, defaultValue, !Constants.competitionMode);
    }

    public LoggableString(String path, boolean override) {
        this(path, "", override);
    }

    public LoggableString(String path) {
        this(path, "", !Constants.competitionMode);
    }

    public void set(String value) {
        // Lazily create a publisher
        if (publisher == null) publisher = topic.publish();

        if (override) publisher.set(value);

        logger.append(value);
    }

    public String get() {
        // Lazily create a subscriber
        if (subscriber == null) subscriber = topic.subscribe(defaultValue);

        var value = subscriber.get();
        logger.append(value);

        return value;
    }
}
