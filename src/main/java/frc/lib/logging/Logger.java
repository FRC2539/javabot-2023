package frc.lib.logging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.*;
import edu.wpi.first.util.datalog.*;
import edu.wpi.first.wpilibj.DataLogManager;
import java.util.HashMap;
import java.util.Map;

public class Logger {
    // Logger v1 - runs realtime
    // Logger v2 - will run in a separate thread

    private static Logger instance;

    private DataLog log = DataLogManager.getLog();

    private Map<String, Publisher> publisherMap = new HashMap<>();
    private Map<String, DataLogEntry> loggerMap = new HashMap<>();

    private StringPublisher messagesPublisher = NetworkTableInstance.getDefault()
            .getTable("Messages")
            .getStringTopic("messages")
            .publish();

    public static Logger getInstance() {
        if (instance == null) instance = new Logger();

        return instance;
    }

    public void message(String message) {
        messagesPublisher.set(message);
    }

    /* Logger Methods - Log to DataLog and NetworkTables, no getting */

    public void log(String key, boolean value) {
        BooleanPublisher publisher = (BooleanPublisher) publisherMap.get(key);
        BooleanLogEntry logger = (BooleanLogEntry) loggerMap.get(key);

        if (publisher == null || logger == null) {
            publisher = NetworkTableInstance.getDefault().getBooleanTopic(key).publish();
            logger = new BooleanLogEntry(log, key);

            publisherMap.put(key, publisher);
            loggerMap.put(key, logger);
        }

        publisher.set(value);
        logger.append(value);
    }

    public void log(String key, boolean[] value) {
        BooleanArrayPublisher publisher = (BooleanArrayPublisher) publisherMap.get(key);
        BooleanArrayLogEntry logger = (BooleanArrayLogEntry) loggerMap.get(key);

        if (publisher == null || logger == null) {
            publisher =
                    NetworkTableInstance.getDefault().getBooleanArrayTopic(key).publish();
            logger = new BooleanArrayLogEntry(log, key);

            publisherMap.put(key, publisher);
            loggerMap.put(key, logger);
        }

        publisher.set(value);
        logger.append(value);
    }

    public void log(String key, double value) {
        DoublePublisher publisher = (DoublePublisher) publisherMap.get(key);
        DoubleLogEntry logger = (DoubleLogEntry) loggerMap.get(key);

        if (publisher == null || logger == null) {
            publisher = NetworkTableInstance.getDefault().getDoubleTopic(key).publish();
            logger = new DoubleLogEntry(log, key);

            publisherMap.put(key, publisher);
            loggerMap.put(key, logger);
        }

        publisher.set(value);
        logger.append(value);
    }

    public void log(String key, double[] value) {
        DoubleArrayPublisher publisher = (DoubleArrayPublisher) publisherMap.get(key);
        DoubleArrayLogEntry logger = (DoubleArrayLogEntry) loggerMap.get(key);

        if (publisher == null || logger == null) {
            publisher =
                    NetworkTableInstance.getDefault().getDoubleArrayTopic(key).publish();
            logger = new DoubleArrayLogEntry(log, key);

            publisherMap.put(key, publisher);
            loggerMap.put(key, logger);
        }

        publisher.set(value);
        logger.append(value);
    }

    public void log(String key, long value) {
        IntegerPublisher publisher = (IntegerPublisher) publisherMap.get(key);
        IntegerLogEntry logger = (IntegerLogEntry) loggerMap.get(key);

        if (publisher == null || logger == null) {
            publisher = NetworkTableInstance.getDefault().getIntegerTopic(key).publish();
            logger = new IntegerLogEntry(log, key);

            publisherMap.put(key, publisher);
            loggerMap.put(key, logger);
        }

        publisher.set(value);
        logger.append(value);
    }

    public void log(String key, long[] value) {
        IntegerArrayPublisher publisher = (IntegerArrayPublisher) publisherMap.get(key);
        IntegerArrayLogEntry logger = (IntegerArrayLogEntry) loggerMap.get(key);

        if (publisher == null || logger == null) {
            publisher =
                    NetworkTableInstance.getDefault().getIntegerArrayTopic(key).publish();
            logger = new IntegerArrayLogEntry(log, key);

            publisherMap.put(key, publisher);
            loggerMap.put(key, logger);
        }

        publisher.set(value);
        logger.append(value);
    }

    public void log(String key, String value) {
        StringPublisher publisher = (StringPublisher) publisherMap.get(key);
        StringLogEntry logger = (StringLogEntry) loggerMap.get(key);

        if (publisher == null || logger == null) {
            publisher = NetworkTableInstance.getDefault().getStringTopic(key).publish();
            logger = new StringLogEntry(log, key);

            publisherMap.put(key, publisher);
            loggerMap.put(key, logger);
        }

        publisher.set(value);
        logger.append(value);
    }

    public void log(String key, String[] value) {
        StringArrayPublisher publisher = (StringArrayPublisher) publisherMap.get(key);
        StringArrayLogEntry logger = (StringArrayLogEntry) loggerMap.get(key);

        if (publisher == null || logger == null) {
            publisher =
                    NetworkTableInstance.getDefault().getStringArrayTopic(key).publish();
            logger = new StringArrayLogEntry(log, key);

            publisherMap.put(key, publisher);
            loggerMap.put(key, logger);
        }

        publisher.set(value);
        logger.append(value);
    }

    public void log(String key, ChassisSpeeds value) {
        log(key, new double[] {value.vxMetersPerSecond, value.vyMetersPerSecond, value.omegaRadiansPerSecond});
    }

    public void log(String key, Pose2d value) {
        log(key, new double[] {value.getX(), value.getY(), value.getRotation().getRadians()});
    }

    public void log(String key, Pose3d value, boolean logQuaternion) {
        if (logQuaternion) {
            var rotation = value.getRotation().getQuaternion();

            log(key, new double[] {
                value.getX(),
                value.getY(),
                value.getZ(),
                rotation.getW(),
                rotation.getX(),
                rotation.getY(),
                rotation.getZ()
            });
        } else {
            var rotation = value.getRotation();

            log(key, new double[] {
                value.getX(),
                value.getY(),
                value.getZ(),
                rotation.getX(),
                rotation.getY(),
                rotation.getZ()
            });
        }
    }

    /* Tunables - Log the value once and returns an subscriber */

    public BooleanSubscriber tunable(String key, boolean value) {
        log(key, value);

        return NetworkTableInstance.getDefault().getBooleanTopic(key).subscribe(value);
    }

    public BooleanArraySubscriber tunable(String key, boolean[] value) {
        log(key, value);

        return NetworkTableInstance.getDefault().getBooleanArrayTopic(key).subscribe(value);
    }

    public DoubleSubscriber tunable(String key, double value) {
        log(key, value);

        return NetworkTableInstance.getDefault().getDoubleTopic(key).subscribe(value);
    }

    public DoubleArraySubscriber tunable(String key, double[] value) {
        log(key, value);

        return NetworkTableInstance.getDefault().getDoubleArrayTopic(key).subscribe(value);
    }

    public IntegerSubscriber tunable(String key, long value) {
        log(key, value);

        return NetworkTableInstance.getDefault().getIntegerTopic(key).subscribe(value);
    }

    public IntegerArraySubscriber tunable(String key, long[] value) {
        log(key, value);

        return NetworkTableInstance.getDefault().getIntegerArrayTopic(key).subscribe(value);
    }

    public StringSubscriber tunable(String key, String value) {
        log(key, value);

        return NetworkTableInstance.getDefault().getStringTopic(key).subscribe(value);
    }

    public StringArraySubscriber tunable(String key, String[] value) {
        log(key, value);

        return NetworkTableInstance.getDefault().getStringArrayTopic(key).subscribe(value);
    }
}
