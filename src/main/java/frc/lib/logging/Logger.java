package frc.lib.logging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;

public class Logger {
    private static Logger instance;

    private int queueCapacity = 50 * 5; // ~ 5 seconds 

    private Map<String, LogValue> updatesMap = new HashMap<>();

    private BlockingQueue<LogTable> updatesQueue = new ArrayBlockingQueue<>(queueCapacity);

    private LoggingThread loggingThread = new LoggingThread(updatesQueue);

    private StringPublisher messagesPublisher = NetworkTableInstance.getDefault()
            .getTable("Messages")
            .getStringTopic("messages")
            .publish();

    public Logger() {
        // Start the logging thread
        loggingThread.start();
    }

    public static Logger getInstance() {
        if (instance == null) instance = new Logger();

        return instance;
    }

    public void message(String message) {
        messagesPublisher.set(message);
    }

    public void update() {
        try {
            // Send the current updates to the updating thread
            updatesQueue.add(new LogTable(updatesMap, 0));

            // Reset the update table
            updatesMap = new HashMap<>();
        } catch (IllegalStateException exception) {
            DriverStation.reportError("Logging queue capacity exceeded, data is no longer being logged.", false);
        }
            
    }

    /* Logger Methods - Log to DataLog and NetworkTables, no getting */

    public void log(String key, boolean value) {
        updatesMap.put(key, new LogValue(value));
    }

    public void log(String key, boolean[] value) {
        updatesMap.put(key, new LogValue(value));
    }

    public void log(String key, double value) {
        updatesMap.put(key, new LogValue(value));
    }

    public void log(String key, double[] value) {
        updatesMap.put(key, new LogValue(value));
    }

    public void log(String key, long value) {
        updatesMap.put(key, new LogValue(value));
    }

    public void log(String key, long[] value) {
        updatesMap.put(key, new LogValue(value));
    }

    public void log(String key, String value) {
        updatesMap.put(key, new LogValue(value));
    }

    public void log(String key, String[] value) {
        updatesMap.put(key, new LogValue(value));
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

    public record LogTable (Map<String, LogValue> updatesMap, long timestamp) {}
}
