package frc.lib.logging;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.logging.LogValue.LoggableType;
import frc.lib.logging.LoggingThread.Writer;
import frc.robot.Constants;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;

public class Logger {
    private static int queueCapacity = 50 * 4; // ~ 4 seconds

    private static Map<String, LogValue> updatesMap = new HashMap<>();
    private static BlockingQueue<LogTable> updatesQueue = new ArrayBlockingQueue<>(queueCapacity);

    private static final List<Writer> writers = Arrays.asList(new DataLogWriter(), new NTWriter());

    private static LoggingThread loggingThread = new LoggingThread(updatesQueue, writers);

    // In competition mode, only log necessary fields to reduce bandwidth usage
    private static boolean ntDebuggingEnabled = !Constants.competitionMode;

    private static StringPublisher messagesPublisher = NetworkTableInstance.getDefault()
            .getTable("Messages")
            .getStringTopic("messages")
            .publish();

    static {
        // Start the logging thread
        loggingThread.start();
    }

    public static void message(String message) {
        messagesPublisher.set(message);
    }

    public static void update() {
        try {
            // Send the current updates to the updating thread
            updatesQueue.add(new LogTable(updatesMap, HALUtil.getFPGATime()));

            // Reset the update table
            updatesMap = new HashMap<>();
        } catch (IllegalStateException exception) {
            DriverStation.reportError("Logging queue capacity exceeded, data is no longer being logged.", false);
        }
    }

    /* Logger Methods - Log to DataLog and NetworkTables, no getting */

    @FunctionalInterface
    public interface AlwaysNT {
        void alwaysNT();
    }

    private static AlwaysNT writeUpdate(String key, LogValue value) {
        updatesMap.put(key, ntDebuggingEnabled ? value : value.withoutNT());

        return () -> writeUpdateAlways(key, value);
    }

    private static void writeUpdateAlways(String key, LogValue value) {
        updatesMap.put(key, value.withNT());
    }

    public static void logOnly(String key, LogValue value) {
        updatesMap.put(key, value.withoutNT());
    }

    public static AlwaysNT log(String key, boolean value) {
        return writeUpdate(key, new LogValue(value));
    }

    public static AlwaysNT log(String key, boolean[] value) {
        return writeUpdate(key, new LogValue(value));
    }

    public static AlwaysNT log(String key, double value) {
        return writeUpdate(key, new LogValue(value));
    }

    public static AlwaysNT log(String key, double[] value) {
        return writeUpdate(key, new LogValue(value));
    }

    public static AlwaysNT log(String key, long value) {
        return writeUpdate(key, new LogValue(value));
    }

    public static AlwaysNT log(String key, long[] value) {
        return writeUpdate(key, new LogValue(value));
    }

    public static AlwaysNT log(String key, String value) {
        return writeUpdate(key, new LogValue(value));
    }

    public static AlwaysNT log(String key, String[] value) {
        return writeUpdate(key, new LogValue(value));
    }

    public static AlwaysNT log(String key, Translation2d value) {
        return log(key, new double[] {value.getX(), value.getY()});
    }

    public static AlwaysNT log(String key, ChassisSpeeds value) {
        return log(key, new double[] {value.vxMetersPerSecond, value.vyMetersPerSecond, value.omegaRadiansPerSecond});
    }

    public static AlwaysNT log(String key, Pose2d value) {
        return log(
                key,
                new double[] {value.getX(), value.getY(), value.getRotation().getRadians()});
    }

    public static AlwaysNT log(String key, Pose3d value, boolean logQuaternion) {
        if (logQuaternion) {
            var rotation = value.getRotation().getQuaternion();

            return log(key, new double[] {
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

            return log(key, new double[] {
                value.getX(), value.getY(), value.getZ(), rotation.getX(), rotation.getY(), rotation.getZ()
            });
        }
    }

    /* Tunables - Log the value once and returns an subscriber */

    public static LoggedReceiver tunable(String key, boolean value) {
        writeUpdateAlways(key, new LogValue(value));

        return receive(key, value);
    }

    public static LoggedReceiver tunable(String key, boolean[] value) {
        writeUpdateAlways(key, new LogValue(value));

        return receive(key, value);
    }

    public static LoggedReceiver tunable(String key, double value) {
        writeUpdateAlways(key, new LogValue(value));

        return receive(key, value);
    }

    public static LoggedReceiver tunable(String key, double[] value) {
        writeUpdateAlways(key, new LogValue(value));

        return receive(key, value);
    }

    public static LoggedReceiver tunable(String key, long value) {
        writeUpdateAlways(key, new LogValue(value));

        return receive(key, value);
    }

    public static LoggedReceiver tunable(String key, long[] value) {
        writeUpdateAlways(key, new LogValue(value));

        return receive(key, value);
    }

    public static LoggedReceiver tunable(String key, String value) {
        writeUpdateAlways(key, new LogValue(value));

        return receive(key, value);
    }

    public static LoggedReceiver tunable(String key, String[] value) {
        writeUpdateAlways(key, new LogValue(value));

        return receive(key, value);
    }

    /* Receivers - Get and log values from NetworkTables */

    public static LoggedReceiver receive(String key, boolean value) {
        return new LoggedReceiver(LoggableType.Boolean, key);
    }

    public static LoggedReceiver receive(String key, boolean[] value) {
        return new LoggedReceiver(LoggableType.BooleanArray, key);
    }

    public static LoggedReceiver receive(String key, double value) {
        return new LoggedReceiver(LoggableType.Double, key);
    }

    public static LoggedReceiver receive(String key, double[] value) {
        return new LoggedReceiver(LoggableType.DoubleArray, key);
    }

    public static LoggedReceiver receive(String key, long value) {
        return new LoggedReceiver(LoggableType.Integer, key);
    }

    public static LoggedReceiver receive(String key, long[] value) {
        return new LoggedReceiver(LoggableType.IntegerArray, key);
    }

    public static LoggedReceiver receive(String key, String value) {
        return new LoggedReceiver(LoggableType.String, key);
    }

    public static LoggedReceiver receive(String key, String[] value) {
        return new LoggedReceiver(LoggableType.StringArray, key);
    }

    public static record LogTable(Map<String, LogValue> updatesMap, long timestamp) {}
}
