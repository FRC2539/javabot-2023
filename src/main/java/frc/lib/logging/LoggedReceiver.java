package frc.lib.logging;

import edu.wpi.first.networktables.*;
import frc.lib.logging.LogValue.LoggableType;

public class LoggedReceiver {
    private LoggableType type;
    private String key;

    private Subscriber subscriber;

    public LoggedReceiver(LoggableType type, String key) {
        this.type = type;
        this.key = key;

        switch (type) {
            case Raw:
                subscriber =
                        NetworkTableInstance.getDefault().getRawTopic(key).subscribe(type.getNT4Type(), new byte[0]);
                break;
            case Boolean:
                subscriber =
                        NetworkTableInstance.getDefault().getBooleanTopic(key).subscribe(false);
                break;
            case Integer:
                subscriber =
                        NetworkTableInstance.getDefault().getIntegerTopic(key).subscribe(0);
                break;
            case Float:
                subscriber =
                        NetworkTableInstance.getDefault().getFloatTopic(key).subscribe(0);
                break;
            case Double:
                subscriber =
                        NetworkTableInstance.getDefault().getDoubleTopic(key).subscribe(0);
                break;
            case String:
                subscriber =
                        NetworkTableInstance.getDefault().getStringTopic(key).subscribe("");
                break;
            case BooleanArray:
                subscriber = NetworkTableInstance.getDefault()
                        .getBooleanArrayTopic(key)
                        .subscribe(new boolean[0]);
                break;
            case IntegerArray:
                subscriber = NetworkTableInstance.getDefault()
                        .getIntegerArrayTopic(key)
                        .subscribe(new long[0]);
                break;
            case FloatArray:
                subscriber = NetworkTableInstance.getDefault()
                        .getFloatArrayTopic(key)
                        .subscribe(new float[0]);
                break;
            case DoubleArray:
                subscriber = NetworkTableInstance.getDefault()
                        .getDoubleArrayTopic(key)
                        .subscribe(new double[0]);
                break;
            case StringArray:
                subscriber = NetworkTableInstance.getDefault()
                        .getStringArrayTopic(key)
                        .subscribe(new String[0]);
                break;
        }
    }

    private LogValue getLogValue() {
        LogValue value = new LogValue(0.0); // Default value will never occur

        switch (type) {
            case Raw:
                value = new LogValue(((RawSubscriber) subscriber).get());
                break;
            case Boolean:
                value = new LogValue(((BooleanSubscriber) subscriber).get());
                break;
            case Integer:
                value = new LogValue(((IntegerSubscriber) subscriber).get());
                break;
            case Float:
                value = new LogValue(((FloatSubscriber) subscriber).get());
                break;
            case Double:
                value = new LogValue(((DoubleSubscriber) subscriber).get());
                break;
            case String:
                value = new LogValue(((StringSubscriber) subscriber).get());
                break;
            case BooleanArray:
                value = new LogValue(((BooleanArraySubscriber) subscriber).get());
                break;
            case IntegerArray:
                value = new LogValue(((IntegerArraySubscriber) subscriber).get());
                break;
            case FloatArray:
                value = new LogValue(((FloatArraySubscriber) subscriber).get());
                break;
            case DoubleArray:
                value = new LogValue(((DoubleArraySubscriber) subscriber).get());
                break;
            case StringArray:
                value = new LogValue(((StringArraySubscriber) subscriber).get());
                break;
        }

        if (subscriber.exists()) Logger.logOnly(key, value);

        return value;
    }

    public byte[] getRaw() {
        return getLogValue().getRaw();
    }

    public boolean getBoolean() {
        return getLogValue().getBoolean();
    }

    public long getInteger() {
        return getLogValue().getInteger();
    }

    public float getFloat() {
        return getLogValue().getFloat();
    }

    public double getDouble() {
        return getLogValue().getDouble();
    }

    public String getString() {
        return getLogValue().getString();
    }

    public boolean[] getBooleanArray() {
        return getLogValue().getBooleanArray();
    }

    public long[] getIntegerArray() {
        return getLogValue().getIntegerArray();
    }

    public float[] getFloatArray() {
        return getLogValue().getFloatArray();
    }

    public double[] getDoubleArray() {
        return getLogValue().getDoubleArray();
    }

    public String[] getStringArray() {
        return getLogValue().getStringArray();
    }
}
