// Original class from AdvantageKit

package frc.lib.logging;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;

public class LogValue {
    public final LoggableType type;
    private final Object value;

    public boolean sendToNT = true;

    LogValue(byte[] value) {
        type = LoggableType.Raw;
        this.value = value;
    }

    LogValue(boolean value) {
        type = LoggableType.Boolean;
        this.value = value;
    }

    LogValue(long value) {
        type = LoggableType.Integer;
        this.value = value;
    }

    LogValue(float value) {
        type = LoggableType.Float;
        this.value = value;
    }

    LogValue(double value) {
        type = LoggableType.Double;
        this.value = value;
    }

    LogValue(String value) {
        type = LoggableType.String;
        if (value != null) {
            this.value = value;
        } else {
            this.value = "";
        }
    }

    LogValue(boolean[] value) {
        type = LoggableType.BooleanArray;
        this.value = value;
    }

    LogValue(long[] value) {
        type = LoggableType.IntegerArray;
        this.value = value;
    }

    LogValue(float[] value) {
        type = LoggableType.FloatArray;
        this.value = value;
    }

    LogValue(double[] value) {
        type = LoggableType.DoubleArray;
        this.value = value;
    }

    LogValue(String[] value) {
        type = LoggableType.StringArray;
        this.value = value;
    }

    public byte[] getRaw() {
        return getRaw(new byte[] {});
    }

    public boolean getBoolean() {
        return getBoolean(false);
    }

    public long getInteger() {
        return getInteger(0);
    }

    public float getFloat() {
        return getFloat(0.0f);
    }

    public double getDouble() {
        return getDouble(0.0);
    }

    public String getString() {
        return getString("");
    }

    public boolean[] getBooleanArray() {
        return getBooleanArray(new boolean[] {});
    }

    public long[] getIntegerArray() {
        return getIntegerArray(new long[] {});
    }

    public float[] getFloatArray() {
        return getFloatArray(new float[] {});
    }

    public double[] getDoubleArray() {
        return getDoubleArray(new double[] {});
    }

    public String[] getStringArray() {
        return getStringArray(new String[] {});
    }

    public byte[] getRaw(byte[] defaultValue) {
        return type == LoggableType.Raw ? (byte[]) value : defaultValue;
    }

    public boolean getBoolean(boolean defaultValue) {
        return type == LoggableType.Boolean ? (boolean) value : defaultValue;
    }

    public long getInteger(long defaultValue) {
        return type == LoggableType.Integer ? (long) value : defaultValue;
    }

    public float getFloat(float defaultValue) {
        return type == LoggableType.Float ? (float) value : defaultValue;
    }

    public double getDouble(double defaultValue) {
        return type == LoggableType.Double ? (double) value : defaultValue;
    }

    public String getString(String defaultValue) {
        return type == LoggableType.String ? (String) value : defaultValue;
    }

    public boolean[] getBooleanArray(boolean[] defaultValue) {
        return type == LoggableType.BooleanArray ? (boolean[]) value : defaultValue;
    }

    public long[] getIntegerArray(long[] defaultValue) {
        return type == LoggableType.IntegerArray ? (long[]) value : defaultValue;
    }

    public float[] getFloatArray(float[] defaultValue) {
        return type == LoggableType.FloatArray ? (float[]) value : defaultValue;
    }

    public double[] getDoubleArray(double[] defaultValue) {
        return type == LoggableType.DoubleArray ? (double[]) value : defaultValue;
    }

    public String[] getStringArray(String[] defaultValue) {
        return type == LoggableType.StringArray ? (String[]) value : defaultValue;
    }

    public LogValue withoutNT() {
        sendToNT = false;
        return this;
    }

    @Override
    public boolean equals(Object other) {
        if (other instanceof LogValue) {
            LogValue otherValue = (LogValue) other;
            if (otherValue.type.equals(type)) {
                switch (type) {
                    case Raw:
                        return Arrays.equals(getRaw(), otherValue.getRaw());
                    case Boolean:
                    case Integer:
                    case Float:
                    case Double:
                    case String:
                        return value.equals(otherValue.value);
                    case BooleanArray:
                        return Arrays.equals(getBooleanArray(), otherValue.getBooleanArray());
                    case IntegerArray:
                        return Arrays.equals(getIntegerArray(), otherValue.getIntegerArray());
                    case FloatArray:
                        return Arrays.equals(getFloatArray(), otherValue.getFloatArray());
                    case DoubleArray:
                        return Arrays.equals(getDoubleArray(), otherValue.getDoubleArray());
                    case StringArray:
                        return Arrays.equals(getStringArray(), otherValue.getStringArray());
                }
            }
        }
        return false;
    }

    @Override
    public int hashCode() {
        return Objects.hash(type, value);
    }

    /**
     * Represents all possible data types that can be logged.
     */
    public enum LoggableType {
        Raw,
        Boolean,
        Integer,
        Float,
        Double,
        String,
        BooleanArray,
        IntegerArray,
        FloatArray,
        DoubleArray,
        StringArray;

        // https://github.com/wpilibsuite/allwpilib/blob/main/wpiutil/doc/datalog.adoc#data-types
        private static final List<String> wpilogTypes = List.of(
                "raw",
                "boolean",
                "int64",
                "float",
                "double",
                "string",
                "boolean[]",
                "int64[]",
                "float[]",
                "double[]",
                "string[]");

        // https://github.com/wpilibsuite/allwpilib/blob/main/ntcore/doc/networktables4.adoc#supported-data-types
        private static final List<String> nt4Types = List.of(
                "raw",
                "boolean",
                "int",
                "float",
                "double",
                "string",
                "boolean[]",
                "int[]",
                "float[]",
                "double[]",
                "string[]");

        /**
         * Returns the standard string type for WPILOGs.
         */
        public String getWPILOGType() {
            return wpilogTypes.get(this.ordinal());
        }

        /**
         * Returns the standard string type for NT4.
         */
        public String getNT4Type() {
            return nt4Types.get(this.ordinal());
        }

        /**
         * Returns the type based on a standard string type for WPILOGs.
         */
        public static LoggableType fromWPILOGType(String type) {
            if (wpilogTypes.contains(type)) {
                return LoggableType.values()[wpilogTypes.indexOf(type)];
            } else {
                return null;
            }
        }

        /**
         * Returns the type based on a standard string type for NT4.
         */
        public static LoggableType fromNT4Type(String type) {
            if (nt4Types.contains(type)) {
                return LoggableType.values()[nt4Types.indexOf(type)];
            } else {
                return null;
            }
        }
    }
}
