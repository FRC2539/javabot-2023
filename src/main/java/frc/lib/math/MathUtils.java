package frc.lib.math;

public class MathUtils {
    public static boolean equalsWithinError(double targetValue, double currentValue, double error) {
        return Math.abs(currentValue - targetValue) <= error;
    }

    public static double ensureRange(double value, double minValue, double maxValue) {
        if (minValue > value) {
            return minValue;
        } else if (maxValue < value) {
            return maxValue;
        } else {
            return value;
        }
    }

    public static boolean isInRange(double value, double minValue, double maxValue) {
        if (minValue > value) {
            return false;
        } else if (maxValue < value) {
            return false;
        } else {
            return true;
        }
    }
}
