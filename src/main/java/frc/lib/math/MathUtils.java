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

    public static double accomidateOverflow(double oldRotations, double newRotations, double period) {
        //weird ((x % b) + b) % b to get modulo instead of remainder
        double result = (((newRotations - oldRotations) % period) + period) % period;
        return ((result > period/2) ? result - period : result) + oldRotations;
    }
}
