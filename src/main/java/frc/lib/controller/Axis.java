package frc.lib.controller;

import edu.wpi.first.math.MathUtil;
import java.util.function.DoubleSupplier;

public abstract class Axis implements DoubleSupplier {
    public static final double DEADBAND = 0.05;

    private boolean inverted = false;
    private double scale = 1.0;

    public boolean isInverted() {
        return inverted;
    }

    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }

    public double getScale() {
        return scale;
    }

    public void setScale(double scale) {
        this.scale = scale;
    }

    public abstract double getRaw();

    public double get() {
        return get(false, false);
    }

    public double get(boolean squared) {
        return get(squared, false);
    }

    @Override
    public double getAsDouble() {
        return get(true);
    }

    public double get(boolean squared, boolean ignoreScale) {
        double value = getRaw();

        // Invert if axis is inverted
        if (inverted) {
            value = -value;
        }

        // Deadband value
        value = MathUtil.applyDeadband(value, DEADBAND);

        // Square value
        if (squared) {
            value = Math.copySign(value * value, value);
        }

        // Scale value
        if (!ignoreScale) {
            value *= scale;
        }

        return value;
    }
}
