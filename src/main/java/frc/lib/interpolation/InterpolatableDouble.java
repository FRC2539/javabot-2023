package frc.lib.interpolation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;

public class InterpolatableDouble implements Interpolatable<InterpolatableDouble> {
    public double value;

    public InterpolatableDouble(double value) {
        this.value = value;
    }

    public InterpolatableDouble() {
        this(0);
    }

    public InterpolatableDouble interpolate(InterpolatableDouble otherState, double t) {
        // Linearly interpolate between the values
        return new InterpolatableDouble(MathUtil.interpolate(this.value, otherState.value, t));
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == null) {
            return false;
        }

        if (!(obj instanceof InterpolatableDouble)) {
            return false;
        }

        if (((InterpolatableDouble) obj).value != value) {
            return false;
        }

        return true;
    }
}
