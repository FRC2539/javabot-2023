package frc.lib.interpolation;

import edu.wpi.first.math.interpolation.Interpolatable;
import java.util.Optional;
import java.util.TreeMap;

public class InterpolatingMap<T extends Interpolatable<T>> extends TreeMap<Double, T> {
    public InterpolatingMap() {}

    public T put(double t, T interpolatable) {
        super.put(t, interpolatable);

        return interpolatable;
    }

    public Optional<T> getInterpolated(double t) {
        T interpolatable = get(t);

        if (interpolatable != null) return Optional.of(interpolatable);

        Double floor = floorKey(t);
        Double ceiling = ceilingKey(t);

        // Account for cases where the distance is outside the given range
        if (floor == null && ceiling == null) {
            return Optional.empty();
        } else if (floor == null) {
            return Optional.of(get(ceiling));
        } else if (ceiling == null) {
            return Optional.of(get(floor));
        }

        T floorState = get(floor);
        T ceilingState = get(ceiling);

        double t_ = (t - floor) / (ceiling - floor);

        return Optional.of(floorState.interpolate(ceilingState, t_));
    }
}
