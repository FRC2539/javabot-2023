package frc.lib.interpolation;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.ArrayList;

public class MovingAverageVelocity {
    ArrayList<ChassisSpeeds> velocities = new ArrayList<ChassisSpeeds>();
    private int maxSize;

    public MovingAverageVelocity(int maxSize) {
        this.maxSize = maxSize;
    }

    public void add(ChassisSpeeds velocity) {
        velocities.add(velocity);

        if (velocities.size() > maxSize) {
            velocities.remove(0);
        }
    }

    public ChassisSpeeds getAverage() {
        double x = 0.0, y = 0.0, t = 0.0;

        for (ChassisSpeeds velocity : velocities) {
            x += velocity.vxMetersPerSecond;
            y += velocity.vyMetersPerSecond;
            t += velocity.omegaRadiansPerSecond;
        }

        double size = getSize();

        return new ChassisSpeeds(x / size, y / size, t / size);
    }

    public int getSize() {
        return velocities.size();
    }

    public boolean isUnderMaxSize() {
        return getSize() < maxSize;
    }

    public void clear() {
        velocities.clear();
    }
}
