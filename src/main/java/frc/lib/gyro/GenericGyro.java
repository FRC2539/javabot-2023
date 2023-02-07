package frc.lib.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public interface GenericGyro {
    Rotation2d getRotation2d();

    Rotation3d getRotation3d();

    Rotation3d getRotationRates3d();
}
