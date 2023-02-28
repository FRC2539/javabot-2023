package frc.lib.gyro;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class PigeonGyro implements GenericGyro {
    private Pigeon2 pigeon;

    public PigeonGyro(int port, String canbus) {
        pigeon = new Pigeon2(port, canbus);
    }

    public PigeonGyro(int port) {
        pigeon = new Pigeon2(port);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(pigeon.getYaw());
    }

    public Rotation3d getRotation3d() {
        // We add 2 degrees to the pitch because the Pigeon is offset by about -2 degrees pitchwise
        // return new Rotation3d(
        //         Units.degreesToRadians(pigeon.getRoll()), Units.degreesToRadians(pigeon.getPitch() + 2), 0);
        return new Rotation3d(
                Units.degreesToRadians(pigeon.getRoll()) + 0.019,
                Units.degreesToRadians(pigeon.getPitch()) - 0.082,
                0); // - 0.035
    }

    public Rotation3d getRotationRates3d() {
        double[] rawXYZ = new double[3];

        pigeon.getRawGyro(rawXYZ);

        return new Rotation3d(rawXYZ[0], rawXYZ[1], rawXYZ[2]);
    }
}
