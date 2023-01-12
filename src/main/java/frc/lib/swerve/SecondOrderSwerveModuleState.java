package frc.lib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SecondOrderSwerveModuleState extends SwerveModuleState {
    // private double accelerationMetersPerSecondSquared; //some day my sweet summer child
    public double angularVelocityRadiansPerSecond;

    private SecondOrderSwerveModuleState(
            double speedMetersPerSecond,
            double accelerationMetersPerSecondSquared,
            Rotation2d angle,
            double angularVelocityMetersPerSecond) {
        super(speedMetersPerSecond, angle);
        // this.accelerationMetersPerSecondSquared = accelerationMetersPerSecondSquared;
        this.angularVelocityRadiansPerSecond = angularVelocityMetersPerSecond;
    }

    public SecondOrderSwerveModuleState(
            double speedMetersPerSecond, Rotation2d angle, double angularVelocityMetersPerSecond) {
        this(speedMetersPerSecond, 0, angle, angularVelocityMetersPerSecond);
    }

    public SecondOrderSwerveModuleState() {
        this(0, new Rotation2d(), 0);
    }

    /*public static SecondOrderSwerveModuleState optimize(
            SecondOrderSwerveModuleState desiredState, Rotation2d currentAngle) {
        Rotation2d angle = desiredState.angle;
        SwerveModuleState outputState = SwerveModuleState.optimize(desiredState, currentAngle);
        double sign = angle.equals(outputState.angle) ? 1 : -1;
        return new SecondOrderSwerveModuleState(
                outputState.speedMetersPerSecond,
                desiredState.accelerationMetersPerSecondSquared * sign,
                outputState.angle,
                desiredState.angularVelocityRadiansPerSecond);
    }*/
}
