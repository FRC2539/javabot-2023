package frc.lib.swerve;

import java.util.Optional;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int cancoderID;
    public final double angleOffset;
    public Optional<String> canivoreName = Optional.empty();

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID
     * @param angleMotorID
     * @param canCoderID
     * @param angleOffset
     */
    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int canCoderID, double angleOffset) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.cancoderID = canCoderID;
        this.angleOffset = angleOffset;
    }

    public SwerveModuleConstants(
            int driveMotorID, int angleMotorID, int canCoderID, double angleOffset, String canivoreName) {
        this(driveMotorID, angleMotorID, canCoderID, angleOffset);
        this.canivoreName = Optional.of(canivoreName);
    }
}
