package frc.lib.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveDriveSignal extends ChassisSpeeds {
    private boolean isFieldOriented;
    private boolean isOpenLoop;

    private boolean isLocked = false;

    public SwerveDriveSignal(ChassisSpeeds velocity, boolean isFieldOriented, boolean isOpenLoop) {
        super(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond, velocity.omegaRadiansPerSecond);

        this.isFieldOriented = isFieldOriented;
        this.isOpenLoop = isOpenLoop;
    }

    public SwerveDriveSignal(ChassisSpeeds velocity, boolean isFieldOriented) {
        this(velocity, isFieldOriented, true);
    }

    public SwerveDriveSignal(boolean locked) {
        this(new ChassisSpeeds(), false, false);

        this.isLocked = true;
    }

    public SwerveDriveSignal() {
        super();

        this.isFieldOriented = false;
        this.isOpenLoop = true;
    }

    public boolean isFieldOriented() {
        return this.isFieldOriented;
    }

    public boolean isOpenLoop() {
        return this.isOpenLoop;
    }

    public boolean isLocked() {
        return isLocked;
    }
}
