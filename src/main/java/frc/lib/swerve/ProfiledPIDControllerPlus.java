package frc.lib.swerve;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ProfiledPIDControllerPlus extends ProfiledPIDController {
    /**
     * Allocates a ProfiledPIDController with the given constants for Kp, Ki, and Kd.
     *
     * @param Kp The proportional coefficient.
     * @param Ki The integral coefficient.
     * @param Kd The derivative coefficient.
     * @param constraints Velocity and acceleration constraints for goal.
     */
    public ProfiledPIDControllerPlus(double Kp, double Ki, double Kd, TrapezoidProfile.Constraints constraints) {
        super(Kp, Ki, Kd, constraints, 0.02);
    }

    /**
     * Allocates a ProfiledPIDController with the given constants for Kp, Ki, and Kd.
     *
     * @param Kp The proportional coefficient.
     * @param Ki The integral coefficient.
     * @param Kd The derivative coefficient.
     * @param constraints Velocity and acceleration constraints for goal.
     * @param period The period between controller updates in seconds. The default is 0.02 seconds.
     */
    public ProfiledPIDControllerPlus(
            double Kp, double Ki, double Kd, TrapezoidProfile.Constraints constraints, double period) {
        super(Kp, Ki, Kd, constraints, period);
    }

    @Override
    public boolean atGoal() {
        return getGoal().equals(getSetpoint())
                && Math.abs(getPositionError()) < getPositionTolerance()
                && Math.abs(getVelocityError()) < getVelocityTolerance();
    }
}
