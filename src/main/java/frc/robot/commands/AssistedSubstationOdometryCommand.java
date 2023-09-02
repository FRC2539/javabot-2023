package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

import java.util.function.DoubleSupplier;

public class AssistedSubstationOdometryCommand extends CommandBase {
    private static final TrapezoidProfile.Constraints driveConstraints = new TrapezoidProfile.Constraints(2, 3);
    private static final TrapezoidProfile.Constraints omegaConstraints = new TrapezoidProfile.Constraints(6, 7);

    private final SwerveDriveSubsystem swerveDriveSubsystem;

    private DoubleSupplier strafe;

    private final ProfiledPIDController driveController = new ProfiledPIDController(5.5, 0, 0, driveConstraints);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(6, 0, 0, omegaConstraints);

    private final SlewRateLimiter xSlewRater = new SlewRateLimiter(3);

    private double desiredXPosition;

    public static double offsetToStation;

    /**
     * Drives directly to the given pose on the field automatically.
     *
     * Should automatically accomidate for starting with a speed.
     *
     * @param swerveDriveSubsystem
     * @param targetPoseSupplier
     */
    public AssistedSubstationOdometryCommand(SwerveDriveSubsystem swerveDriveSubsystem, DoubleSupplier forward,
    DoubleSupplier strafe,
    DoubleSupplier rotate, double offsetToStation) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;

        this.strafe = strafe;

        this.offsetToStation = offsetToStation;

        driveController.setTolerance(0.02);
        omegaController.setTolerance(Units.degreesToRadians(1));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void initialize() {
        var robotPose = swerveDriveSubsystem.getPose();
        var robotVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(
                swerveDriveSubsystem.getDesiredVelocity(), swerveDriveSubsystem.getRotation());

        desiredXPosition = swerveDriveSubsystem.getLastMinimumXValue() + offsetToStation;

        xSlewRater.reset(robotVelocity.vxMetersPerSecond);

        driveController.reset(robotPose.getX());

        driveController.setGoal(desiredXPosition);

        omegaController.reset(robotPose.getRotation().getRadians(), robotVelocity.omegaRadiansPerSecond);

        omegaController.setGoal(Math.PI);
    }

    @Override
    public void execute() {
        var robotPose = swerveDriveSubsystem.getPose();

        var driveSpeed = driveController.calculate(robotPose.getX()) + driveController.getSetpoint().velocity;
        var omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians())
                + omegaController.getSetpoint().velocity;

        if (driveController.atGoal()) driveSpeed = 0;
        if (omegaController.atGoal()) omegaSpeed = 0;

        swerveDriveSubsystem.setVelocity(
                new ChassisSpeeds(
                        driveSpeed,
                        strafe.getAsDouble(),
                        omegaSpeed),
                true,
                true);
    }

    @Override
    public boolean isFinished() {
        return driveController.atGoal() && omegaController.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        swerveDriveSubsystem.stop();
    }
}
