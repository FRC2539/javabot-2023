package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;
import java.util.function.Supplier;

public class DriveToPositionCommand extends CommandBase {
    private static final TrapezoidProfile.Constraints driveConstraints = new TrapezoidProfile.Constraints(2, 3);
    private static final TrapezoidProfile.Constraints omegaConstraints = new TrapezoidProfile.Constraints(6, 7);

    private final SwerveDriveSubsystem swerveDriveSubsystem;

    private Supplier<Pose2d> targetPoseSupplier;

    private final ProfiledPIDController driveController = new ProfiledPIDController(5.5, 0, 0, driveConstraints);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(6, 0, 0, omegaConstraints);

    private final SlewRateLimiter xSlewRater = new SlewRateLimiter(3);
    private final SlewRateLimiter ySlewRater = new SlewRateLimiter(3);

    /**
     * Drives directly to the given pose on the field automatically.
     *
     * Should automatically accomidate for starting with a speed.
     *
     * @param swerveDriveSubsystem
     * @param targetPoseSupplier
     */
    public DriveToPositionCommand(SwerveDriveSubsystem swerveDriveSubsystem, Supplier<Pose2d> targetPoseSupplier) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.targetPoseSupplier = targetPoseSupplier;

        driveController.setTolerance(0.02);
        omegaController.setTolerance(Units.degreesToRadians(1));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerveDriveSubsystem);
    }

    /**
     * Drives to the given pose on the field automatically)
     *
     * @param swerveDriveSubsystem
     * @param targetPose
     */
    public DriveToPositionCommand(SwerveDriveSubsystem swerveDriveSubsystem, Pose2d targetPose) {
        this(swerveDriveSubsystem, () -> targetPose);
    }

    @Override
    public void initialize() {
        var robotPose = swerveDriveSubsystem.getPose();
        var robotVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(
                swerveDriveSubsystem.getDesiredVelocity(), swerveDriveSubsystem.getRotation());

        var targetPose = targetPoseSupplier.get();

        var desiredPoseTranslation = getGoalTranslation(robotPose, targetPose);

        xSlewRater.reset(robotVelocity.vxMetersPerSecond);
        ySlewRater.reset(robotVelocity.vyMetersPerSecond);

        driveController.reset(
                desiredPoseTranslation.getNorm(),
                (robotVelocity.vxMetersPerSecond * desiredPoseTranslation.getX()
                                + robotVelocity.vyMetersPerSecond * desiredPoseTranslation.getY())
                        / desiredPoseTranslation.getNorm());

        driveController.setGoal(0);

        omegaController.reset(robotPose.getRotation().getRadians(), robotVelocity.omegaRadiansPerSecond);
    }

    @Override
    public void execute() {
        var robotPose = swerveDriveSubsystem.getPose();

        var targetPose = targetPoseSupplier.get();

        var goalTranslation = getGoalTranslation(robotPose, targetPose);

        // Update controllers
        omegaController.setGoal(targetPose.getRotation().getRadians());

        var driveSpeed = driveController.calculate(goalTranslation.getNorm()) + driveController.getSetpoint().velocity;
        var omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians())
                + omegaController.getSetpoint().velocity;

        if (driveController.atGoal()) driveSpeed = 0;
        if (omegaController.atGoal()) omegaSpeed = 0;

        swerveDriveSubsystem.setVelocity(
                new ChassisSpeeds(
                        driveSpeed * goalTranslation.getX() / goalTranslation.getNorm(),
                        driveSpeed * goalTranslation.getY() / goalTranslation.getNorm(),
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

    public Translation2d getGoalTranslation(Pose2d robotPose, Pose2d targetPose) {
        return targetPose.getTranslation().minus(robotPose.getTranslation());
    }
}
