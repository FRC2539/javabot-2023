package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.logging.LoggableDouble;
import frc.robot.subsystems.SwerveDriveSubsystem;
import java.util.function.Supplier;

public class DriveToPoseCommand extends CommandBase {
    private static final TrapezoidProfile.Constraints xConstraints = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints yConstraints = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints omegaConstraints = new TrapezoidProfile.Constraints(8, 8);

    private final SwerveDriveSubsystem swerveDriveSubsystem;

    private Supplier<Pose2d> targetPoseSupplier;

    private final ProfiledPIDController xController = new ProfiledPIDController(3.5, 0, 0, xConstraints);
    private final ProfiledPIDController yController = new ProfiledPIDController(3.5, 0, 0, yConstraints);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(4, 0, 0, omegaConstraints);

    private LoggableDouble xErrorLogger = new LoggableDouble("/Commands/xError");
    private LoggableDouble xSetpointLogger = new LoggableDouble("/Commands/xSetpoint");
    private LoggableDouble yErrorLogger = new LoggableDouble("/Commands/yError");
    private LoggableDouble ySetpointLogger = new LoggableDouble("/Commands/ySetpoint");
    private LoggableDouble omegaErrorLogger = new LoggableDouble("/Commands/omegaError");
    private LoggableDouble omegaSetpointLogger = new LoggableDouble("/Commands/omegaSetpoint");

    /**
     * Drives to the given pose on the field automatically.
     *
     * While the driving is generally smooth and fast, this algorithm currently assumes zero initial velocity.
     * It will behave erratically at the start if the robot is moving.
     *
     * @param swerveDriveSubsystem
     * @param targetPoseSupplier
     */
    public DriveToPoseCommand(SwerveDriveSubsystem swerveDriveSubsystem, Supplier<Pose2d> targetPoseSupplier) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.targetPoseSupplier = targetPoseSupplier;

        xController.setTolerance(0.2);
        yController.setTolerance(0.2);
        omegaController.setTolerance(Units.degreesToRadians(3));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void initialize() {
        var robotPose = swerveDriveSubsystem.getPose();
        var robotVelocity = swerveDriveSubsystem.getVelocity();

        omegaController.reset(robotPose.getRotation().getRadians(), -robotVelocity.omegaRadiansPerSecond);
        xController.reset(robotPose.getX(), -robotVelocity.vxMetersPerSecond);
        yController.reset(robotPose.getY(), -robotVelocity.vyMetersPerSecond);
    }

    @Override
    public void execute() {
        var robotPose = swerveDriveSubsystem.getPose();
        var targetPose = targetPoseSupplier.get();

        // Update controllers
        xController.setGoal(targetPose.getX());
        yController.setGoal(targetPose.getY());
        omegaController.setGoal(targetPose.getRotation().getRadians());

        xSetpointLogger.set(xController.getSetpoint().position);
        ySetpointLogger.set(yController.getSetpoint().position);
        omegaSetpointLogger.set(omegaController.getSetpoint().position);

        // Drive to the target
        var xSpeed = xController.calculate(robotPose.getX());
        var ySpeed = yController.calculate(robotPose.getY());
        var omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());

        xErrorLogger.set(robotPose.getX());
        yErrorLogger.set(robotPose.getY());
        omegaErrorLogger.set(robotPose.getRotation().getRadians());

        if (xController.atGoal()) xSpeed = 0;
        if (yController.atGoal()) ySpeed = 0;
        if (omegaController.atGoal()) omegaSpeed = 0;

        swerveDriveSubsystem.setVelocity(new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed), true, false);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDriveSubsystem.stop();
    }
}
