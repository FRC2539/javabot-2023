package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class AssistToPositionCommand extends CommandBase {
    private static final TrapezoidProfile.Constraints yConstraints = new TrapezoidProfile.Constraints(2, 3);
    private static final TrapezoidProfile.Constraints omegaConstraints = new TrapezoidProfile.Constraints(6, 7);

    private final SwerveDriveSubsystem swerveDriveSubsystem;

    private Supplier<Pose2d> targetPoseSupplier;
    private DoubleSupplier xAxis;

    private final ProfiledPIDController yController = new ProfiledPIDController(8, 0, 0, yConstraints);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(6, 0, 0, omegaConstraints);

    /**
     * Drives to the given pose on the field automatically.
     *
     * While the driving is generally smooth and fast, this algorithm currently assumes zero initial velocity.
     * It will behave erratically at the start if the robot is moving.
     *
     * @param swerveDriveSubsystem
     * @param targetPoseSupplier
     */
    public AssistToPositionCommand(SwerveDriveSubsystem swerveDriveSubsystem, Supplier<Pose2d> targetPoseSupplier, DoubleSupplier forward) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.targetPoseSupplier = targetPoseSupplier;

        this.xAxis = forward;
        yController.setTolerance(0.01);
        omegaController.setTolerance(Units.degreesToRadians(0.5));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void initialize() {
        var robotPose = swerveDriveSubsystem.getPose();
        var robotVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(
                swerveDriveSubsystem.getDesiredVelocity(), swerveDriveSubsystem.getRotation());

        omegaController.reset(robotPose.getRotation().getRadians(), robotVelocity.omegaRadiansPerSecond);
        yController.reset(robotPose.getY(), robotVelocity.vyMetersPerSecond);
    }

    @Override
    public void execute() {
        var robotPose = swerveDriveSubsystem.getPose();
        var targetPose = targetPoseSupplier.get();

        // Update controllers
        yController.setGoal(targetPose.getY());
        omegaController.setGoal(targetPose.getRotation().getRadians());

        var xSpeed = xAxis.getAsDouble();
        var ySpeed = yController.calculate(robotPose.getY());
        var omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());

        if (yController.atGoal()) ySpeed = 0;
        if (omegaController.atGoal()) omegaSpeed = 0;

        swerveDriveSubsystem.setVelocity(new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed), true, true);
    }
}
