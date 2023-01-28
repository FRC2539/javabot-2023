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

public class AimAtPoseCommand extends CommandBase {
    private static final TrapezoidProfile.Constraints omegaConstraints = new TrapezoidProfile.Constraints(8, 8);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(5, 0, 0, omegaConstraints);

    private final SwerveDriveSubsystem swerveDriveSubsystem;

    private Supplier<Pose2d> targetPoseSupplier;

    DoubleSupplier forwardAxis;
    DoubleSupplier strafeAxis;

    /**
     * Aims at the given pose on the field automatically.
     *
     * While the driving is generally smooth and fast, this algorithm currently assumes zero initial velocity.
     * It will behave erratically at the start if the robot is moving.
     *
     * @param swerveDriveSubsystem
     * @param targetPoseSupplier
     */
    public AimAtPoseCommand(
            SwerveDriveSubsystem swerveDriveSubsystem,
            Supplier<Pose2d> targetPoseSupplier,
            DoubleSupplier forwardAxis,
            DoubleSupplier strafeAxis) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.targetPoseSupplier = targetPoseSupplier;

        this.forwardAxis = forwardAxis;
        this.strafeAxis = strafeAxis;

        omegaController.setTolerance(Units.degreesToRadians(3));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void initialize() {
        var robotPose = swerveDriveSubsystem.getPose();
        var robotVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(
                swerveDriveSubsystem.getDesiredVelocity(), swerveDriveSubsystem.getRotation());

        omegaController.reset(robotPose.getRotation().getRadians(), -robotVelocity.omegaRadiansPerSecond);
    }

    @Override
    public void execute() {
        var robotPose = swerveDriveSubsystem.getPose();
        var targetPose = targetPoseSupplier.get();

        var targetAngle = targetPose.minus(robotPose).getTranslation().getAngle();

        omegaController.setGoal(robotPose.getRotation().plus(targetAngle).getRadians());

        var xSpeed = forwardAxis.getAsDouble();
        var ySpeed = strafeAxis.getAsDouble();
        var omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());

        if (omegaController.atGoal()) omegaSpeed = 0;

        swerveDriveSubsystem.setVelocity(new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed), true, true);
    }
}
