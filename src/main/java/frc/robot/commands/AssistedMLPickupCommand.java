package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.LimelightMode;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class AssistedMLPickupCommand extends CommandBase {
    private static final TrapezoidProfile.Constraints distanceConstraints = new TrapezoidProfile.Constraints(6, 4);
    private static final TrapezoidProfile.Constraints omegaConstraints = new TrapezoidProfile.Constraints(8, 8);

    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final VisionSubsystem visionSubsystem;

    private final ProfiledPIDController distanceController = new ProfiledPIDController(3, 0, 0, distanceConstraints);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(5, 0, 0, omegaConstraints);

    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;

    @SuppressWarnings("unused")
    private final DoubleSupplier rotate;

    private Optional<Pose2d> lastSeenCargoPose = Optional.empty();

    private static final double PICKUP_DISTANCE = 0.97;

    /**
     * Drives to the given pose on the field automatically.
     *
     * While the driving is generally smooth and fast, this algorithm currently assumes zero initial velocity.
     * It will behave erratically at the start if the robot is moving.
     *
     * @param swerveDriveSubsystem
     * @param targetPoseSupplier
     */
    public AssistedMLPickupCommand(
            SwerveDriveSubsystem swerveDriveSubsystem,
            VisionSubsystem visionSubsystem,
            DoubleSupplier forward,
            DoubleSupplier strafe,
            DoubleSupplier rotate) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.visionSubsystem = visionSubsystem;

        distanceController.setTolerance(0.1);
        omegaController.setTolerance(Units.degreesToRadians(3));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerveDriveSubsystem, visionSubsystem);

        this.forward = forward;
        this.strafe = strafe;
        this.rotate = rotate;
    }

    @Override
    public void initialize() {
        lastSeenCargoPose = Optional.empty();

        var robotPose = swerveDriveSubsystem.getPose();
        var robotVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(
                swerveDriveSubsystem.getDesiredVelocity(), swerveDriveSubsystem.getRotation());

        omegaController.reset(robotPose.getRotation().getRadians(), 0);
        distanceController.reset(robotPose.getX(), 0);

        visionSubsystem.setLimelightMode(LimelightMode.ML);
    }

    @Override
    public void execute() {
        Optional<Pose2d> optionalCargo = visionSubsystem.getLLMLFieldPoseEstimate();
        if (optionalCargo.isPresent()) {
            lastSeenCargoPose = optionalCargo;
        }

        var robotPose = swerveDriveSubsystem.getPose();
        var targetPose = getDesiredPosition();

        // Update controllers
        distanceController.setGoal(0);
        omegaController.setGoal(targetPose.getRotation().getRadians());

        var distanceSpeed = distanceController.calculate(getDesiredPosition()
                .minus(swerveDriveSubsystem.getPose())
                .getTranslation()
                .getNorm());
        var omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());

        if (distanceController.atGoal()) distanceSpeed = 0;
        if (omegaController.atGoal()) omegaSpeed = 0;

        double towardsValue = getDriverValueTowardsBall();

        Translation2d finalMotion = new Translation2d(
                /*distanceSpeed * towardsValue*/ 1, targetPose.getRotation().unaryMinus());

        swerveDriveSubsystem.setVelocity(
                new ChassisSpeeds(finalMotion.getX(), finalMotion.getY(), omegaSpeed), true, true);
    }

    @Override
    public boolean isFinished() {
        return distanceController.atGoal() && omegaController.atGoal() && lastSeenCargoPose.isPresent();
    }

    @Override
    public void end(boolean interrupted) {
        swerveDriveSubsystem.stop();
    }

    private Pose2d getDesiredPosition() {
        if (lastSeenCargoPose.isEmpty()) return swerveDriveSubsystem.getPose();

        Transform2d poseDifference = swerveDriveSubsystem.getPose().minus(lastSeenCargoPose.get());
        Rotation2d intendedRotation = poseDifference.getTranslation().getAngle();
        Translation2d finalTranslationLocation =
                lastSeenCargoPose.get().getTranslation().plus(new Translation2d(PICKUP_DISTANCE, intendedRotation));

        return new Pose2d(finalTranslationLocation, intendedRotation);
    }

    private Rotation2d getAngleToDesiredPose() {
        return getDesiredPosition().getRotation().unaryMinus();
    }

    private double getDriverValueTowardsBall() {
        var desiredPositionAngle = getAngleToDesiredPose();
        double towardsValue = -forward.getAsDouble() * desiredPositionAngle.getCos()
                + -strafe.getAsDouble() * desiredPositionAngle.getSin();
        return towardsValue;
    }
}
