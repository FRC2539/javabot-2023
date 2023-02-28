package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FieldConstants.PlacementLocation;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.LightsSubsystem.LEDSegment;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.LimelightMode;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class AssistToGridCommand extends CommandBase {
    private static final TrapezoidProfile.Constraints yConstraints = new TrapezoidProfile.Constraints(2, 3);
    private static final TrapezoidProfile.Constraints omegaConstraints = new TrapezoidProfile.Constraints(6, 7);

    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final VisionSubsystem visionSubsystem;

    private Supplier<PlacementLocation> targetPoseSupplier;
    private DoubleSupplier xAxis;

    private Optional<Pose2d> validTargetPose;

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
    public AssistToGridCommand(
            SwerveDriveSubsystem swerveDriveSubsystem,
            VisionSubsystem visionSubsystem,
            LightsSubsystem lightsSubsystem,
            Supplier<PlacementLocation> targetPoseSupplier,
            DoubleSupplier forward) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.targetPoseSupplier = targetPoseSupplier;

        this.xAxis = forward;
        yController.setTolerance(0.01);
        omegaController.setTolerance(Units.degreesToRadians(0.5));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerveDriveSubsystem, visionSubsystem, lightsSubsystem);
    }

    @Override
    public void initialize() {
        var robotPose = swerveDriveSubsystem.getPose();
        var robotVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(
                swerveDriveSubsystem.getDesiredVelocity(), swerveDriveSubsystem.getRotation());

        omegaController.reset(robotPose.getRotation().getRadians(), robotVelocity.omegaRadiansPerSecond);
        yController.reset(robotPose.getY(), robotVelocity.vyMetersPerSecond);

        validTargetPose = Optional.empty();
    }

    @Override
    public void execute() {
        var robotPose = swerveDriveSubsystem.getPose();
        var targetPlacementLocation = targetPoseSupplier.get();
        var targetPose = targetPlacementLocation.robotPlacementPose;

        // Use vision for cones if it is available
        if (targetPlacementLocation.isCone) {
            visionSubsystem.setLimelightMode(LimelightMode.RETROREFLECTIVEMID);

            var retroreflectivePose = visionSubsystem.getLLFieldRelativeRetroflectiveEstimate();

            // Use retroreflective y estimate
            if (retroreflectivePose.isPresent()) {
                targetPose =
                        new Pose2d(targetPose.getX(), retroreflectivePose.get().getY(), targetPose.getRotation());

                validTargetPose = Optional.of(targetPose);
            } else {
                // No tape? Use original placement location
                if (validTargetPose.isEmpty()) validTargetPose = Optional.of(targetPose);
            }
        } else {
            visionSubsystem.setLimelightMode(LimelightMode.APRILTAG);

            // Not a cone? Use the original placement location
            if (validTargetPose.isEmpty()) validTargetPose = Optional.of(targetPose);
        }

        // Update controllers
        yController.setGoal(validTargetPose.get().getY());
        omegaController.setGoal(validTargetPose.get().getRotation().getRadians());

        var xSpeed = xAxis.getAsDouble();
        var ySpeed = yController.calculate(robotPose.getY());
        var omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());

        // Prevent unecessary movement
        if (yController.atGoal()) ySpeed = 0;
        if (omegaController.atGoal()) omegaSpeed = 0;

        // Indicate to the driver if we are aimed within tolerance
        LEDSegment.MainStrip.setColor(LightsSubsystem.green);

        swerveDriveSubsystem.setVelocity(new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed), true, true);
    }
}
