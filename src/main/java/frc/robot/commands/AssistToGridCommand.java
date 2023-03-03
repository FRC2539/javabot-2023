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
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class AssistToGridCommand extends CommandBase {
    private static final TrapezoidProfile.Constraints yConstraints = new TrapezoidProfile.Constraints(5, 5);
    private static final TrapezoidProfile.Constraints omegaConstraints = new TrapezoidProfile.Constraints(7, 7);

    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final VisionSubsystem visionSubsystem;

    private Supplier<PlacementLocation> targetPoseSupplier;
    private DoubleSupplier xAxis;

    private final ProfiledPIDController yController = new ProfiledPIDController(9, 0, 0, yConstraints);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(8, 0, 0, omegaConstraints);

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
        yController.setTolerance(0.008);
        omegaController.setTolerance(Units.degreesToRadians(0.4));
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
    }

    @Override
    public void execute() {
        var robotPose = swerveDriveSubsystem.getPose();
        var targetPlacementLocation = targetPoseSupplier.get();
        var targetPose = targetPlacementLocation.robotPlacementPose;

        // Use vision for cones if it is available
        if (targetPlacementLocation.isCone) {
            visionSubsystem.setLimelightMode(LimelightMode.RETROREFLECTIVEHIGH);

            var retroreflectivePose = visionSubsystem.getLLFieldRelativeRetroflectiveEstimate();

            // Use retroreflective y estimate
            if (retroreflectivePose.isPresent()) {
                targetPose =
                        new Pose2d(targetPose.getX(), retroreflectivePose.get().getY(), targetPose.getRotation());
            }
        } else {
            visionSubsystem.setLimelightMode(LimelightMode.APRILTAG);
        }

        // Update controllers
        yController.setGoal(targetPose.getY());
        omegaController.setGoal(targetPose.getRotation().getRadians());

        var xSpeed = xAxis.getAsDouble();
        var ySpeed = yController.calculate(robotPose.getY());
        var omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());

        // Prevent unecessary movement
        if (yController.atGoal()) ySpeed = 0;
        if (omegaController.atGoal()) omegaSpeed = 0;

        // Indicate to the driver if we are aimed within tolerance
        if (yController.atGoal() && omegaController.atGoal()) LEDSegment.MainStrip.setColor(LightsSubsystem.green);

        swerveDriveSubsystem.setVelocity(new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed), true, true);
    }
}
