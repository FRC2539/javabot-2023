package frc.lib.swerve;

import static frc.robot.Constants.VisionConstants.robotToCamera;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import java.util.function.Supplier;

public class ChaseTagCommand extends CommandBase {

    private static final TrapezoidProfile.Constraints xConstraints = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints yConstraints = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints omegaConstraints = new TrapezoidProfile.Constraints(8, 8);

    private static final Transform3d tagToGoal =
            new Transform3d(new Translation3d(0.5, 0.0, 0.0), new Rotation3d(0.0, 0.0, Math.PI));

    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final Supplier<Pose2d> poseProvider;

    private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, xConstraints);
    private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, yConstraints);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(0.05, 0, 0, omegaConstraints);

    private final Timer deadbandTimer = new Timer();

    private static final double DEADBAND_DURATION = 1.0;

    public ChaseTagCommand(
            VisionSubsystem visionSubsystem, SwerveDriveSubsystem swerveDriveSubsystem, Supplier<Pose2d> poseProvider) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.poseProvider = poseProvider;

        xController.setTolerance(0.2);
        yController.setTolerance(0.2);
        omegaController.setTolerance(Units.degreesToRadians(3));
        // omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void initialize() {
        var robotPose = poseProvider.get();
        omegaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());

        deadbandTimer.reset();
        deadbandTimer.stop();
    }

    @Override
    public void execute() {
        var robotPose2d = poseProvider.get();
        var robotPose = new Pose3d(
                robotPose2d.getX(),
                robotPose2d.getY(),
                0.0,
                new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));

        if (visionSubsystem.hasTargets()) {
            // Transform the robot's pose to find the camera's pose
            var cameraPose = robotPose.transformBy(robotToCamera);

            // Transform the camera's pose to the target's pose
            var camToTarget = visionSubsystem.getCameraToTarget();
            var targetPose = cameraPose.transformBy(camToTarget);

            // Transform the tag's pose to set our goal
            var goalPose = targetPose.transformBy(tagToGoal).toPose2d();

            // Update controllers
            xController.setGoal(goalPose.getX());
            yController.setGoal(goalPose.getY());
            omegaController.setGoal(goalPose.getRotation().getRadians());
        }

        if (!visionSubsystem.hasTargets()) {
            if (deadbandTimer.hasElapsed(DEADBAND_DURATION)) {
                // No target has been visible
                swerveDriveSubsystem.stop();

                deadbandTimer.reset();
                deadbandTimer.stop();
            } else {
                deadbandTimer.start();
            }
        } else {
            // Drive to the target
            var xSpeed = xController.calculate(robotPose.getX());
            var ySpeed = yController.calculate(robotPose.getY());
            var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());

            if (xController.atGoal()) xSpeed = 0;
            if (yController.atGoal()) ySpeed = 0;
            if (omegaController.atGoal()) omegaSpeed = 0;

            swerveDriveSubsystem.setVelocity(new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed), true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveDriveSubsystem.stop();
    }
}
