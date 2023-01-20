package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimesliceRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.controller.Axis;
import frc.lib.controller.LogitechController;
import frc.lib.controller.ThrustmasterJoystick;
import frc.lib.logging.LoggablePose;
import frc.lib.loops.UpdateManager;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.TimesliceConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.subsystems.*;
import frc.robot.utils.AutoPlaceManager;
import java.util.function.Supplier;

public class RobotContainer {
    private final ThrustmasterJoystick leftDriveController =
            new ThrustmasterJoystick(ControllerConstants.LEFT_DRIVE_CONTROLLER);
    private final ThrustmasterJoystick rightDriveController =
            new ThrustmasterJoystick(ControllerConstants.RIGHT_DRIVE_CONTROLLER);
    private final LogitechController operatorController =
            new LogitechController(ControllerConstants.OPERATOR_CONTROLLER);

    private final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem();
    // private final LightsSubsystem lightsSubsystem = new LightsSubsystem();
    private final VisionSubsystem visionSubsystem = new VisionSubsystem();
    private final ArmSubsystem armSubsystem = new ArmSubsystem();

    private AutonomousManager autonomousManager;
    private UpdateManager updateManager;

    public RobotContainer(TimesliceRobot robot) {
        updateManager = new UpdateManager(robot);
        autonomousManager = new AutonomousManager(this);

        // Allocate timeslices
        updateManager.schedule(swerveDriveSubsystem, TimesliceConstants.DRIVETRAIN_PERIOD);

        configureBindings();
    }

    private void configureBindings() {
        leftDriveController.getXAxis().setScale(Constants.SwerveConstants.maxSpeed);
        leftDriveController.getYAxis().setScale(Constants.SwerveConstants.maxSpeed);
        rightDriveController.getXAxis().setScale(Constants.SwerveConstants.maxAngularVelocity);
        leftDriveController.getXAxis().setInverted(true);
        leftDriveController.getYAxis().setInverted(true);
        rightDriveController.getXAxis().setInverted(true);

        /* Set default commands */
        // lightsSubsystem.setDefaultCommand(lightsSubsystem.defaultCommand());
        swerveDriveSubsystem.setDefaultCommand(swerveDriveSubsystem.driveCommand(
                getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis(), true));

        /* Set non-button, multi-subsystem triggers */
        new Trigger(visionSubsystem::hasTargets)
                .and(new Trigger(() -> visionSubsystem.getAmbiguity() < VisionConstants.ambiguityThreshold))
                .whileTrue(run(() -> {
                    swerveDriveSubsystem.addVisionPoseEstimate(
                            visionSubsystem.getRobotPoseEstimate(), visionSubsystem.getTimestamp());
                }));

        /* Set left joystick bindings */
        leftDriveController.getLeftTopLeft().onTrue(runOnce(swerveDriveSubsystem::zeroRotation, swerveDriveSubsystem));
        leftDriveController
                .getLeftTopRight()
                .onTrue(runOnce(() -> swerveDriveSubsystem.setPose(new Pose2d()), swerveDriveSubsystem));
        leftDriveController
                .getBottomThumb()
                .whileTrue(swerveDriveSubsystem.preciseDriveCommand(
                        getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis(), true));
        leftDriveController.nameLeftTopLeft("Reset Gyro Angle");
        leftDriveController.nameLeftTopRight("Reset Pose");
        leftDriveController.nameBottomThumb("Precise Driving");

        // Leveling
        leftDriveController.getLeftBottomLeft().toggleOnTrue(swerveDriveSubsystem.levelChargeStationCommand());
        leftDriveController.getLeftBottomMiddle().whileTrue(run(swerveDriveSubsystem::lock, swerveDriveSubsystem));
        leftDriveController.nameLeftBottomLeft("Level Charge Station");
        leftDriveController.nameLeftBottomMiddle("Lock Wheels");

        /* Set right joystick bindings */
        rightDriveController.getRightBottomMiddle().whileTrue(swerveDriveSubsystem.characterizeCommand(true, true));
        rightDriveController.getRightBottomRight().whileTrue(swerveDriveSubsystem.characterizeCommand(true, false));
        rightDriveController.nameRightBottomMiddle("Characterize Forwards");
        rightDriveController.nameRightBottomMiddle("Characterize Backwards");

        LoggablePose targetPoseLogger = new LoggablePose("/SwerveDriveSubsystem/TargetPose");

        Supplier<Pose2d> targetPoseSupplier = () -> {
            var targetPose = visionSubsystem
                    .getAprilTagFieldPose()
                    .toPose2d()
                    .transformBy(new Transform2d(new Translation2d(1.5, 0), Rotation2d.fromDegrees(180)));
            targetPoseLogger.set(targetPose);
            return targetPose;
        };

        rightDriveController
                .getBottomThumb()
                .whileTrue(new DriveToPoseCommand(swerveDriveSubsystem, targetPoseSupplier));
        rightDriveController.nameBottomThumb("Drive to Pose");

        /* Set operator controller bindings */
        AutoPlaceManager.initializeAutoPlaceManager();
        operatorController.getDPadUp().onTrue(runOnce(() -> AutoPlaceManager.incrementLevel()));
        operatorController.getDPadRight().onTrue(runOnce(() -> AutoPlaceManager.incrementRow()));
        operatorController.getDPadDown().onTrue(runOnce(() -> AutoPlaceManager.decrementLevel()));
        operatorController.getDPadLeft().onTrue(runOnce(() -> AutoPlaceManager.decrementRow()));

        rightDriveController.sendButtonNamesToNT();
        leftDriveController.sendButtonNamesToNT();
        operatorController.sendButtonNamesToNT();
    }

    public Command getAutonomousCommand() {
        return autonomousManager.getAutonomousCommand();
    }

    public Axis getDriveForwardAxis() {
        return leftDriveController.getYAxis();
    }

    public Axis getDriveStrafeAxis() {
        return leftDriveController.getXAxis();
    }

    public Axis getDriveRotationAxis() {
        return rightDriveController.getXAxis();
    }

    public SwerveDriveSubsystem getSwerveDriveSubsystem() {
        return swerveDriveSubsystem;
    }

    // public LightsSubsystem getLightsSubsystem() {
    //     return lightsSubsystem;
    // }

    public VisionSubsystem getVisionSubsystem() {
        return visionSubsystem;
    }

    public ArmSubsystem getArmSubsystem() {
        return armSubsystem;
    }
}
