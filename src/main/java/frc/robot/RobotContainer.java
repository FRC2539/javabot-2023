package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimesliceRobot;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.controller.Axis;
import frc.lib.controller.LogitechController;
import frc.lib.controller.ThrustmasterJoystick;
import frc.lib.logging.LoggablePose;
import frc.lib.loops.UpdateManager;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.TimesliceConstants;
import frc.robot.commands.AimAtPoseCommand;
import frc.robot.commands.DriveToPositionCommand;
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
    private final VisionSubsystem visionSubsystem =
            new VisionSubsystem(swerveDriveSubsystem::addVisionPoseEstimate, swerveDriveSubsystem::getPose);
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
            var targetPose = FieldConstants.getNearestPlacementLocation(swerveDriveSubsystem.getPose()).robotPlacementPose;
            targetPoseLogger.set(targetPose);
            return targetPose;
        };

        rightDriveController
                .getLeftThumb()
                .whileTrue(new DriveToPositionCommand(swerveDriveSubsystem, targetPoseSupplier));
        rightDriveController
                .getRightThumb()
                .whileTrue(new AimAtPoseCommand(swerveDriveSubsystem, targetPoseSupplier, getDriveForwardAxis(), getDriveStrafeAxis()));
        rightDriveController.nameLeftThumb("Drive to Pose");
        rightDriveController.nameRightThumb("Aim at Pose");

        /* Set operator controller bindings */
        // Change this. One button for each level. That happens independently of everything else.
        // It will constrain the arm position based on the velocity of the drivetrain
        // Driver will likely get close to the placement area, so this isn't necessary
        // We do need auto aim though, and it needs to be more of an assist that is continuous
        // We do need an auto pickup from double substation though
        // Also track the internal state (where we have game pieces, and which types)
        // This way we can regulate which pipeline to run

        // Also trust encoder estimate more
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
