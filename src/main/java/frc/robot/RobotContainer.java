package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimesliceRobot;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.controller.Axis;
import frc.lib.controller.LogitechController;
import frc.lib.controller.ThrustmasterJoystick;
import frc.lib.logging.Logger;
import frc.lib.loops.UpdateManager;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.PlacementLocation;
import frc.robot.Constants.TimesliceConstants;
import frc.robot.commands.AimAtPoseCommand;
import frc.robot.commands.DriveToPositionCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ArmSubsystem.ArmState;
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

        Supplier<Pose2d> targetPoseSupplier = () -> {
            PlacementLocation targetLocation =
                    FieldConstants.getNearestPlacementLocation(swerveDriveSubsystem.getPose());

            var targetPose = targetLocation.robotPlacementPose;

            Logger.getInstance().log("/SwerveDriveSubsystem/TargetPose", targetPose);
            return targetPose;
        };

        Supplier<Pose2d> targetAimPoseSupplier = () -> {
            PlacementLocation targetLocation =
                    FieldConstants.getNearestPlacementLocation(swerveDriveSubsystem.getPose());

            ArmState armState = armSubsystem.getState();
            Pose3d targetPose3d;

            switch (armState) {
                case HYBRID:
                    targetPose3d = targetLocation.getHybridPose();
                    break;
                case MID:
                    targetPose3d = targetLocation.getMidPose();
                    break;
                case HIGH:
                    targetPose3d = targetLocation.getHighPose();
                    break;
                default:
                    targetPose3d = new Pose3d(targetLocation.robotPlacementPose.plus(
                            new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180))));
                    break;
            }

            var targetPose =
                    targetPose3d.toPose2d().plus(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180)));

            Logger.getInstance().log("/SwerveDriveSubsystem/TargetPose", targetPose);
            return targetPose;
        };

        rightDriveController
                .getLeftThumb()
                .whileTrue(new DriveToPositionCommand(swerveDriveSubsystem, targetPoseSupplier));
        rightDriveController
                .getRightThumb()
                .whileTrue(new AimAtPoseCommand(
                        swerveDriveSubsystem, targetAimPoseSupplier, getDriveForwardAxis(), getDriveStrafeAxis()));
        rightDriveController.nameLeftThumb("Drive to Pose");
        rightDriveController.nameRightThumb("Aim at Pose");

        /* Set operator controller bindings */
        operatorController.getA().onTrue(runOnce(armSubsystem::setHybrid, armSubsystem));
        operatorController.getB().onTrue(runOnce(armSubsystem::setMid, armSubsystem));
        operatorController.getY().onTrue(runOnce(armSubsystem::setHigh, armSubsystem));
        operatorController.getX().onTrue(runOnce(armSubsystem::setAwaitingDeployment, armSubsystem));
        operatorController.nameA("Place Hybrid");
        operatorController.nameB("Place Mid");
        operatorController.nameY("Place High");
        operatorController.nameX("Protect Arm");

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
