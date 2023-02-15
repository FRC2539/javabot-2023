package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.controller.Axis;
import frc.lib.controller.LogitechController;
import frc.lib.controller.ThrustmasterJoystick;
import frc.lib.logging.Logger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.PlacementLocation;
import frc.robot.commands.AimAtPoseCommand;
import frc.robot.commands.AssistedDriveToPositionCommand;
import frc.robot.commands.DriveToPositionCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import java.util.function.Supplier;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.music.Orchestra;

public class RobotContainer {
    private final ThrustmasterJoystick leftDriveController =
            new ThrustmasterJoystick(ControllerConstants.LEFT_DRIVE_CONTROLLER);
    private final ThrustmasterJoystick rightDriveController =
            new ThrustmasterJoystick(ControllerConstants.RIGHT_DRIVE_CONTROLLER);
    private final LogitechController operatorController =
            new LogitechController(ControllerConstants.OPERATOR_CONTROLLER);

    public static Orchestra orchestra = new Orchestra();

    private final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem();
    private final LightsSubsystem lightsSubsystem = new LightsSubsystem();
    private final GripperSubsystem gripperSubsystem = new GripperSubsystem();
    // private final VisionSubsystem visionSubsystem =
    //         new VisionSubsystem(swerveDriveSubsystem::addVisionPoseEstimate, swerveDriveSubsystem::getPose);
    private final ArmSubsystem armSubsystem = new ArmSubsystem(swerveDriveSubsystem::getPose);

    public AutonomousManager autonomousManager;

    public RobotContainer(TimedRobot robot) {
        autonomousManager = new AutonomousManager(this);

        configureBindings();
    }

    private void configureBindings() {
        leftDriveController.getXAxis().setScale(Constants.SwerveConstants.maxSpeed);
        leftDriveController.getYAxis().setScale(Constants.SwerveConstants.maxSpeed);
        rightDriveController.getXAxis().setScale(Constants.SwerveConstants.maxAngularVelocity * 0.75);
        leftDriveController.getXAxis().setInverted(true);
        leftDriveController.getYAxis().setInverted(true);
        rightDriveController.getXAxis().setInverted(true);

        /* Set default commands */
        swerveDriveSubsystem.setDefaultCommand(swerveDriveSubsystem.driveCommand(
                getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis(), true));

        /* Set non-button, multi-subsystem triggers */

        /* Set left joystick bindings */
        leftDriveController.getLeftTopLeft().onTrue(runOnce(swerveDriveSubsystem::zeroRotation, swerveDriveSubsystem));
        leftDriveController
                .getLeftTopRight()
                .onTrue(runOnce(() -> swerveDriveSubsystem.setPose(new Pose2d()), swerveDriveSubsystem));
        leftDriveController.getLeftTopMiddle().onTrue(runOnce(swerveDriveSubsystem::switchToBackupGyro));
        leftDriveController
                .getBottomThumb()
                .whileTrue(swerveDriveSubsystem.preciseDriveCommand(
                        getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis(), true));
        leftDriveController.nameLeftTopLeft("Reset Gyro Angle");
        leftDriveController.nameLeftTopRight("Reset Pose");
        leftDriveController.nameBottomThumb("Precise Driving");

        leftDriveController.getTrigger().whileTrue(gripperSubsystem.openGripperCommand());
        rightDriveController.getTrigger().whileTrue(gripperSubsystem.ejectFromGripperCommand());

        leftDriveController
                .getRightTopRight()
                .toggleOnTrue(armSubsystem.passthroughCommand(
                        operatorController.getLeftXAxis(),
                        operatorController.getLeftYAxis(),
                        operatorController.getRightXAxis()));

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

        rightDriveController.getRightTopLeft().whileTrue(startEnd(() -> {
            armSubsystem.setBrake();
 
            ErrorCode error = orchestra.loadMusic("acdc.chrp");

            System.out.println(error.value);

            orchestra.play();
        }, () -> {
            orchestra.stop();
        }, swerveDriveSubsystem, armSubsystem));

        Supplier<Pose2d> targetPoseSupplier = () -> {
            PlacementLocation targetLocation =
                    FieldConstants.getNearestPlacementLocation(swerveDriveSubsystem.getPose());

            var targetPose = targetLocation.robotPlacementPose;

            Logger.log("/SwerveDriveSubsystem/TargetPose", targetPose);
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

            var targetPose = targetPose3d
                    .toPose2d()
                    .plus(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(0))); // was 180

            Logger.log("/SwerveDriveSubsystem/TargetPose", targetPose);
            return targetPose;
        };

        rightDriveController
                .getLeftThumb()
                .whileTrue(new DriveToPositionCommand(swerveDriveSubsystem, targetPoseSupplier));
        rightDriveController
                .getRightThumb()
                .whileTrue(new AimAtPoseCommand(
                        swerveDriveSubsystem, targetAimPoseSupplier, getDriveForwardAxis(), getDriveStrafeAxis()));
        rightDriveController
                .getBottomThumb()
                .whileTrue(new AssistedDriveToPositionCommand(
                        swerveDriveSubsystem, targetPoseSupplier, getDriveForwardAxis()));
        rightDriveController.nameLeftThumb("Drive to Pose");
        rightDriveController.nameRightThumb("Aim at Pose");
        rightDriveController.nameBottomThumb("Assisted Drive");

        /* Set operator controller bindings */
        operatorController.getA().onTrue(runOnce(armSubsystem::setHybrid, armSubsystem));
        operatorController.getB().onTrue(runOnce(armSubsystem::setMid, armSubsystem));
        operatorController.getY().onTrue(runOnce(armSubsystem::setHigh, armSubsystem));
        operatorController.getX().onTrue(runOnce(armSubsystem::setAwaitingDeployment, armSubsystem));
        operatorController.nameA("Place Hybrid");
        operatorController.nameB("Place Mid");
        operatorController.nameY("Place High");
        operatorController.nameX("Protect Arm");

        operatorController.getBack().onTrue(runOnce(armSubsystem::setNetworkTablesMode, armSubsystem));

        // Manual arm controls, no sussy stuff here
        operatorController.getDPadDown().onTrue(runOnce(armSubsystem::setHybridManual, armSubsystem));
        operatorController.getDPadLeft().onTrue(runOnce(armSubsystem::setAwaitingDeployment, armSubsystem));
        operatorController.getDPadUp().onTrue(runOnce(armSubsystem::setHighManual, armSubsystem));
        operatorController.getDPadRight().onTrue(runOnce(armSubsystem::setMidManual, armSubsystem));
        operatorController.nameDPadDown("Hybrid Manual");
        operatorController.nameDPadLeft("Mid Manual");
        operatorController.nameDPadUp("High Manual");
        operatorController.nameDPadRight("Pickup");

        operatorController
                .getRightTrigger()
                .whileTrue(run(
                        () -> LightsSubsystem.LEDSegment.MainStrip.setColor(LightsSubsystem.yellow), lightsSubsystem));
        operatorController
                .getLeftTrigger()
                .whileTrue(run(
                        () -> LightsSubsystem.LEDSegment.MainStrip.setColor(LightsSubsystem.purple), lightsSubsystem));

        // operatorController.getRightTrigger().whileTrue(lightsSubsystem.indicateConeCommand());
        // operatorController.getLeftTrigger().whileTrue(lightsSubsystem.indicateCubeCommand());
        operatorController.nameRightTrigger("Indicate Cone");
        operatorController.nameLeftTrigger("Indicate Cube");

        // 20.5
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

    public LightsSubsystem getLightsSubsystem() {
        return lightsSubsystem;
    }

    // public VisionSubsystem getVisionSubsystem() {
    //     return visionSubsystem;
    // }

    public ArmSubsystem getArmSubsystem() {
        return armSubsystem;
    }
}
