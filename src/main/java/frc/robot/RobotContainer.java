package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimesliceRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.controller.Axis;
import frc.lib.controller.LogitechController;
import frc.lib.controller.ThrustmasterJoystick;
import frc.lib.loops.UpdateManager;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.TimesliceConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.*;

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

        // Set default commands
        // lightsSubsystem.setDefaultCommand(lightsSubsystem.defaultCommand());
        swerveDriveSubsystem.setDefaultCommand(swerveDriveSubsystem.driveCommand(
                getDriveForwardAxis(),
                getDriveStrafeAxis(),
                getDriveRotationAxis(),
                true,
                rightDriveController.getBottomThumb()));
        rightDriveController.nameBottomThumb("Activate Snekey Mode");
        // Set non-button, multi-subsystem triggers
        new Trigger(visionSubsystem::hasTargets)
                .and(new Trigger(() -> visionSubsystem.getAmbiguity() < VisionConstants.ambiguityThreshold))
                .whileTrue(run(() -> {
                    swerveDriveSubsystem.addVisionPoseEstimate(
                            visionSubsystem.getRobotPoseEstimate(), visionSubsystem.getTimestamp());
                }));

        // Set left joystick bindings
        leftDriveController.getLeftTopLeft().onTrue(runOnce(swerveDriveSubsystem::zeroRotation, swerveDriveSubsystem));
        leftDriveController
                .getLeftTopRight()
                .onTrue(runOnce(() -> swerveDriveSubsystem.setPose(new Pose2d()), swerveDriveSubsystem));
        leftDriveController
                .getBottomThumb()
                .whileTrue(swerveDriveSubsystem.driveCommand(
                        getDriveForwardAxis(),
                        getDriveStrafeAxis(),
                        getDriveRotationAxis(),
                        false,
                        rightDriveController.getBottomThumb()));
        leftDriveController.nameLeftTopLeft("Reset Gyro Angle");
        leftDriveController.nameLeftTopRight("Reset Pose");
        leftDriveController.nameBottomThumb("Robot Oriented Drive");

        // Set right joystick bindings
        rightDriveController.getRightBottomMiddle().whileTrue(swerveDriveSubsystem.characterizeCommand(true, true));
        rightDriveController.getRightBottomRight().whileTrue(swerveDriveSubsystem.characterizeCommand(true, false));
        rightDriveController.nameRightBottomMiddle("Characterize Forwards");
        rightDriveController.nameRightBottomMiddle("Characterize Backwards");
        // rightDriveController.getBottomThumb().whileTrue(new ChaseTagCommand(visionSubsystem, swerveDriveSubsystem,
        // swerveDriveSubsystem::getPose));

        // Set operator controller bindings

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
