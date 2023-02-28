package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix.music.Orchestra;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.controller.LogitechController;
import frc.lib.controller.ThrustmasterJoystick;
import frc.lib.logging.Logger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.PlacementLocation;
import frc.robot.commands.AimAtPoseCommand;
import frc.robot.commands.AssistToPositionCommand;
import frc.robot.commands.MusicRevealCommand;
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

    public static Orchestra orchestra = new Orchestra();

    public static SlewRateLimiter forwardRateLimiter = new SlewRateLimiter(35, -16, 0);
    public static SlewRateLimiter strafeRateLimiter = new SlewRateLimiter(35, -16, 0);

    private final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem();
    private final LightsSubsystem lightsSubsystem = new LightsSubsystem();
    private final GripperSubsystem gripperSubsystem = new GripperSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final VisionSubsystem visionSubsystem =
            new VisionSubsystem(swerveDriveSubsystem::addVisionPoseEstimate, swerveDriveSubsystem::getPose);
    private final ArmSubsystem armSubsystem = new ArmSubsystem(swerveDriveSubsystem);

    public AutonomousManager autonomousManager;

    public RobotContainer(TimedRobot robot) {
        autonomousManager = new AutonomousManager(this);

        configureBindings();
    }

    private void configureBindings() {
        // Decrease the max drivetrain speed when the arm is extended
        swerveDriveSubsystem.setCustomMaxSpeedSupplier(() -> {
            if (armSubsystem.getState() != ArmState.AWAITING_DEPLOYMENT) return 5;
            else if (armSubsystem.getState() == ArmState.AWAITING_DEPLOYMENT
                    && !armSubsystem.isArmApproximatelyAtGoal()) return 5;
            else return Constants.SwerveConstants.maxSpeed;
        });

        /* Set default commands */
        swerveDriveSubsystem.setDefaultCommand(swerveDriveSubsystem.driveCommand(
                this::getDriveForwardAxis, this::getDriveStrafeAxis, this::getDriveRotationAxis, true));

        /* Set non-button, multi-subsystem triggers */
        var hasGamePieceTrigger = new Trigger(() -> gripperSubsystem.hasGamePiece() || intakeSubsystem.hasGamePiece());
        hasGamePieceTrigger.onTrue(runOnce(() -> Logger.log("/Robot/Has Game Piece", true)));
        hasGamePieceTrigger.onFalse(runOnce(() -> Logger.log("/Robot/Has Game Piece", false)));

        hasGamePieceTrigger.debounce(0.1).onTrue(run(() -> LightsSubsystem.LEDSegment.MainStrip.setColor(LightsSubsystem.white), lightsSubsystem).withTimeout(1.5));

        /* Set left joystick bindings */
        leftDriveController.getLeftTopLeft().onTrue(runOnce(swerveDriveSubsystem::zeroRotation, swerveDriveSubsystem));
        leftDriveController
                .getLeftTopRight()
                .onTrue(runOnce(() -> swerveDriveSubsystem.setPose(new Pose2d()), swerveDriveSubsystem));
        leftDriveController.getLeftTopMiddle().onTrue(runOnce(swerveDriveSubsystem::switchToBackupGyro));
        leftDriveController.nameLeftTopLeft("Reset Gyro Angle");
        leftDriveController.nameLeftTopRight("Reset Pose");
        leftDriveController.nameLeftTopMiddle("Use NavX");

        leftDriveController.getLeftBottomRight().whileTrue(new MusicRevealCommand(null, lightsSubsystem));

        leftDriveController.getTrigger().whileTrue(gripperSubsystem.openGripperCommand());
        rightDriveController.getTrigger().whileTrue(gripperSubsystem.ejectFromGripperCommand());
        leftDriveController.nameTrigger("Run Gripper");
        leftDriveController.nameTrigger("Eject Gripper");

        // Leveling
        leftDriveController.getLeftBottomLeft().toggleOnTrue(swerveDriveSubsystem.levelChargeStationCommandDestiny());

        new Trigger(() -> swerveDriveSubsystem.isLevelDestiny()).whileTrue(run(() -> LightsSubsystem.LEDSegment.MainStrip.setRainbowAnimation(1), lightsSubsystem));

        leftDriveController.getLeftBottomMiddle().whileTrue(run(swerveDriveSubsystem::lock, swerveDriveSubsystem));
        leftDriveController.nameLeftBottomLeft("Level Charge Station");
        leftDriveController.nameLeftBottomMiddle("Lock Wheels");

        // Auto Aim Behaviors
        leftDriveController
                .getRightThumb()
                .whileTrue(new AssistToPositionCommand(swerveDriveSubsystem, getTargetPoseSupplier(), this::getDriveForwardAxis));
        leftDriveController
                .getLeftThumb()
                .whileTrue(new AimAtPoseCommand(
                                swerveDriveSubsystem,
                                getTargetAimPoseSupplier(),
                                this::getDriveForwardAxis,
                                this::getDriveStrafeAxis)
                        .alongWith(visionSubsystem.customLimelightModeCommand()));
        leftDriveController.getBottomThumb().whileTrue(gripperSubsystem.dropFromGripperCommand());
        leftDriveController.nameRightThumb("Assist to Pose");
        leftDriveController.nameLeftThumb("Aim at Pose");

        /* Set right joystick bindings */
        rightDriveController.getRightBottomMiddle().whileTrue(swerveDriveSubsystem.characterizeCommand(true, true));
        rightDriveController.getRightBottomRight().whileTrue(swerveDriveSubsystem.characterizeCommand(true, false));
        rightDriveController.nameRightBottomMiddle("Characterize Forwards");
        rightDriveController.nameRightBottomMiddle("Characterize Backwards");

        // Will only need two triggers for this once we have a sensor
        rightDriveController.getLeftThumb().whileTrue(intakeSubsystem.intakeModeCommand());
        rightDriveController.getBottomThumb().whileTrue(intakeSubsystem.shootCommand());
        rightDriveController.getRightThumb().whileTrue(intakeSubsystem.reverseIntakeModeCommand());
        rightDriveController.nameLeftThumb("Run Intake");
        rightDriveController.nameRightThumb("Reverse Intake");
        rightDriveController.nameBottomThumb("Shoot");

        // rightDriveController
        //         .getBottomThumb()
        //         .whileTrue(new AssistedMLPickupCommand(
        //                 swerveDriveSubsystem,
        //                 visionSubsystem,
        //                 this::getDriveForwardAxis,
        //                 this::getDriveStrafeAxis,
        //                 this::getDriveRotationAxis)); // before running set the pipeline
        // rightDriveController.nameBottomThumb("ML Pickup");

        rightDriveController.getRightTopLeft().whileTrue(swerveDriveSubsystem.orchestraCommand());
        rightDriveController.nameRightTopLeft("Symphony");

        // Cardinal drive commands (inverted since arm is back of robot)
        rightDriveController.getPOVUp().whileTrue(swerveDriveSubsystem.cardinalCommand(Rotation2d.fromDegrees(180), this::getDriveForwardAxis, this::getDriveStrafeAxis));
        rightDriveController.getPOVRight().whileTrue(swerveDriveSubsystem.cardinalCommand(Rotation2d.fromDegrees(90), this::getDriveForwardAxis, this::getDriveStrafeAxis));
        rightDriveController.getPOVDown().whileTrue(swerveDriveSubsystem.cardinalCommand(Rotation2d.fromDegrees(0), this::getDriveForwardAxis, this::getDriveStrafeAxis));
        rightDriveController.getPOVLeft().whileTrue(swerveDriveSubsystem.cardinalCommand(Rotation2d.fromDegrees(-90), this::getDriveForwardAxis, this::getDriveStrafeAxis));

        /* Set operator controller bindings */
        operatorController.getY().onTrue(armSubsystem.highManualCubeCommand());
        operatorController.getA().onTrue(armSubsystem.tippedPickupCommand());
        operatorController.getB().onTrue(armSubsystem.substationPickupCommand());
        operatorController.nameY("High Manual for Cube");
        operatorController.nameA("Tipped Pickup");

        operatorController.getBack().whileTrue(gripperSubsystem.ejectFromGripperCommand());
        operatorController.nameBack("Backup Gripper");

        operatorController
                .getStart()
                .toggleOnTrue(run(() -> LightsSubsystem.LEDSegment.MainStrip.setRainbowAnimation(1), lightsSubsystem));
        operatorController.nameStart("Rainbow Mode");

        // Manual arm controls, no sussy stuff here
        operatorController.getDPadDown().onTrue(armSubsystem.pickupCommand());
        operatorController.getDPadLeft().onTrue(armSubsystem.awaitingDeploymentCommand());
        operatorController.getDPadUp().onTrue(armSubsystem.highManualConeCommand());
        operatorController.getDPadRight().onTrue(armSubsystem.midManualCommand());
        operatorController.nameDPadDown("Pickup");
        operatorController.nameDPadLeft("Awaiting Deployment");
        operatorController.nameDPadUp("High Manual");
        operatorController.nameDPadRight("Mid Manual");

        operatorController
                .getRightTrigger()
                .whileTrue(run(
                        () -> LightsSubsystem.LEDSegment.MainStrip.setStrobeAnimation(LightsSubsystem.yellow, .3),
                        lightsSubsystem));
        operatorController
                .getLeftTrigger()
                .whileTrue(run(
                        () -> LightsSubsystem.LEDSegment.MainStrip.setStrobeAnimation(LightsSubsystem.purple, .3),
                        lightsSubsystem));
        operatorController.nameRightTrigger("Indicate Cone");
        operatorController.nameLeftTrigger("Indicate Cube");

        rightDriveController.sendButtonNamesToNT();
        leftDriveController.sendButtonNamesToNT();
        operatorController.sendButtonNamesToNT();
    }

    public Command getAutonomousCommand() {
        return autonomousManager.getAutonomousCommand();
    }

    public double getDriveForwardAxis() {
        return forwardRateLimiter.calculate(
                -square(deadband(leftDriveController.getYAxis().getRaw(), 0.05)) * Constants.SwerveConstants.maxSpeed);
    }

    public double getDriveStrafeAxis() {
        return strafeRateLimiter.calculate(
                -square(deadband(leftDriveController.getXAxis().getRaw(), 0.05)) * Constants.SwerveConstants.maxSpeed);
    }

    public double getDriveRotationAxis() {
        return -square(deadband(rightDriveController.getXAxis().getRaw(), 0.05))
                * Constants.SwerveConstants.maxAngularVelocity
                * 0.75;
    }

    private static double deadband(double value, double tolerance) {
        if (Math.abs(value) < tolerance) return 0.0;

        return Math.copySign(value, (value - tolerance) / (1.0 - tolerance));
    }

    private static double square(double value) {
        return Math.copySign(value * value, value);
    }

    public Supplier<Pose2d> getTargetPoseSupplier() {
        return () -> {
            PlacementLocation targetLocation =
                    FieldConstants.getNearestPlacementLocation(swerveDriveSubsystem.getPose());

            var targetPose = targetLocation.robotPlacementPose;

            Logger.log("/SwerveDriveSubsystem/TargetPose", targetPose);
            return targetPose;
        };
    }

    public Supplier<Pose2d> getTargetAimPoseSupplier() {
        return () -> {
            PlacementLocation targetLocation =
                    FieldConstants.getNearestPlacementLocation(swerveDriveSubsystem.getPose());

            ArmState armState = armSubsystem.getState();
            Pose3d targetPose3d;

            switch (armState) {
                case HYBRID_MANUAL:
                    targetPose3d = targetLocation.getHybridPose();
                    break;
                case MID_MANUAL_CONE:
                    targetPose3d = targetLocation.getMidPose();

                    // If cone, enable limelight cone mode 1
                    // if (targetLocation.isCone) {
                    //     visionSubsystem.setLimelightMode(LimelightMode.RETROREFLECTIVEMID);

                    //     if (visionSubsystem.hasLLFieldRelativeRetroflectiveEstimate()) {
                    //         var visionEstimate = visionSubsystem
                    //                 .getValidLLRetroreflectiveEstimate()
                    //                 .get();

                    //         targetPose3d = new Pose3d(
                    //                 visionEstimate.getX(),
                    //                 visionEstimate.getY(),
                    //                 targetPose3d.getZ(),
                    //                 targetPose3d.getRotation());
                    //     }
                    // } else visionSubsystem.setLimelightMode(LimelightMode.APRILTAG);

                    break;
                default: // HIGH or other
                    targetPose3d = targetLocation.getHighPose();

                    // If cone, enable limelight cone mode 2
                    // if (targetLocation.isCone) {
                    //     visionSubsystem.setLimelightMode(LimelightMode.RETROREFLECTIVEHIGH);

                    //     if (visionSubsystem.hasLLFieldRelativeRetroflectiveEstimate()) {
                    //         var visionEstimate = visionSubsystem
                    //                 .getValidLLRetroreflectiveEstimate()
                    //                 .get();

                    //         targetPose3d = new Pose3d(
                    //                 visionEstimate.getX(),
                    //                 visionEstimate.getY(),
                    //                 targetPose3d.getZ(),
                    //                 targetPose3d.getRotation());
                    //     }
                    // } else visionSubsystem.setLimelightMode(LimelightMode.APRILTAG);

                    break;
            }

            var targetPose =
                    targetPose3d.toPose2d().plus(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(0)));

            Logger.log("/SwerveDriveSubsystem/TargetPose", targetPose);
            return targetPose;
        };
    }

    public SwerveDriveSubsystem getSwerveDriveSubsystem() {
        return swerveDriveSubsystem;
    }

    public LightsSubsystem getLightsSubsystem() {
        return lightsSubsystem;
    }

    public GripperSubsystem getGripperSubsystem() {
        return gripperSubsystem;
    }

    public IntakeSubsystem getIntakeSubsystem() {
        return intakeSubsystem;
    }

    public VisionSubsystem getVisionSubsystem() {
        return visionSubsystem;
    }

    public ArmSubsystem getArmSubsystem() {
        return armSubsystem;
    }
}
