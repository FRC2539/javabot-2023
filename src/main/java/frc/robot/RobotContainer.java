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
import frc.robot.commands.IndicateGridAimedCommand;
import frc.robot.commands.IndicateSubstationAimedCommand;
import frc.robot.commands.IntakingAimAssistCommand;
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

    public static SlewRateLimiter forwardRateLimiter = new SlewRateLimiter(40, -40, 0);
    public static SlewRateLimiter strafeRateLimiter = new SlewRateLimiter(40, -40, 0);

    private final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem();
    private final LightsSubsystem lightsSubsystem = new LightsSubsystem();
    private final GripperSubsystem gripperSubsystem = new GripperSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final VisionSubsystem visionSubsystem = new VisionSubsystem(swerveDriveSubsystem);
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
        Trigger hasGamePieceTrigger =
                new Trigger(() -> gripperSubsystem.hasGamePiece() || intakeSubsystem.hasGamePiece());
        hasGamePieceTrigger.onTrue(runOnce(() -> Logger.log("/Robot/Has Game Piece", true)));
        hasGamePieceTrigger.onFalse(runOnce(() -> Logger.log("/Robot/Has Game Piece", false)));

        new Trigger(() -> gripperSubsystem.hasGamePiece() && gripperSubsystem.isOpen())
                .onTrue(run(() -> LightsSubsystem.LEDSegment.MainStrip.setColor(LightsSubsystem.white), lightsSubsystem)
                        .withTimeout(1.5));

        new Trigger(() -> intakeSubsystem.hasGamePiece())
                .debounce(0.05)
                .onTrue(run(() -> LightsSubsystem.LEDSegment.MainStrip.setColor(LightsSubsystem.white), lightsSubsystem)
                        .withTimeout(1.5));

        /* Set left joystick bindings */
        leftDriveController.getLeftTopLeft().onTrue(runOnce(swerveDriveSubsystem::zeroRotation, swerveDriveSubsystem));
        leftDriveController.getLeftTopRight().whileTrue(visionSubsystem.resetPoseWithApriltag());
        leftDriveController.nameLeftTopLeft("Reset Gyro Angle");
        leftDriveController.nameLeftTopRight("Reset Pose w/ AprilTag");

        leftDriveController.getTrigger().whileTrue(gripperSubsystem.openGripperCommand());
        rightDriveController.getTrigger().whileTrue(gripperSubsystem.ejectFromGripperCommand());
        leftDriveController.nameTrigger("Run Gripper");
        rightDriveController.nameTrigger("Eject Gripper");

        // Leveling
        leftDriveController.getLeftBottomLeft().toggleOnTrue(swerveDriveSubsystem.levelChargeStationCommandDestiny());

        new Trigger(() -> swerveDriveSubsystem.isLevelDestiny())
                .whileTrue(run(() -> LightsSubsystem.LEDSegment.MainStrip.setRainbowAnimation(1), lightsSubsystem));

        leftDriveController
                .getLeftBottomMiddle()
                .toggleOnTrue(run(swerveDriveSubsystem::lock, swerveDriveSubsystem)
                        .alongWith(run(
                                () -> LightsSubsystem.LEDSegment.MainStrip.setRainbowAnimation(1), lightsSubsystem)));
        leftDriveController.nameLeftBottomLeft("Level Charge Station");
        leftDriveController.nameLeftBottomMiddle("Lock Wheels");

        leftDriveController.getRightThumb().whileTrue(new IndicateGridAimedCommand(visionSubsystem, lightsSubsystem));
        leftDriveController.nameRightThumb("Aim to Grid");

        leftDriveController
                .getLeftThumb()
                .whileTrue(new IntakingAimAssistCommand(
                                visionSubsystem,
                                swerveDriveSubsystem,
                                lightsSubsystem,
                                this::getDriveForwardAxis,
                                this::getDriveStrafeAxis,
                                this::getDriveRotationAxis)
                        .alongWith(intakeSubsystem.intakeModeCommand()));
        leftDriveController.nameLeftThumb("ML Pickup");

        leftDriveController.getBottomThumb().whileTrue(gripperSubsystem.dropFromGripperCommand());
        leftDriveController.nameBottomThumb("Drop Game Piece");

        /* Set right joystick bindings */
        if (!Constants.competitionMode) {
            rightDriveController.getRightBottomMiddle().whileTrue(swerveDriveSubsystem.characterizeCommand(true, true));
            rightDriveController.getRightBottomRight().whileTrue(swerveDriveSubsystem.characterizeCommand(true, false));
            rightDriveController.nameRightBottomMiddle("Characterize Forwards");
            rightDriveController.nameRightBottomMiddle("Characterize Backwards");
        }

        // Will only need two triggers for this once we have a sensor
        rightDriveController.getLeftThumb().whileTrue(intakeSubsystem.intakeModeCommand());
        rightDriveController.getBottomThumb().whileTrue(gripperSubsystem.gripperShootHighCommand());
        rightDriveController.getRightThumb().whileTrue(intakeSubsystem.reverseIntakeModeCommand());
        rightDriveController.nameLeftThumb("Run Intake");
        rightDriveController.nameRightThumb("Reverse Intake");
        rightDriveController.nameBottomThumb("Shoot");

        // Cardinal drive commands (inverted since the arm is on the back of the robot)
        rightDriveController
                .getPOVUp()
                .whileTrue(swerveDriveSubsystem.cardinalCommand(
                        Rotation2d.fromDegrees(180), this::getDriveForwardAxis, this::getDriveStrafeAxis));
        rightDriveController
                .getPOVRight()
                .whileTrue(swerveDriveSubsystem.cardinalCommand(
                        Rotation2d.fromDegrees(90), this::getDriveForwardAxis, this::getDriveStrafeAxis));
        rightDriveController
                .getPOVDown()
                .whileTrue(swerveDriveSubsystem.cardinalCommand(
                        Rotation2d.fromDegrees(0), this::getDriveForwardAxis, this::getDriveStrafeAxis));
        rightDriveController
                .getPOVLeft()
                .whileTrue(swerveDriveSubsystem.cardinalCommand(
                        Rotation2d.fromDegrees(-90), this::getDriveForwardAxis, this::getDriveStrafeAxis));

        /* Set operator controller bindings */
        var shiftButton = operatorController.getRightBumper();

        var substationAimedCommand = new IndicateSubstationAimedCommand(visionSubsystem, lightsSubsystem);

        operatorController.getA().onTrue(armSubsystem.armStateCommand(ArmState.SHOOT_HYBRID));
        // operatorController.getY().onTrue(armSubsystem.armStateCommand(ArmState.SHOOT_HIGH));
        operatorController.getB().onTrue(armSubsystem.slidePickupCommand());
        operatorController.getB().whileTrue(substationAimedCommand);
        operatorController.getX().onTrue(armSubsystem.awaitingDeploymentCommand());
        operatorController.getX().whileTrue(substationAimedCommand);
        operatorController.getY().onTrue(armSubsystem.hoverCommand());
        operatorController.nameA("Shoot Hybrid");
        operatorController.nameY("Hover on High");
        operatorController.nameX("Slide Pickup");
        operatorController.nameB("Other Slide Pickup");

        // Pickup commands
        operatorController.getDPadDown().and(shiftButton.negate()).onTrue(armSubsystem.pickupCommand());
        operatorController.getDPadDown().and(shiftButton).onTrue(armSubsystem.tippedPickupCommand());

        operatorController.getDPadLeft().onTrue(armSubsystem.awaitingDeploymentCommand());

        // High commands
        // operatorController.getDPadUp().and(shiftButton.negate()).onTrue(armSubsystem.highManualConeCommand());
        operatorController
                .getDPadUp()
                .and(shiftButton.negate())
                .onTrue(armSubsystem
                        .armStateCommand(ArmState.HIGH_MANUAL_1)
                        .until(operatorController.getDPadUp().negate())
                        .andThen(waitUntil(operatorController.getDPadUp().negate())
                                .andThen(armSubsystem.armStateCommand(ArmState.HIGH_MANUAL_CONE))));

        // operatorController.getDPadUp().and(shiftButton).onTrue(armSubsystem.highManualCubeCommand());
        operatorController
                .getDPadUp()
                .and(shiftButton)
                .onTrue(armSubsystem
                        .armStateCommand(ArmState.HIGH_MANUAL_1)
                        .until(operatorController.getDPadUp().negate())
                        .andThen(waitUntil(operatorController.getDPadUp().negate())
                                .andThen(armSubsystem.armStateCommand(ArmState.HIGH_MANUAL_CUBE))));

        // Mid commands
        operatorController.getDPadRight().and(shiftButton.negate()).onTrue(armSubsystem.midManualConeCommand());
        operatorController.getDPadRight().and(shiftButton).onTrue(armSubsystem.midManualCubeCommand());

        operatorController.nameDPadDown("Pickup");
        operatorController.nameDPadLeft("Awaiting Deployment");
        operatorController.nameDPadUp("High Manual");
        operatorController.nameDPadRight("Mid Manual");

        operatorController
                .getLeftBumper()
                .onTrue(armSubsystem
                        .handoffCommand()
                        .deadlineWith(gripperSubsystem.dropFromGripperCommand())
                        .andThen(gripperSubsystem
                                .openGripperCommand()
                                .deadlineWith(intakeSubsystem
                                        .slowReverseIntakeModeCommand()
                                        .withTimeout(0.06)
                                        .andThen(waitSeconds(0.05).andThen(intakeSubsystem.handoffCommand()))))
                        .until(operatorController.getLeftBumper().negate())
                        .andThen(armSubsystem.undoHandoffCommand().asProxy()));

        operatorController.nameLeftBumper("Handoff Button");

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

        operatorController
                .getStart()
                .toggleOnTrue(run(() -> LightsSubsystem.LEDSegment.MainStrip.setRainbowAnimation(1), lightsSubsystem));
        operatorController.nameStart("Rainbow Mode");

        operatorController.getBack().whileTrue(intakeSubsystem.reverseIntakeModeCommand());
        operatorController.nameBack("Reverse Intake");

        // Send all button names to network tables
        rightDriveController.sendButtonNamesToNT();
        leftDriveController.sendButtonNamesToNT();
        operatorController.sendButtonNamesToNT();
    }

    public Command getAutonomousCommand() {
        return autonomousManager.getAutonomousCommand();
    }

    public double getDriveForwardAxis() {
        return -forwardRateLimiter.calculate(
                square(deadband(leftDriveController.getYAxis().getRaw(), 0.05)) * Constants.SwerveConstants.maxSpeed);
    }

    public double getDriveStrafeAxis() {
        return -strafeRateLimiter.calculate(
                square(deadband(leftDriveController.getXAxis().getRaw(), 0.05)) * Constants.SwerveConstants.maxSpeed);
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

    public Supplier<PlacementLocation> getTargetPoseSupplier() {
        return () -> {
            PlacementLocation targetLocation =
                    FieldConstants.getNearestPlacementLocation(swerveDriveSubsystem.getPose());

            Logger.log("/SwerveDriveSubsystem/TargetPose", targetLocation.robotPlacementPose);
            return targetLocation;
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
                    break;
                default: // HIGH or other
                    targetPose3d = targetLocation.getHighPose();
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
