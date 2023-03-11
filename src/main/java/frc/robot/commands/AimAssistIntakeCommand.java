package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.controller.Axis;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.LimelightRawAngles;

public class AimAssistIntakeCommand extends CommandBase {
    private VisionSubsystem visionSubsystem;
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private LightsSubsystem lightsSubsystem;

    private DoubleSupplier forward;
    private DoubleSupplier strafe;
    private DoubleSupplier rotate;

    private double speedModifier;

    private static final double INTAKE_FACTOR = 0.85;

    private static final double INTAKE_DOWN_DISTANCE = 1.2;

    private static final double RELIABILITY_MINIMUM_ANGLE = -20;

    private LimelightRawAngles lastSeenLLAngles = new LimelightRawAngles(0,0);

    private boolean sawGamePieceSoFar;

    private boolean hasGamePieceGottenTooClose;

    private ProfiledPIDController rotationControl = new ProfiledPIDController(
            1,
            0,
            0,
            new TrapezoidProfile.Constraints(
                    SwerveConstants.maxAngularVelocity / 2, SwerveConstants.maxAngularVelocity / 4));

    public AimAssistIntakeCommand(
            VisionSubsystem visionSubsystem,
            SwerveDriveSubsystem swerveDriveSubsystem,
            LightsSubsystem lightsSubsystem,
            Axis forward,
            Axis strafe,
            Axis rotate) {
        this.visionSubsystem = visionSubsystem;
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.lightsSubsystem = lightsSubsystem;

        addRequirements(visionSubsystem, swerveDriveSubsystem, lightsSubsystem);

        this.forward = forward;
        this.strafe = strafe;
        this.rotate = rotate;

        rotationControl.setGoal(0);
        rotationControl.setTolerance(Math.toRadians(5));
    }

    @Override
    public void initialize() {
        speedModifier = 1;
        sawGamePieceSoFar = false;
        hasGamePieceGottenTooClose = false;
    }

    @Override
    public void execute() {
        if (visionSubsystem.hasLLMLFieldPoseEstimate()) {
            LightsSubsystem.LEDSegment.MainStrip.setColor(LightsSubsystem.green);
            lastSeenLLAngles = visionSubsystem.getLimelightRawAngles().get();
            sawGamePieceSoFar = true;
            if (lastSeenLLAngles.ty() < RELIABILITY_MINIMUM_ANGLE) {
                hasGamePieceGottenTooClose = true;
            }
        }

        double rotationAngle = Math.toRadians(lastSeenLLAngles.tx());

        if (hasGamePieceGottenTooClose) {
            rotationAngle = 0;
        }

        if (isDriverGoingForBall() && sawGamePieceSoFar) {
            swerveDriveSubsystem.setVelocity(
                    new ChassisSpeeds(
                            -getDriverValueTowardsBall() * Math.cos(rotationAngle) * speedModifier,
                            -getDriverValueTowardsBall() * Math.sin(rotationAngle) * speedModifier,
                            rotationControl.calculate(rotationAngle)),
                    false, true);
        } else {
            swerveDriveSubsystem.setVelocity
             (
                new ChassisSpeeds(
                        forward.getAsDouble() * speedModifier,
                        strafe.getAsDouble() * speedModifier,
                        rotate.getAsDouble() * speedModifier),
            true, true);
        }
        // if (shouldIntake()) {
        //     speedModifier = INTAKE_FACTOR;
        // }
    }

    @Override
    public void end(boolean interrupted) {

    }

    private double getVelocity() {
        ChassisSpeeds chassisSpeeds = swerveDriveSubsystem.getVelocity();
        return Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    }

    private boolean isDriverGoingForBall() {
        // if (Math.abs(getDriverStrafeFromBall()) < 0.5) return false;

        if (Math.abs(lastSeenLLAngles.tx()) < 30) {
            return true;
        } else {
            return false;
        }
    }

    private double getDriverValueTowardsBall() {
        double ballAngle = lastSeenLLAngles.tx()
                + swerveDriveSubsystem.getGyroRotation().getRadians();
        double towardsBall = -forward.getAsDouble() * Math.cos(ballAngle) + -strafe.getAsDouble() * Math.sin(ballAngle);
        return towardsBall;
    }

    private double getDriverStrafeFromBall() {
        double ballAngle = lastSeenLLAngles.tx()
                + swerveDriveSubsystem.getGyroRotation().getRadians();
        double strafeBall = forward.getAsDouble() * Math.sin(ballAngle) + -strafe.getAsDouble() * Math.cos(ballAngle);
        return strafeBall;
    }

    private double getDistanceFromBall() {
        return (0.6 /*height of ll*/ - .12 /*height of mid of cube*/) / Math.tan(Math.toRadians(lastSeenLLAngles.ty() + -20 /*angle of camera*/));
    }

    private boolean shouldIntake() {
        if (!visionSubsystem.hasLLMLFieldPoseEstimate()) return false;

        if (getDistanceFromBall() / getVelocity()
                < (INTAKE_DOWN_DISTANCE * INTAKE_FACTOR)) {
            return true;
        }

        if (getDistanceFromBall() < 0.6) return true;

        return false;
    }
}