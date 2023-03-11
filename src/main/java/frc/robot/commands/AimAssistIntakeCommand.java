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
import frc.robot.subsystems.LightsSubsystem.Color;
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

    private LimelightRawAngles lastSeenLLAngles = new LimelightRawAngles(0,0);

    private boolean sawGamePieceSoFar;

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
    }

    @Override
    public void execute() {
        if (visionSubsystem.hasLLMLFieldPoseEstimate()) {
            LightsSubsystem.LEDSegment.MainStrip.setColor(LightsSubsystem.green);
            lastSeenLLAngles = visionSubsystem.getLimelightRawAngles().get();
            sawGamePieceSoFar = true;
        }

        double rotationAngle = lastSeenLLAngles.tx();
        if (isDriverGoingForBall() && sawGamePieceSoFar) {
            swerveDriveSubsystem.setVelocity(
                    new ChassisSpeeds(
                            // NOTE: RE ADD - SIGNS
                            getDriverValueTowardsBall() * Math.cos(rotationAngle) * speedModifier,
                            getDriverValueTowardsBall() * Math.sin(rotationAngle) * speedModifier,
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
        //     balltrackSubsystem.intakeMode();
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

        if (Math.abs(lastSeenLLAngles.tx()) < Math.toRadians(30)) {
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

    // private boolean shouldIntake() {
    //     if (!visionSubsystem.getBallDistance().isPresent()) return false;

    //     if (visionSubsystem.getBallDistance().orElse(2) / getVelocity()
    //             < (INTAKE_DOWN_DISTANCE * INTAKE_FACTOR)) {
    //         return true;
    //     }

    //     if (visionSubsystem.getBallDistance().orElse(2) < 0.6) return true;

    //     return false;
    // }
}