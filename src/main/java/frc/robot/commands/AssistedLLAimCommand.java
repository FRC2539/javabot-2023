package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.LimelightMode;
import java.util.function.DoubleSupplier;

public class AssistedLLAimCommand extends CommandBase {
    private DoubleSupplier forward;
    private DoubleSupplier strafe;

    private SwerveDriveSubsystem swerveDriveSubsystem;
    private VisionSubsystem visionSubsystem;

    private static final TrapezoidProfile.Constraints angleConstraints = new TrapezoidProfile.Constraints(4, 4);
    private static final TrapezoidProfile.Constraints strafeConstraints = new TrapezoidProfile.Constraints(3, 3);

    private ProfiledPIDController angleController = new ProfiledPIDController(0.5, 0.0, 0.0, angleConstraints);
    private ProfiledPIDController strafeController = new ProfiledPIDController(0.5, 0.0, 0.0, strafeConstraints);

    public AssistedLLAimCommand(
            SwerveDriveSubsystem swerveDriveSubsystem,
            VisionSubsystem visionSubsystem,
            DoubleSupplier forward,
            DoubleSupplier strafe,
            DoubleSupplier rotate) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.visionSubsystem = visionSubsystem;

        addRequirements(swerveDriveSubsystem, visionSubsystem);

        this.forward = forward;
        this.strafe = strafe;

        angleController.setGoal(0);
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        strafeController.setGoal(0);
    }

    @Override
    public void initialize() {
        visionSubsystem.setBackLimelightMode(LimelightMode.RETROREFLECTIVEHIGH);

        angleController.reset(swerveDriveSubsystem.getRotation().getRadians());
    }

    @Override
    public void execute() {
        double strafeingValue;

        if (visionSubsystem.hasBackRetroreflectiveAngles() && visionSubsystem.isBackLimelightAtPipeline()) {
            double lastTx = visionSubsystem.getBackRetroreflectiveAngles().get().tx();

            double setpointCorrection = strafeController.calculate(lastTx);

            strafeingValue = -(strafeController.getSetpoint().velocity + setpointCorrection);
        } else {
            strafeingValue = strafe.getAsDouble();
        }

        double angularCorrection = angleController.calculate(
                swerveDriveSubsystem.getPose().getRotation().getRadians());
        double angularSpeed = angleController.getSetpoint().velocity + angularCorrection;

        swerveDriveSubsystem.setVelocity(
                new ChassisSpeeds(forward.getAsDouble(), strafeingValue, angularSpeed), true, true);
    }
}
