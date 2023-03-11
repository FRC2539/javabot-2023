package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.math.MathUtils;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.LimelightMode;

public class AssistedLLAimCommand extends CommandBase {
    private DoubleSupplier forward;
    private DoubleSupplier strafe;

    private SwerveDriveSubsystem swerveDriveSubsystem;
    private VisionSubsystem visionSubsystem;

    private double maxAngularVelocity = 4;
    private double maxStrafeVelocity = 4;
    private PIDController angleController = new PIDController(0.1, 0.0, 0.0);
    private PIDController strafeController = new PIDController(0.1, 0.0, 0.0);

    public AssistedLLAimCommand(SwerveDriveSubsystem swerveDriveSubsystem, VisionSubsystem visionSubsystem, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotate) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.visionSubsystem = visionSubsystem;
        
        addRequirements(swerveDriveSubsystem, visionSubsystem);
        
        this.forward = forward;
        this.strafe = strafe;

        angleController.setSetpoint(0);
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        strafeController.setSetpoint(0);
    }

    public void initialize() {
        angleController.reset();
        strafeController.reset();
        visionSubsystem.setLimelightMode(LimelightMode.RETROREFLECTIVEHIGH);
    }

    public void execute() {
        double strafeingValue;

        if (visionSubsystem.hasLLFieldRelativeRetroflectiveEstimate()) {
            double lastTx = visionSubsystem.getLimelightRawAngles().get().tx();
            strafeingValue = MathUtils.ensureRange(strafeController.calculate(lastTx), -maxStrafeVelocity, maxStrafeVelocity);
        } else {
            strafeingValue = strafe.getAsDouble();
        }

        double angularSpeed = angleController.calculate(swerveDriveSubsystem.getPose().getRotation().getRadians());
        angularSpeed = MathUtils.ensureRange(angularSpeed, -maxAngularVelocity, maxAngularVelocity);

        swerveDriveSubsystem.setVelocity(new ChassisSpeeds(forward.getAsDouble(), strafeingValue, angularSpeed), true, true);
    }
}
