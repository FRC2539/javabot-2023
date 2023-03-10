package frc.robot.commands;

import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.math.MathUtils;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.LimelightMode;

public class AssistedLLAimCommand extends CommandBase {
    private DoubleSupplier forward;
    private DoubleSupplier strafe;
    private DoubleSupplier rotate;

    private SwerveDriveSubsystem swerveDriveSubsystem;
    private VisionSubsystem visionSubsystem;

    private double maxAngularVelocity = 4;
    private double maxStrafeVelocity = 4;
    private TrapezoidProfile.Constraints pidConstraints = new TrapezoidProfile.Constraints(7, 7);
    private TrapezoidProfile.Constraints slowerPidConstraints = new TrapezoidProfile.Constraints(7, 4);
    private PIDController angleController = new PIDController(0.1, 0.0, 0.0);
    private PIDController strafeController = new PIDController(0.1, 0.0, 0.0);
    private TrapezoidProfile.State goalState = new TrapezoidProfile.State(0,0);

    private OptionalDouble lastRawEstimate;

    public AssistedLLAimCommand(SwerveDriveSubsystem swerveDriveSubsystem, VisionSubsystem visionSubsystem, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotate) {
        
        
        addRequirements(swerveDriveSubsystem, visionSubsystem);
        
        this.forward = forward;
        this.strafe = strafe;
        this.rotate = rotate;

        angleController.setSetpoint(0);
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        strafeController.setSetpoint(0);
    }

    public void initialize() {
        angleController.reset();
        strafeController.reset();
        visionSubsystem.setLimelightMode(LimelightMode.RETROREFLECTIVEHIGH);
        lastRawEstimate = OptionalDouble.empty();
    }

    public void execute() {
        if (visionSubsystem.hasLLFieldRelativeRetroflectiveEstimate()) {
            lastRawEstimate = OptionalDouble.of(visionSubsystem.getLimelightRawAngles().get().tx());
        }

        double strafeingValue = lastRawEstimate.isPresent() 
            ? MathUtils.ensureRange(strafeController.calculate(lastRawEstimate.getAsDouble()), -maxStrafeVelocity, maxStrafeVelocity)
            : strafe.getAsDouble();

        double angularSpeed = angleController.calculate(swerveDriveSubsystem.getPose().getRotation().getRadians());
        angularSpeed = MathUtils.ensureRange(angularSpeed, -maxAngularVelocity, maxAngularVelocity);

        swerveDriveSubsystem.setVelocity(new ChassisSpeeds(forward.getAsDouble(), strafeingValue, angularSpeed), true, true);
    }
}
