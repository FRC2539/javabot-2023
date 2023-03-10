package frc.robot.commands;

import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.LimelightMode;

public class AssistedMLAimCommand extends CommandBase {
    private DoubleSupplier forward;
    private DoubleSupplier strafe;
    private DoubleSupplier rotate;

    private SwerveDriveSubsystem swerveDriveSubsystem;
    private VisionSubsystem visionSubsystem;

    private TrapezoidProfile.Constraints pidConstraints = new TrapezoidProfile.Constraints(7, 7);
    private TrapezoidProfile.Constraints slowerPidConstraints = new TrapezoidProfile.Constraints(7, 4);
    private ProfiledPIDController pidController = new ProfiledPIDController(0.1, 0.0, 0.0, pidConstraints);
    private TrapezoidProfile.State goalState = new TrapezoidProfile.State(0,0);

    private OptionalDouble lastRawEstimate;

    public AssistedMLAimCommand(SwerveDriveSubsystem swerveDriveSubsystem, VisionSubsystem visionSubsystem, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotate) {
        
        
        addRequirements(swerveDriveSubsystem, visionSubsystem);
        
        this.forward = forward;
        this.strafe = strafe;
        this.rotate = rotate;

        pidController.setGoal(goalState);
    }

    public void initialize() {
        pidController.reset(0);
        visionSubsystem.setLimelightMode(LimelightMode.ML);

        lastRawEstimate = OptionalDouble.empty();
    }

    public void execute() {
        if (visionSubsystem.hasLLMLFieldPoseEstimate()) {
            lastRawEstimate = OptionalDouble.of(visionSubsystem.getLimelightRawAngles().get().tx());
        }

        double desiredRotation;

        if (lastRawEstimate.isPresent()) {
            double pidCorrection = pidController.calculate(
                lastRawEstimate.getAsDouble(),
                goalState,
                pidConstraints
            );
            desiredRotation = pidController.getSetpoint().velocity + pidCorrection;
           
        } else {
            desiredRotation = rotate.getAsDouble();
        }
        swerveDriveSubsystem.setVelocity(new ChassisSpeeds(forward.getAsDouble(), strafe.getAsDouble(), desiredRotation), true, true);
    }
}
