package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.LimelightMode;
import frc.robot.subsystems.VisionSubsystem.LimelightRawAngles;

public class AssistedMLAimCommand extends CommandBase {
    private DoubleSupplier forward;
    private DoubleSupplier strafe;
    private DoubleSupplier rotate;

    private SwerveDriveSubsystem swerveDriveSubsystem;
    private VisionSubsystem visionSubsystem;

    private TrapezoidProfile.Constraints pidConstraints = new TrapezoidProfile.Constraints(7, 7);
    private ProfiledPIDController pidController = new ProfiledPIDController(0.1, 0.0, 0.0, pidConstraints);
    private TrapezoidProfile.State goalState = new TrapezoidProfile.State(0,0);
    private final double lowestAllowedY = -50; //just guessed right now
    private boolean stopAcceptingNewPoses = false;

    private Optional<LimelightRawAngles> lastRawAngles;

    public AssistedMLAimCommand(SwerveDriveSubsystem swerveDriveSubsystem, VisionSubsystem visionSubsystem, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotate) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.visionSubsystem = visionSubsystem;
        
        addRequirements(swerveDriveSubsystem, visionSubsystem);
        
        this.forward = forward;
        this.strafe = strafe;
        this.rotate = rotate;

        pidController.setGoal(goalState);
    }

    @Override
    public void initialize() {
        pidController.reset(0);
        visionSubsystem.setLimelightMode(LimelightMode.ML);

        lastRawAngles = Optional.empty();
    }

    @Override
    public void execute() {
        if (visionSubsystem.hasLLMLFieldPoseEstimate() && !stopAcceptingNewPoses) {
            lastRawAngles = visionSubsystem.getLimelightRawAngles();
        }

        double desiredRotation;

        if (lastRawAngles.isPresent()) {
            if (lastRawAngles.get().ty() < lowestAllowedY) {
                double pidCorrection = pidController.calculate(
                    lastRawAngles.get().tx(),
                    goalState,
                    pidConstraints
                );
                desiredRotation = pidController.getSetpoint().velocity + pidCorrection;
            } else {
                desiredRotation = 0;
                stopAcceptingNewPoses = true;
            }
        } else {
            desiredRotation = rotate.getAsDouble();
        }
        swerveDriveSubsystem.setVelocity(new ChassisSpeeds(forward.getAsDouble(), strafe.getAsDouble(), desiredRotation), true, true);
    }
}
