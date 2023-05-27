package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.logging.Logger;
import frc.lib.math.MathUtils;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.LimelightMode;
import frc.robot.subsystems.VisionSubsystem.LimelightRawAngles;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class AssistedLLAimCommand extends CommandBase {
    private DoubleSupplier forward;
    private DoubleSupplier strafe;

    private SwerveDriveSubsystem swerveDriveSubsystem;
    private VisionSubsystem visionSubsystem;

    private boolean hasSeenGoal = false;
    private double strafeingValue = 0;

    private LinearFilter txRollingAverage = LinearFilter.movingAverage(5);

    private static final TrapezoidProfile.Constraints angleConstraints = new TrapezoidProfile.Constraints(4, 4);
    private static final double maxStrafeVelocity = 2;

    private ProfiledPIDController angleController = new ProfiledPIDController(1.5, 0.0, 0.0, angleConstraints);
    private PIDController strafeController = new PIDController(0.05, 0.0, 0.1);

    public AssistedLLAimCommand(
            SwerveDriveSubsystem swerveDriveSubsystem,
            VisionSubsystem visionSubsystem,
            DoubleSupplier forward,
            DoubleSupplier strafe,
            DoubleSupplier rotate,
            BooleanSupplier isLeftTarget) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.visionSubsystem = visionSubsystem;

        addRequirements(swerveDriveSubsystem, visionSubsystem);

        this.forward = forward;
        this.strafe = strafe;

        angleController.setGoal(0);
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        strafeController.setSetpoint(0);
        strafeController.setTolerance(1.3);
    }

    @Override
    public void initialize() {
        visionSubsystem.setBackLimelightMode(LimelightMode.RETROREFLECTIVEHIGH);

        angleController.reset(swerveDriveSubsystem.getRotation().getRadians());
    }

    @Override
    public void execute() {
        // var allVisionAngles = visionSubsystem.getBackAllRetroreflectiveAngles();

        if (visionSubsystem.backLimelightHasTarget() && visionSubsystem.isBackLimelightAtPipeline() && angleController.atGoal()) {
            
            LimelightRawAngles bestCurrentTarget = visionSubsystem.getBackRetroreflectiveAngles().get();//allVisionAngles.get(0);
            // for (LimelightRawAngles i : allVisionAngles) {
            //     if (Math.abs(i.tx()) < Math.abs(bestCurrentTarget.tx())) {
            //         bestCurrentTarget = i;
            //     }
            // }

            Logger.log("/LLAimCommand/tx", bestCurrentTarget.tx());

            strafeingValue = -MathUtils.ensureRange(
                    strafeController.calculate(bestCurrentTarget.tx()),
                    -maxStrafeVelocity,
                    maxStrafeVelocity);

            if (strafeController.atSetpoint()) strafeingValue = 0;

            hasSeenGoal = true;
        } else if (!hasSeenGoal) {
            strafeingValue = strafe.getAsDouble();
        }

        double angularCorrection = angleController.calculate(
                swerveDriveSubsystem.getPose().getRotation().getRadians());
        double angularSpeed = angleController.getSetpoint().velocity + angularCorrection;

        swerveDriveSubsystem.setVelocity(
                new ChassisSpeeds(forward.getAsDouble(), strafeingValue, angularSpeed), true, true);
    }
}
