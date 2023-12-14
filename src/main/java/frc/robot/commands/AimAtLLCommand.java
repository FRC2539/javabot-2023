package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.logging.LoggedReceiver;
import frc.lib.logging.Logger;
import frc.lib.math.MathUtils;
import frc.lib.vision.BackLimelight;
import frc.lib.vision.LimelightRawAngles;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class AimAtLLCommand extends CommandBase {
    private DoubleSupplier forward;
    private DoubleSupplier strafe;

    private SwerveDriveSubsystem swerveDriveSubsystem;
    private VisionSubsystem visionSubsystem;

    private static final TrapezoidProfile.Constraints angleConstraints = new TrapezoidProfile.Constraints(2, 4);
    private static final double maxTurningVelocity = 3;

    private ProfiledPIDController angleController = new ProfiledPIDController(1.5, 0.0, 0.0, angleConstraints);
    private PIDController strafeController = new PIDController(0.05, 0.0, 0.1);

    // this order is {kP rot, kI rot, kD rot, kP horiz, kI horiz, kD horiz, speed rot, accel rot}
    private double[] configValues = new double[] {6, 0.0, 0.0, 0.15, 0.0, 0.0, 4.0, 4.0};

    private BackLimelight camera;

    private LoggedReceiver pidValueReciever;
    private boolean isUsingNetPID;

    private double goal;

    public AimAtLLCommand(
            SwerveDriveSubsystem swerveDriveSubsystem,
            VisionSubsystem visionSubsystem,
            LightsSubsystem lightsSubsystem,
            DoubleSupplier forward,
            DoubleSupplier strafe,
            DoubleSupplier rotate,
            BooleanSupplier isLeftTarget,
            boolean isUsingNetPID) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.visionSubsystem = visionSubsystem;

        addRequirements(swerveDriveSubsystem, visionSubsystem, lightsSubsystem);

        this.forward = forward;
        this.strafe = strafe;

        angleController.setGoal(Math.PI);
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        strafeController.setSetpoint(0);

        camera = visionSubsystem.getBackLimelight();

        // this order is {kP rot, kI rot, kD rot, kP horiz, kI horiz, kD horiz, speed rot, accel rot}
        pidValueReciever = Logger.tunable("/LLAimCommand/pid_values", configValues);
        this.isUsingNetPID = isUsingNetPID;
    }

    @Override
    public void initialize() {
        double[] someValues = pidValueReciever.getDoubleArray();

        if (someValues.length != 8 || !isUsingNetPID) someValues = configValues;

        angleController = new ProfiledPIDController(
                someValues[0],
                someValues[1],
                someValues[2],
                new TrapezoidProfile.Constraints(someValues[6], someValues[7]));

        angleController.setGoal(0);
        angleController.enableContinuousInput(-Math.PI, Math.PI);
        angleController.setTolerance(Math.toRadians(3));

        strafeController.setPID(someValues[3], someValues[4], someValues[5]);
        strafeController.setTolerance(0.5);
        strafeController.setSetpoint(0);

        camera.setMode(BackLimelight.Mode.RETROREFLECTIVEALIGN);

        angleController.reset(swerveDriveSubsystem.getRotation().getRadians());
    }

    @Override
    public void execute() {
        // var allVisionAngles = visionSubsystem.getBackAllRetroreflectiveAngles();

        if (camera.hasLimelightRawAngles()) {

            LimelightRawAngles bestCurrentTarget =
                    camera.getLimelightRawAngles().get(); // allVisionAngles.get(0);
            // for (LimelightRawAngles i : allVisionAngles) {
            //     if (Math.abs(i.tx()) < Math.abs(bestCurrentTarget.tx())) {
            //         bestCurrentTarget = i;
            //     }
            // }
            if (DriverStation.isFMSAttached()) {
                NetworkTableInstance.getDefault()
                        .getTable("limelight")
                        .getEntry("snapshot")
                        .setNumber(1);
            }

            goal = swerveDriveSubsystem.getPose().getRotation().getRadians() + Math.toRadians(bestCurrentTarget.tx());
            angleController.setGoal(goal);
        }

        double angularCorrection = angleController.calculate(
                swerveDriveSubsystem.getPose().getRotation().getRadians());
        double angularSpeed = angleController.getSetpoint().velocity + angularCorrection;

        angularSpeed = MathUtils.ensureRange(angularSpeed, -maxTurningVelocity, maxTurningVelocity);

        swerveDriveSubsystem.setVelocity(
                new ChassisSpeeds(forward.getAsDouble(), strafe.getAsDouble(), angularSpeed), true, true);
    }

    @Override
    public void end(boolean interrupted) {
        if (DriverStation.isFMSAttached()) {
            NetworkTableInstance.getDefault()
                    .getTable("limelight")
                    .getEntry("snapshot")
                    .setNumber(0);
        }
    }
}