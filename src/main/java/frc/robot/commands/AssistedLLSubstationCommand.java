package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

public class AssistedLLSubstationCommand extends CommandBase {
    private DoubleSupplier forward;
    private DoubleSupplier strafe;

    private Debouncer isAimed = new Debouncer(0.2, DebounceType.kFalling);
    private SlewRateLimiter forwardSlewer = new SlewRateLimiter(3);

    private SwerveDriveSubsystem swerveDriveSubsystem;
    private VisionSubsystem visionSubsystem;

    private boolean hasSeenGoal = false;
    private boolean atAngle = false;
    private double forwardValue = 0;

    private static final TrapezoidProfile.Constraints angleConstraints = new TrapezoidProfile.Constraints(4, 4);
    private static final double maxForwardVelocity = 2;

    private ProfiledPIDController angleController = new ProfiledPIDController(1.5, 0.0, 0.0, angleConstraints);
    private PIDController forwardController = new PIDController(0.05, 0.0, 0.1);

    //this order is {kP rot, kI rot, kD rot, kP horiz, kI horiz, kD horiz, speed rot, accel rot}
    private double[] configValues = new double[]{6,0.0,0.0,0.15,0.0,0.0,4.0,4.0};

    private BackLimelight camera;

    private LoggedReceiver pidValueReciever;
    private boolean isUsingNetPID;

    private double howCloseGotten;

    public AssistedLLSubstationCommand(
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

        forwardController.setSetpoint(DriverStation.getAlliance() == Alliance.Blue ? -Math.PI / 2: Math.PI / 2);

        camera = visionSubsystem.getBackLimelight();
    
        //this order is {kP rot, kI rot, kD rot, kP horiz, kI horiz, kD horiz, speed rot, accel rot}
        pidValueReciever = Logger.tunable("/LLAimCommand/pid_values_two", configValues);
        this.isUsingNetPID = isUsingNetPID;
    }

    @Override
    public void initialize() { 
        double[] someValues = pidValueReciever.getDoubleArray();

        if (someValues.length != 8 || !isUsingNetPID) someValues = configValues;

        angleController = new ProfiledPIDController(someValues[0],someValues[1],someValues[2], new TrapezoidProfile.Constraints(someValues[6], someValues[7]));

        angleController.setGoal(DriverStation.getAlliance() == DriverStation.Alliance.Blue ? -Math.PI / 2 : Math.PI / 2);
        angleController.enableContinuousInput(-Math.PI, Math.PI);
        angleController.setTolerance(Math.toRadians(3));

        forwardController.setPID(someValues[3], someValues[4], someValues[5]);
        forwardController.setTolerance(0.5);
        forwardController.setSetpoint(0);

        camera.setMode(BackLimelight.Mode.CONE);

        angleController.reset(swerveDriveSubsystem.getRotation().getRadians());

        hasSeenGoal = false;
        atAngle = false;
        forwardValue = 0;

        forwardSlewer.reset(0);
        howCloseGotten = 40;
    }

    @Override
    public void execute() {
        // var allVisionAngles = visionSubsystem.getBackAllRetroreflectiveAngles();

        if (camera.hasLimelightRawAngles()
                && atAngle) {

            LimelightRawAngles bestCurrentTarget =
                    camera.getLimelightRawAngles().get(); // allVisionAngles.get(0);
            // for (LimelightRawAngles i : allVisionAngles) {
            //     if (Math.abs(i.tx()) < Math.abs(bestCurrentTarget.tx())) {
            //         bestCurrentTarget = i;
            //     }
            // }

            if (Math.abs(bestCurrentTarget.tx()) < howCloseGotten + 10) {
                Logger.log("/LLAimCommand/tx", bestCurrentTarget.tx());

                forwardValue = -MathUtils.ensureRange(
                        deadband(forwardSlewer.calculate(forwardController.calculate(bestCurrentTarget.tx()))), -maxForwardVelocity, maxForwardVelocity);

                if (forwardController.atSetpoint()) forwardValue = 0;
                
                if (Math.abs(forwardController.getPositionError()) <= 2.0) {
                    LightsSubsystem.LEDSegment.MainStrip.setColor(LightsSubsystem.green);
                } else {
                    LightsSubsystem.LEDSegment.MainStrip.setColor(LightsSubsystem.yellow);
                }

                hasSeenGoal = true;
                howCloseGotten = Math.min(Math.abs(forwardController.getPositionError()), howCloseGotten);
            }
        } else if (!hasSeenGoal) {
            forwardValue = forward.getAsDouble();
            forwardSlewer.reset(forwardValue);
            LightsSubsystem.LEDSegment.MainStrip.setColor(LightsSubsystem.blue);
        }

        double angularCorrection = angleController.calculate(
                swerveDriveSubsystem.getPose().getRotation().getRadians());
        double angularSpeed = angleController.getSetpoint().velocity + angularCorrection;
        if (angleController.atGoal()) {
            //angularSpeed = 0;
            atAngle = true;
        }

        swerveDriveSubsystem.setVelocity(
                new ChassisSpeeds(forwardValue, strafe.getAsDouble(), angularSpeed), true, true);
    }

    private static double deadband(double value) {
        if (Math.abs(value) < 0.075 * maxForwardVelocity) return Math.copySign(0.075 * maxForwardVelocity, value);

        return value;
    }
}
