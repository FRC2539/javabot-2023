package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.controller.Axis;
import frc.lib.interpolation.MovingAverageVelocity;
import frc.lib.logging.LoggableChassisSpeeds;
import frc.lib.logging.LoggableDouble;
import frc.lib.logging.LoggableDoubleArray;
import frc.lib.logging.LoggablePose;
import frc.lib.loops.Updatable;
import frc.lib.swerve.SwerveDriveSignal;
import frc.lib.swerve.SwerveModule;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TimesliceConstants;

public class SwerveDriveSubsystem extends SubsystemBase implements Updatable {
    private final SwerveDrivePoseEstimator swervePoseEstimator;

    private Pose2d pose = new Pose2d();
    private final MovingAverageVelocity velocityEstimator = new MovingAverageVelocity(20);
    private ChassisSpeeds velocity = new ChassisSpeeds();
    private SwerveDriveSignal driveSignal = new SwerveDriveSignal();

    private SwerveModule[] modules;

    private final Pigeon2 gyro = new Pigeon2(SwerveConstants.PIGEON_PORT);

    // Create a PID controller and heading tracker for maintaining desired heading
    private final PIDController headingController = new PIDController(0.3, 0, 0, TimesliceConstants.CONTROLLER_PERIOD);
    private Rotation2d desiredHeading = null;
    private SwerveDriveSignal previousDriveSignal = new SwerveDriveSignal();

    LoggableDouble gyroLogger = new LoggableDouble("/SwerveDriveSubsystem/Gyro");
    LoggablePose poseLogger = new LoggablePose("/SwerveDriveSubsystem/Pose", pose, true);
    LoggableChassisSpeeds velocityLogger = new LoggableChassisSpeeds("/SwerveDriveSubsystem/Velocity");
    LoggableDoubleArray desiredVelocityLogger = new LoggableDoubleArray("/SwerveDriveSubsystem/Desired Velocity");
    LoggableDoubleArray wheelAnglesLogger = new LoggableDoubleArray("/SwerveDriveSubsystem/Wheel Angles");
    LoggableDoubleArray driveTemperatureLogger = new LoggableDoubleArray("/SwerveDriveSubsystem/Drive Temperatures");
    LoggableDoubleArray angleTemperatureLogger = new LoggableDoubleArray("/SwerveDriveSubsystem/Angle Temperatures");
    LoggableDoubleArray driveVoltageLogger = new LoggableDoubleArray("/SwerveDriveSubsystem/Drive Voltage");
    LoggableDoubleArray angleVoltageLogger = new LoggableDoubleArray("/SwerveDriveSubsystem/Angle Voltage");

    public SwerveDriveSubsystem() {
        modules = new SwerveModule[] {
            new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
            new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
            new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
            new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
        };

        // Reset each module using its absolute encoder to avoid having modules fail to align
        for (SwerveModule module : modules) {
            module.resetToAbsolute();
        }

        // Initialize the swerve drive pose estimator with access to the module positions.
        swervePoseEstimator = new SwerveDrivePoseEstimator(
                SwerveConstants.swerveKinematics, getGyroRotation(), getModulePositions(), new Pose2d());
    }

    public CommandBase driveCommand(Axis forward, Axis strafe, Axis rotation, boolean isFieldOriented) {
        return runEnd(
                        () -> setVelocity(
                                new ChassisSpeeds(forward.get(true), strafe.get(true), rotation.get(true)),
                                isFieldOriented),
                        this::stop)
                .withName("Drive");
    }

    public Pose2d getPose() {
        return pose;
    }

    public void setPose(Pose2d pose) {
        this.pose = pose;
        swervePoseEstimator.resetPosition(getGyroRotation(), getModulePositions(), pose);
    }

    public void addVisionPoseEstimate(Pose2d pose, double timestamp) {
        swervePoseEstimator.addVisionMeasurement(pose, timestamp);
    }

    public ChassisSpeeds getVelocity() {
        return velocity;
    }

    public double getVelocityMagnitude() {
        return Math.sqrt(Math.pow(velocity.vxMetersPerSecond, 2) + Math.pow(velocity.vyMetersPerSecond, 2));
    }

    public Rotation2d getVelocityRotation() {
        return (new Translation2d(velocity.vxMetersPerSecond, velocity.vxMetersPerSecond)).getAngle();
    }

    public ChassisSpeeds getSmoothedVelocity() {
        return velocityEstimator.getAverage();
    }

    public Rotation2d getGyroRotation() {
        return Rotation2d.fromDegrees(gyro.getYaw());
    }

    public Rotation2d getRotation() {
        return pose.getRotation();
    }

    public void setRotation(Rotation2d angle) {
        setPose(new Pose2d(getPose().getX(), getPose().getY(), angle));
    }

    public void zeroRotation() {
        setRotation(new Rotation2d());
    }

    public void setVelocity(ChassisSpeeds velocity, boolean isFieldOriented, boolean isOpenLoop) {
        driveSignal = new SwerveDriveSignal(velocity, isFieldOriented, isOpenLoop);
    }

    public void setVelocity(ChassisSpeeds velocity, boolean isFieldOriented) {
        setVelocity(velocity, isFieldOriented, true);
    }

    public void setVelocity(ChassisSpeeds velocity) {
        setVelocity(velocity, false);
    }

    public void stop() {
        driveSignal = new SwerveDriveSignal();
    }

    @Override
    public void update() {
        updateOdometry();

        updateModules(driveSignal);
    }

    private void updateOdometry() {
        SwerveModuleState[] moduleStates = getModuleStates();
        SwerveModulePosition[] modulePositions = getModulePositions();

        velocity = Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(moduleStates);

        velocityEstimator.add(velocity);

        pose = swervePoseEstimator.update(getGyroRotation(), modulePositions);
    }

    private void updateModules(SwerveDriveSignal driveSignal) {
        ChassisSpeeds chassisVelocity;

        if (driveSignal.isFieldOriented()) {
            chassisVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(
                    driveSignal.vxMetersPerSecond,
                    driveSignal.vyMetersPerSecond,
                    driveSignal.omegaRadiansPerSecond,
                    getRotation());
        } else {
            chassisVelocity = (ChassisSpeeds) driveSignal;
        }

        // Store the current heading as desired if we are no longer turning
        if (driveSignal.omegaRadiansPerSecond == 0 && previousDriveSignal.omegaRadiansPerSecond != 0) {
            desiredHeading = getGyroRotation();
        } else if (driveSignal.omegaRadiansPerSecond != 0) {
            desiredHeading = null;
        }

        // Use a PID controller to maintain the current heading (as long as we are in teleop)
        if (desiredHeading != null && driveSignal.isOpenLoop()) {
            chassisVelocity.omegaRadiansPerSecond =
                    -headingController.calculate(getGyroRotation().getDegrees(), desiredHeading.getDegrees());
        }

        SwerveModuleState[] moduleStates =
                Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(chassisVelocity);

        setModuleStates(moduleStates, isDriveSignalStopped(driveSignal) ? true : driveSignal.isOpenLoop());

        // Store the current drive signal for the next loop
        previousDriveSignal = driveSignal;
    }

    private void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);

        for (SwerveModule module : modules) {
            module.setDesiredState(desiredStates[module.moduleNumber], isOpenLoop);
        }
    }

    private boolean isDriveSignalStopped(SwerveDriveSignal driveSignal) {
        return driveSignal.vxMetersPerSecond == 0
                && driveSignal.vyMetersPerSecond == 0
                && driveSignal.omegaRadiansPerSecond == 0;
    }

    @Override
    public void periodic() {
        gyroLogger.set(getGyroRotation().getDegrees());

        poseLogger.set(pose);
        velocityLogger.set(velocity);
        desiredVelocityLogger.set(new double[] {
            driveSignal.vxMetersPerSecond, driveSignal.vyMetersPerSecond, driveSignal.omegaRadiansPerSecond
        });

        wheelAnglesLogger.set(new double[] {
            modules[0].getPosition().angle.getDegrees(),
            modules[1].getPosition().angle.getDegrees(),
            modules[2].getPosition().angle.getDegrees(),
            modules[3].getPosition().angle.getDegrees()
        });

        driveTemperatureLogger.set(getDriveTemperatures());
        angleTemperatureLogger.set(getAngleTemperatures());
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule module : modules) {
            states[module.moduleNumber] = module.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule module : modules) {
            positions[module.moduleNumber] = module.getPosition();
        }
        return positions;
    }

    public double[] getDriveTemperatures() {
        return new double[] {
            modules[0].getDriveTemperature(),
            modules[1].getDriveTemperature(),
            modules[2].getDriveTemperature(),
            modules[3].getDriveTemperature()
        };
    }

    public double[] getAngleTemperatures() {
        return new double[] {
            modules[0].getAngleTemperature(),
            modules[1].getAngleTemperature(),
            modules[2].getAngleTemperature(),
            modules[3].getAngleTemperature()
        };
    }
}
