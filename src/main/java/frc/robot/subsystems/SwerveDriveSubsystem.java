package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.gyro.GenericGyro;
import frc.lib.gyro.NavXGyro;
import frc.lib.gyro.PigeonGyro;
import frc.lib.interpolation.MovingAverageVelocity;
import frc.lib.logging.LoggedReceiver;
import frc.lib.logging.Logger;
import frc.lib.math.MathUtils;
import frc.lib.swerve.SwerveDriveSignal;
import frc.lib.swerve.SwerveModule;
import frc.robot.Constants;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.DoubleStream;

public class SwerveDriveSubsystem extends SubsystemBase {
    private final SwerveDrivePoseEstimator swervePoseEstimator;

    private Pose2d pose = new Pose2d();
    private final MovingAverageVelocity velocityEstimator = new MovingAverageVelocity(3);
    private ChassisSpeeds velocity = new ChassisSpeeds();
    private ChassisSpeeds previousVelocity = new ChassisSpeeds();
    private SwerveDriveSignal driveSignal = new SwerveDriveSignal();

    private LoggedReceiver pidValueReciever;

    private double levelingMaxSpeed;

    private boolean isLevelingAuto = false;

    private SwerveModule[] modules;

    private GenericGyro gyro;

    boolean isCharacterizing = false;

    private LoggedReceiver isSecondOrder;

    // PID controller used for auto-leveling
    private PIDController tiltController = new PIDController(0.75/15, 0, 0.02);

    // PID controller used for cardinal command
    private ProfiledPIDController omegaController =
            new ProfiledPIDController(5, 0, 0, new TrapezoidProfile.Constraints(8, 8));

    private double previousTilt = 0;
    private double tiltRate = 0;
    private DoubleSupplier maxSpeedSupplier = () -> Constants.SwerveConstants.maxSpeed;

    public SwerveDriveSubsystem() {
        if (SwerveConstants.hasPigeon)
            gyro = new PigeonGyro(SwerveConstants.PIGEON_PORT, GlobalConstants.CANIVORE_NAME);
        else gyro = new NavXGyro();

        modules = new SwerveModule[] {
            new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
            new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
            new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
            new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
        };

        // Reset each module using its absolute encoder to avoid having modules fail to align
        calibrateIntegratedEncoders();

        // Add all motors to orchestra lol
        for (SwerveModule module : modules) {
            RobotContainer.orchestra.addInstrument(module.getDriveMotor());
            RobotContainer.orchestra.addInstrument(module.getAngleMotor());
        }

        // Initialize the swerve drive pose estimator with access to the module positions.
        swervePoseEstimator = new SwerveDrivePoseEstimator(
                SwerveConstants.swerveKinematics,
                getGyroRotation(),
                getModulePositions(),
                new Pose2d(),
                VecBuilder.fill(0.01, 0.01, 0.01),
                VecBuilder.fill(0.5, 0.5, 0.5)); // might need to bring these back up

        // Allow us to toggle on second order kinematics
        isSecondOrder = Logger.tunable("/SwerveDriveSubsystem/isSecondOrder", true);
        pidValueReciever = Logger.tunable("/SwerveDriveSubsystem/levelPIDValues", new double[]{0.75/15, 0, .02, 8, 0.75}); //P I D stopAngle leveingMaxSpeed
    }

    public Command driveCommand(
            DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation, boolean isFieldOriented) {
        return run(() -> {
            setVelocity(
                    new ChassisSpeeds(forward.getAsDouble(), strafe.getAsDouble(), rotation.getAsDouble()),
                    isFieldOriented);
        });
    }

    public Command preciseDriveCommand(
            DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation, boolean isFieldOriented) {
        var speedMultiplier = SwerveConstants.preciseDrivingModeSpeedMultiplier;

        return run(() -> {
            setVelocity(
                    new ChassisSpeeds(
                            speedMultiplier * forward.getAsDouble(),
                            speedMultiplier * strafe.getAsDouble(),
                            speedMultiplier * rotation.getAsDouble()),
                    isFieldOriented);
        });
    }

    public Command cardinalCommand(Rotation2d targetAngle, DoubleSupplier forward, DoubleSupplier strafe) {
        omegaController.setTolerance(Units.degreesToRadians(1));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        return run(() -> {
            var rotationVelocity =
                    omegaController.calculate(pose.getRotation().getRadians(), targetAngle.getRadians());

            setVelocity(new ChassisSpeeds(forward.getAsDouble(), strafe.getAsDouble(), rotationVelocity), true);
        });
    }

    public Command orchestraCommand() {
        return startEnd(
                        () -> {
                            RobotContainer.orchestra.loadMusic("thunderstruck.chrp");

                            RobotContainer.orchestra.play();
                        },
                        () -> {
                            RobotContainer.orchestra.stop();
                        })
                .withName("Orchestra");
    }

    public Command levelChargeStationCommandArlene() {
        var constraints = new TrapezoidProfile.Constraints(0.4, 0.8);
        var tiltController = new ProfiledPIDController(0.27, 0, 0.01, constraints);

        // End with no pitch and stationary
        State goal = new State(0, 0);

        // Four degrees of tolerance
        tiltController.setTolerance(4, 0.1);

        return run(() -> {
                    double tilt = getTiltAmountInDegrees();

                    Logger.log("/SwerveDriveSubsystem/Tilt Rate", getTiltRate());

                    // Negative pitch -> drive forward, Positive pitch -> drive backward

                    Translation2d direction = new Translation2d(
                            getNormalVector3d().getX(), getNormalVector3d().getY());

                    Translation2d finalDirection = direction.times(tiltController.calculate(tilt, goal));

                    ChassisSpeeds velocity = new ChassisSpeeds(finalDirection.getX(), finalDirection.getY(), 0);

                    if (MathUtils.equalsWithinError(0, tilt, 8) || getTiltAmount() < -0.2) lock();
                    else setVelocity(velocity, false);
                })
                .repeatedly();
    }

    public Command levelChargeStationCommandDestiny() {
        return run(() -> {
                    double tilt = getTiltAmountInDegrees();

                    // TODO: stop when angle changes

                    // Negative pitch -> drive forward, Positive pitch -> drive backward

                    Translation2d direction = new Translation2d(1, new Rotation2d(
                            getNormalVector3d().getX(), getNormalVector3d().getY()));

                    double speed = tiltController.calculate(tilt, 0);
                    if (speed >= levelingMaxSpeed) speed = levelingMaxSpeed;

                    Translation2d finalDirection = direction.times(tiltController.calculate(tilt, 0));

                    ChassisSpeeds velocity = new ChassisSpeeds(finalDirection.getX(), finalDirection.getY(), 0);

                    if (tiltController.atSetpoint()) {
                        lock();
                        LightsSubsystem.LEDSegment.MainStrip.setRainbowAnimation(1);
                    }
                    else setVelocity(velocity, false);
                }).beforeStarting(() -> {
                    isLevelingAuto = true;
                    var values = pidValueReciever.getDoubleArray();
                    if (values.length < 5) return;
                    tiltController.setPID(values[0], values[1], values[2]);
                    tiltController.setTolerance(values[3]);
                    levelingMaxSpeed = values[4];
                })
                .finallyDo((interrupted) -> {tiltController.reset(); isLevelingAuto = false;});
    }

    public boolean isLevelDestiny() {
        return tiltController.atSetpoint() && isLevelingAuto;
    }

    public Command levelChargeStationCommandBrooklyn() {
        // 0.05 is the response time. this prevents the wheels from going all crazy
        Debouncer isLevelEnough = new Debouncer(0.05, Debouncer.DebounceType.kBoth);

        return run(() -> {
            double tilt = getTiltAmount();

            Translation2d finalDirection = new Translation2d(
                            getNormalVector3d().getX(), getNormalVector3d().getY())
                    .times(0.5); // 0.05 is the max speed

            if (isLevelEnough.calculate(tilt < Math.toRadians(6))) {
                lock();
            } else {
                setVelocity(new ChassisSpeeds(finalDirection.getX(), finalDirection.getY(), 0), false);
            }
        });
    }

    public Command levelChargeStationCommandCatherine() {
        // 0.05 is the response time. this prevents the wheels from going all crazy
        Debouncer isFastEnough = new Debouncer(0.05, Debouncer.DebounceType.kBoth);

        BooleanSupplier whenToStop = () -> {
            return isFastEnough.calculate(getTiltRate() < -15);
        };

        return runEnd(
                        () -> {
                            Translation2d finalDirection = new Translation2d(
                                            getNormalVector3d().getX(),
                                            getNormalVector3d().getY())
                                    .times(0.5); // 0.05 is the max speed

                            setVelocity(new ChassisSpeeds(finalDirection.getX(), finalDirection.getY(), 0), false);
                        },
                        () -> {
                            lock();
                        })
                .until(whenToStop);
    }

    public Command characterizeCommand(boolean forwards, boolean isDriveMotors) {
        Consumer<Double> voltageConsumer = isDriveMotors
                ? (Double voltage) -> {
                    for (SwerveModule module : modules) {
                        module.setDriveCharacterizationVoltage(voltage);
                    }
                }
                : (Double voltage) -> {
                    for (SwerveModule module : modules) {
                        module.setAngleCharacterizationVoltage(voltage);
                    }
                };

        Supplier<Double> velocitySupplier = isDriveMotors
                ? () -> {
                    return DoubleStream.of(
                                    modules[0].getState().speedMetersPerSecond,
                                    modules[1].getState().speedMetersPerSecond,
                                    modules[2].getState().speedMetersPerSecond,
                                    modules[3].getState().speedMetersPerSecond)
                            .average()
                            .getAsDouble();
                }
                : () -> {
                    return DoubleStream.of(
                                    modules[0].getAngularVelocity(),
                                    modules[1].getAngularVelocity(),
                                    modules[2].getAngularVelocity(),
                                    modules[3].getAngularVelocity())
                            .average()
                            .getAsDouble();
                };

        return new FeedForwardCharacterization(
                        this,
                        forwards,
                        new FeedForwardCharacterizationData("Swerve Drive"),
                        voltageConsumer,
                        velocitySupplier)
                .beforeStarting(() -> isCharacterizing = true)
                .finallyDo((boolean interrupted) -> isCharacterizing = false);
    }

    public void switchToBackupGyro() {
        gyro = new NavXGyro();
    }

    public void calibrateIntegratedEncoders() {
        // Reset each module using its absolute encoder
        for (SwerveModule module : modules) {
            module.resetToAbsolute();
        }
    }

    public void setCustomMaxSpeedSupplier(DoubleSupplier maxSpeedSupplier) {
        this.maxSpeedSupplier = maxSpeedSupplier;
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

    /**
     * @return The robot relative velocity of the drivetrain
     */
    public ChassisSpeeds getVelocity() {
        return velocity;
    }

    public ChassisSpeeds getAcceleration() {
        return new ChassisSpeeds(
                velocity.vxMetersPerSecond - previousVelocity.vxMetersPerSecond,
                velocity.vyMetersPerSecond - previousVelocity.vyMetersPerSecond,
                velocity.omegaRadiansPerSecond - previousVelocity.omegaRadiansPerSecond);
    }

    /**
     * @return The potentially field relative desired velocity of the drivetrain
     */
    public ChassisSpeeds getDesiredVelocity() {
        return (ChassisSpeeds) driveSignal;
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
        return gyro.getRotation2d();
    }

    public Rotation2d getRotation() {
        return pose.getRotation();
    }

    public Rotation3d getGyroRotation3d() {
        return gyro.getRotation3d();
    }

    public Translation3d getNormalVector3d() {
        return new Translation3d(0, 0, 1).rotateBy(getGyroRotation3d());
    }

    /**is in degrees*/
    public double getTiltAmountInDegrees() {
        return Math.toDegrees(getTiltAmount());
    }

    public double getTiltAmount() {
        return Math.acos(getNormalVector3d().getZ());
    }

    public double getTiltRate() {
        return tiltRate;
    }

    public Rotation2d getTiltDirection() {
        return new Rotation2d(getNormalVector3d().getX(), getNormalVector3d().getY());
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

    public void lock() {
        driveSignal = new SwerveDriveSignal(true);
    }

    public void update() {
        updateOdometry();

        if (isCharacterizing) return;

        updateModules(driveSignal);
    }

    private void updateOdometry() {
        SwerveModuleState[] moduleStates = getModuleStates();
        SwerveModulePosition[] modulePositions = getModulePositions();

        previousVelocity = velocity;
        velocity = Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(moduleStates);

        // velocityEstimator.add(velocity);

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

        SwerveModuleState[] moduleStates =
                Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(chassisVelocity);

        if (driveSignal.isLocked()) {
            // get X for stopping
            moduleStates = new SwerveModuleState[] {
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            };

            // Set the angle of each module only
            for (int i = 0; i < moduleStates.length; i++) {
                modules[i].setDesiredAngleOnly(moduleStates[i].angle, true);
            }
        } else {
            setModuleStates(moduleStates, isDriveSignalStopped(driveSignal) ? true : driveSignal.isOpenLoop());
            // setModuleStates(moduleStates, driveSignal.isOpenLoop());
        }
    }

    private void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeedSupplier.getAsDouble());

        for (SwerveModule module : modules) {
            module.setDesiredState(desiredStates[module.moduleNumber], isOpenLoop, isSecondOrder.getBoolean());
        }

        Logger.log("/SwerveDriveSubsystem/Wheel Setpoint", desiredStates[0].speedMetersPerSecond);
    }

    private boolean isDriveSignalStopped(SwerveDriveSignal driveSignal) {
        return driveSignal.vxMetersPerSecond == 0
                && driveSignal.vyMetersPerSecond == 0
                && driveSignal.omegaRadiansPerSecond == 0;
    }

    @Override
    public void periodic() {
        var startTimeMS = Timer.getFPGATimestamp() * 1000;

        var tilt = getTiltAmount();
        tiltRate = (tilt - previousTilt) / 0.02;
        previousTilt = tilt;

        // Comment out to play music
        update();

        Logger.log("/SwerveDriveSubsystem/Pose", pose);
        // Logger.log("/SwerveDriveSubsystem/Velocity", velocity);
        // Logger.log("/SwerveDriveSubsystem/Desired Velocity", (ChassisSpeeds) driveSignal);

        Logger.log("/SwerveDriveSubsystem/Velocity Magnitude", getVelocityMagnitude());

        Logger.log("/SwerveDriveSubsystem/Wheel Speed", modules[0].getState().speedMetersPerSecond);

        Logger.log("/SwerveDriveSubsystem/Pitch", getGyroRotation3d().getY());
        Logger.log("/SwerveDriveSubsystem/Roll", getGyroRotation3d().getX());
        Logger.log("/SwerveDriveSubsystem/Tilt", getTiltAmountInDegrees());

        // Logger.log("/SwerveDriveSubsystem/Wheel Angles", new double[] {
        //     modules[0].getPosition().angle.getDegrees(),
        //     modules[1].getPosition().angle.getDegrees(),
        //     modules[2].getPosition().angle.getDegrees(),
        //     modules[3].getPosition().angle.getDegrees()
        // });

        Logger.log("/SwerveDriveSubsystem/Drive Temperatures", getDriveTemperatures());
        // Logger.log("/SwerveDriveSubsystem/Angle Temperatures", getAngleTemperatures());

        Logger.log("/SwerveDriveSubsystem/LoopDuration", Timer.getFPGATimestamp() * 1000 - startTimeMS);
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
