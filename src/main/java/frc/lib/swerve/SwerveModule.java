package frc.lib.swerve;


import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Robot;

public class SwerveModule {
    public int moduleNumber;
    private double zeroedWheelCANCoderAngle;
    private TalonFX angleMotor;
    private TalonFX driveMotor;
    private CANcoder angleEncoder;
    private double lastAngle;

    private VoltageOut voltageRequestDrive = new VoltageOut(0).withEnableFOC(true);

    private PositionVoltage positionVoltageRequestAngle = new PositionVoltage(0).withSlot(0).withEnableFOC(true);
    private VelocityVoltage velocityVoltageRequestDrive = new VelocityVoltage(0).withSlot(0).withEnableFOC(true);

    SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(
            Constants.SwerveConstants.calculatedDriveKS,
            Constants.SwerveConstants.calculatedDriveKV,
            Constants.SwerveConstants.calculatedDriveKA);

    // Testing a calculation method
    // SimpleMotorFeedforward angleFeedforward = new SimpleMotorFeedforward(
    //         Constants.SwerveConstants.angleKS, Constants.SwerveConstants.angleKV, Constants.SwerveConstants.angleKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        zeroedWheelCANCoderAngle = moduleConstants.angleOffset / 360;

        /* Angle Encoder Config */
        angleEncoder = moduleConstants.canivoreName.isEmpty()
                ? new CANcoder(moduleConstants.cancoderID)
                : new CANcoder(moduleConstants.cancoderID, moduleConstants.canivoreName.get());
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = moduleConstants.canivoreName.isEmpty()
                ? new TalonFX(moduleConstants.angleMotorID)
                : new TalonFX(moduleConstants.angleMotorID, moduleConstants.canivoreName.get());
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = moduleConstants.canivoreName.isEmpty()
                ? new TalonFX(moduleConstants.driveMotorID)
                : new TalonFX(moduleConstants.driveMotorID, moduleConstants.canivoreName.get());
        configDriveMotor();

        lastAngle = getState().angle.getDegrees();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        setDesiredState(desiredState, isOpenLoop, false);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean isSecondOrder) {
        // Custom optimize command, since default WPILib optimize assumes continuous controller, which CTRE is not
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);

        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
            driveMotor.setControl(voltageRequestDrive.withOutput(percentOutput * Constants.GlobalConstants.targetVoltage));
        } else {
            double velocity = Conversions.MPSToMotorRPS(
                    desiredState.speedMetersPerSecond,
                    Constants.SwerveConstants.wheelCircumference,
                    Constants.SwerveConstants.driveGearRatio);
            driveMotor.setControl(
                    velocityVoltageRequestDrive.withVelocity(velocity).withFeedForward(driveFeedforward.calculate(desiredState.speedMetersPerSecond)));
        }

        // Determine the angle to set the module to
        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle
                        .getDegrees(); // Prevent rotating module if speed is less then 1%. Prevents Jittering.

        lastAngle = angle;

        // Account for the velocity of the angle motor if in second order mode
        if (isSecondOrder && desiredState instanceof SecondOrderSwerveModuleState) {
            angleMotor.setControl(positionVoltageRequestAngle.withPosition(
                    angle/360).withFeedForward(
                    ((SecondOrderSwerveModuleState) desiredState).angularVelocityRadiansPerSecond
                            * Constants.SwerveConstants.calculatedAngleKV));
        } else {
            angleMotor.setControl(positionVoltageRequestAngle.withPosition(angle/360).withFeedForward(0));
        }
    }

    public void setDesiredAngleOnly(Rotation2d desiredAngle, boolean optimized) {
        // Set the module to face forwards
        if (optimized) {
            desiredAngle = CTREModuleState.optimize(new SwerveModuleState(1, desiredAngle), getState().angle).angle;
        }

        angleMotor.setControl(positionVoltageRequestAngle.withPosition(
                desiredAngle.getRadians()/360).withFeedForward(0));

        lastAngle = 0;

        // Stop the motor to bypass the speed check
        driveMotor.stopMotor();
    }

    public void setDriveCharacterizationVoltage(double voltage) {
        // Set the module to face forwards
        angleMotor.setControl(positionVoltageRequestAngle.withPosition(0).withFeedForward(0));

        lastAngle = 0;

        // Set the drive motor to the specified voltage
        driveMotor.setControl(voltageRequestDrive.withOutput(voltage));
    }

    public void setAngleCharacterizationVoltage(double voltage) {
        // Set the module to face forwards
        angleMotor.setControl(voltageRequestDrive.withOutput(voltage));

        lastAngle = 0;

        // Set the drive motor to just enough to overcome static friction
        driveMotor.setControl(voltageRequestDrive.withOutput(0));
    }

    public void disableMotors() {
        driveMotor.stopMotor();
        angleMotor.stopMotor();
    }

    private void configAngleEncoder() {
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCanCoderConfig);
        MagnetSensorConfigs myMagnetSensorConfigs = new MagnetSensorConfigs();
        angleEncoder.getConfigurator().refresh(myMagnetSensorConfigs);
        myMagnetSensorConfigs.MagnetOffset = -zeroedWheelCANCoderAngle;
        angleEncoder.getConfigurator().apply(myMagnetSensorConfigs);
    }

    private void  configAngleMotor() {
        TalonFXConfigurator configurator = angleMotor.getConfigurator();
        configurator.apply(Robot.ctreConfigs.swerveAngleFXConfig);

        MotorOutputConfigs outputConfigs = new MotorOutputConfigs();

        outputConfigs.Inverted = Constants.SwerveConstants.angleMotorInvert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        outputConfigs.NeutralMode = Constants.SwerveConstants.angleNeutralMode;

        configurator.apply(outputConfigs);

        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.FeedbackRemoteSensorID = angleEncoder.getDeviceID();
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        feedbackConfigs.RotorToSensorRatio = Constants.SwerveConstants.angleGearRatio;
        configurator.apply(feedbackConfigs);
    }

    private void configDriveMotor() {
        TalonFXConfigurator configurator = driveMotor.getConfigurator();
        configurator.apply(Robot.ctreConfigs.swerveDriveFXConfig);

        MotorOutputConfigs outputConfigs = new MotorOutputConfigs();

        outputConfigs.NeutralMode = Constants.SwerveConstants.driveNeutralMode;
        outputConfigs.Inverted = Constants.SwerveConstants.driveMotorInvert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        
        configurator.apply(outputConfigs);
    }

    public Rotation2d getCanCoderWithOffset() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    public Rotation2d getCanCoderNoOffset() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue() + zeroedWheelCANCoderAngle/360);
    }

    public TalonFX getDriveMotor() {
        return driveMotor;
    }

    public TalonFX getAngleMotor() {
        return angleMotor;
    }

    public SwerveModuleState getState() {
        double velocity =
                Conversions.motorRPSToMPS(driveMotor.getRotorVelocity().getValue(),
                Constants.SwerveConstants.wheelCircumference,
                Constants.SwerveConstants.driveGearRatio);
        Rotation2d angle = Rotation2d.fromRotations(
                BaseStatusSignal.getLatencyCompensatedValue(angleMotor.getPosition().refresh(), angleMotor.getVelocity().refresh()));
        return new SwerveModuleState(velocity, angle);
    }

    public double getAngularVelocity() {
        return Conversions.motorRPSToRadPS(angleMotor.getVelocity().getValue(), Constants.SwerveConstants.angleGearRatio);
    }

    public SwerveModulePosition getPosition() {
        double encoder = Conversions.motorToMeters(
                        driveMotor.getPosition().getValue(),
                        Constants.SwerveConstants.wheelCircumference,
                        Constants.SwerveConstants.driveGearRatio);
        Rotation2d angle = Rotation2d.fromRotations(
                angleMotor.getPosition().getValue());
        return new SwerveModulePosition(encoder, angle);
    }

    public double getDriveTemperature() {
        return driveMotor.getDeviceTemp().getValue();
    }

    public double getAngleTemperature() {
        return angleMotor.getDeviceTemp().getValue();
    }

    public double getDriveVoltage() {
        return driveMotor.getSupplyVoltage().getValue();
    }

    public double getAngleVoltage() {
        return angleMotor.getSupplyVoltage().getValue();
    }

    public double getDriveCurrent() {
        return driveMotor.getSupplyCurrent().getValue();
    }

    public double getAngleCurrent() {
        return angleMotor.getSupplyCurrent().getValue();
    }
}
