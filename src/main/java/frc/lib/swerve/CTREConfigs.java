package frc.lib.swerve;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import frc.robot.Constants;
import frc.robot.Constants.GlobalConstants;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;

    public CTREConfigs() {
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
                Constants.SwerveConstants.angleEnableCurrentLimit,
                Constants.SwerveConstants.angleContinuousCurrentLimit,
                Constants.SwerveConstants.anglePeakCurrentLimit,
                Constants.SwerveConstants.anglePeakCurrentDuration);

        swerveAngleFXConfig.slot0.kP = Constants.SwerveConstants.angleKP;
        swerveAngleFXConfig.slot0.kI = Constants.SwerveConstants.angleKI;
        swerveAngleFXConfig.slot0.kD = Constants.SwerveConstants.angleKD;
        swerveAngleFXConfig.slot0.kF = Constants.SwerveConstants.angleKF;
        swerveAngleFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;
        swerveAngleFXConfig.voltageCompSaturation = GlobalConstants.targetVoltage;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
                Constants.SwerveConstants.driveEnableCurrentLimit,
                Constants.SwerveConstants.driveContinuousCurrentLimit,
                Constants.SwerveConstants.drivePeakCurrentLimit,
                Constants.SwerveConstants.drivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = Constants.SwerveConstants.driveKP;
        swerveDriveFXConfig.slot0.kI = Constants.SwerveConstants.driveKI;
        swerveDriveFXConfig.slot0.kD = Constants.SwerveConstants.driveKD;
        swerveDriveFXConfig.slot0.kF = Constants.SwerveConstants.driveKF;
        swerveDriveFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        swerveDriveFXConfig.openloopRamp = Constants.SwerveConstants.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = Constants.SwerveConstants.closedLoopRamp;
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.voltageCompSaturation = GlobalConstants.targetVoltage;

        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = Constants.SwerveConstants.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}
