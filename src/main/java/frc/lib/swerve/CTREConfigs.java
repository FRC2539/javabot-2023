package frc.lib.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import frc.robot.Constants;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANcoderConfiguration swerveCanCoderConfig;

    public CTREConfigs() {
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANcoderConfiguration();

        /* Swerve Angle Motor Configurations */
        CurrentLimitsConfigs angleSupplyLimit = new CurrentLimitsConfigs();
        angleSupplyLimit.SupplyCurrentLimitEnable = Constants.SwerveConstants.angleEnableCurrentLimit;
        angleSupplyLimit.SupplyCurrentLimit = Constants.SwerveConstants.angleContinuousCurrentLimit;
        angleSupplyLimit.SupplyCurrentThreshold = Constants.SwerveConstants.anglePeakCurrentLimit;
        angleSupplyLimit.SupplyTimeThreshold = Constants.SwerveConstants.anglePeakCurrentDuration;

        swerveAngleFXConfig.Slot0.kP = Constants.SwerveConstants.angleKP;
        swerveAngleFXConfig.Slot0.kI = Constants.SwerveConstants.angleKI;
        swerveAngleFXConfig.Slot0.kD = Constants.SwerveConstants.angleKD;
        swerveAngleFXConfig.Slot0.kS = Constants.SwerveConstants.angleKS;
        swerveAngleFXConfig.Slot0.kV = Constants.SwerveConstants.angleKV;
        swerveAngleFXConfig.CurrentLimits = angleSupplyLimit;

        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;

        /* Swerve Drive Motor Configuration */
        CurrentLimitsConfigs driveSupplyLimit = new CurrentLimitsConfigs();
        driveSupplyLimit.SupplyCurrentLimitEnable = Constants.SwerveConstants.driveEnableCurrentLimit;
        driveSupplyLimit.SupplyCurrentLimit = Constants.SwerveConstants.driveContinuousCurrentLimit;
        driveSupplyLimit.SupplyCurrentThreshold = Constants.SwerveConstants.drivePeakCurrentLimit;
        driveSupplyLimit.SupplyTimeThreshold = Constants.SwerveConstants.drivePeakCurrentDuration;

        swerveDriveFXConfig.Slot0.kP = Constants.SwerveConstants.driveKP;
        swerveDriveFXConfig.Slot0.kI = Constants.SwerveConstants.driveKI;
        swerveDriveFXConfig.Slot0.kD = Constants.SwerveConstants.driveKD;
        swerveDriveFXConfig.Slot0.kS = Constants.SwerveConstants.driveKS;
        swerveDriveFXConfig.Slot0.kV = Constants.SwerveConstants.driveKV;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.SwerveConstants.openLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.SwerveConstants.closedLoopRamp;
        swerveDriveFXConfig.CurrentLimits = driveSupplyLimit;

        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        swerveCanCoderConfig.MagnetSensor.SensorDirection = Constants.SwerveConstants.canCoderInvert;

        /* does not set the magnet offset currently due to needing to set magnet offset to trim */
    }
}
