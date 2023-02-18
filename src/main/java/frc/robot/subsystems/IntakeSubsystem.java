package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logging.LoggedReceiver;
import frc.lib.logging.Logger;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private WPI_TalonSRX intakeMotor = new WPI_TalonSRX(16);
    private DoubleSolenoid positionSolenoid = new DoubleSolenoid(
            GlobalConstants.PCM_ID,
            PneumaticsModuleType.REVPH,
            IntakeConstants.positionForwardChannel,
            IntakeConstants.positionReverseChannel);
    private DoubleSolenoid shootingSolenoid = new DoubleSolenoid(
            GlobalConstants.PCM_ID,
            PneumaticsModuleType.REVPH,
            IntakeConstants.shootingForwardChannel,
            IntakeConstants.shootingReverseChannel);

    private IntakeMode intakeMode = IntakeMode.DISABLED;

    private LoggedReceiver intakeSpeedReceiver;
    private LoggedReceiver reverseSpeedReceiver;
    private LoggedReceiver shootingSpeedReceiver;
    private LoggedReceiver shootingDelayReceiver;

    private Timer shootingDelayTimer = new Timer();

    public IntakeSubsystem() {
        intakeMotor.setNeutralMode(NeutralMode.Brake);
        intakeMotor.configVoltageCompSaturation(GlobalConstants.targetVoltage);
        intakeMotor.enableVoltageCompensation(true);
        intakeMotor.setInverted(true);

        SupplyCurrentLimitConfiguration supplyLimit = new SupplyCurrentLimitConfiguration(true, 3.6, .5, .25);

        intakeMotor.configSupplyCurrentLimit(supplyLimit);

        // run cylinder before wheels

        intakeSpeedReceiver = Logger.tunable("/IntakeSubsystem/IntakeSpeed", 0.7);
        reverseSpeedReceiver = Logger.tunable("/IntakeSubsystem/ReverseSpeed", -0.3);
        shootingSpeedReceiver = Logger.tunable("/IntakeSubsystem/ShootingSpeed", -1.0);

        shootingDelayReceiver = Logger.tunable("/IntakeSubsystem/ShootingDelay", 0.0);

        setDefaultCommand(stopIntakeCommand());
    }

    public Command runIntakeCommand() {
        return run(() -> {
            setIntakeMode(IntakeMode.INTAKE);
        });
    }

    public Command reverseIntakeCommand() {
        return run(() -> {
            setIntakeMode(IntakeMode.REVERSE);
        });
    }

    public Command shootCommand() {
        return run(() -> {
            setIntakeMode(IntakeMode.SHOOT);
        });
    }

    public Command stopIntakeCommand() {
        return run(() -> {
            setIntakeMode(IntakeMode.DISABLED);
        });
    }

    public void setIntakeMode(IntakeMode intakeMode) {
        this.intakeMode = intakeMode;
    }

    public IntakeMode getIntakeMode() {
        return intakeMode;
    }

    @Override
    public void periodic() {
        switch (intakeMode) {
            case DISABLED:
                shootingDelayTimer.stop();
                shootingDelayTimer.reset();

                positionSolenoid.set(Value.kReverse);
                shootingSolenoid.set(Value.kReverse);
                intakeMotor.stopMotor();
                break;
            case INTAKE:
                positionSolenoid.set(Value.kForward);
                shootingSolenoid.set(Value.kReverse);
                intakeMotor.set(ControlMode.PercentOutput, intakeSpeedReceiver.getDouble());
                break;
            case REVERSE:
                positionSolenoid.set(Value.kReverse);
                shootingSolenoid.set(Value.kReverse);
                intakeMotor.set(ControlMode.PercentOutput, reverseSpeedReceiver.getDouble());
                break;
            case SHOOT:
                shootingDelayTimer.start();

                if (shootingDelayTimer.hasElapsed(shootingDelayReceiver.getDouble())) {
                    shootingSolenoid.set(Value.kForward);
                }

                // Shooting should be extended, and position should be retracted.
                positionSolenoid.set(Value.kReverse);
                intakeMotor.set(ControlMode.PercentOutput, shootingSpeedReceiver.getDouble());

                break;
        }

        Logger.log("/IntakeSubsystem/IntakeMotorSupplyCurrent", intakeMotor.getSupplyCurrent());
    }

    public enum IntakeMode {
        DISABLED,
        INTAKE,
        REVERSE,
        SHOOT
    }
}
