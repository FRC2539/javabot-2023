package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    private LoggedReceiver stoppedSpeedReciever;
    private LoggedReceiver reverseSpeedReceiver;
    private LoggedReceiver shootingSpeedReceiver;
    private LoggedReceiver shootingDelayReceiver;

    private Timer shootingDelayTimer = new Timer();
    private Timer holdDelayTimer = new Timer();

    AnalogInput intakeSensor1 = new AnalogInput(2);
    AnalogInput intakeSensor2 = new AnalogInput(3);

    public IntakeSubsystem() {
        intakeMotor.setNeutralMode(NeutralMode.Brake);
        intakeMotor.configVoltageCompSaturation(GlobalConstants.targetVoltage);
        intakeMotor.enableVoltageCompensation(true);
        intakeMotor.setInverted(true);

        SupplyCurrentLimitConfiguration supplyLimit = new SupplyCurrentLimitConfiguration(true, 20, 30, 0.1);

        intakeMotor.configSupplyCurrentLimit(supplyLimit);

        intakeSpeedReceiver = Logger.tunable("/IntakeSubsystem/IntakeSpeed", 0.7);
        reverseSpeedReceiver = Logger.tunable("/IntakeSubsystem/ReverseSpeed", -0.3);
        shootingSpeedReceiver = Logger.tunable("/IntakeSubsystem/ShootingSpeed", -1.0);
        stoppedSpeedReciever = Logger.tunable("/IntakeSubsystem/HoldingSpeed", -0.1);

        shootingDelayReceiver = Logger.tunable("/IntakeSubsystem/ShootingDelay", 0.0);

        setDefaultCommand(stopIntakeCommand());

        holdDelayTimer.reset();
        holdDelayTimer.start();

        shootingDelayTimer.reset();
        shootingDelayTimer.start();
    }

    private boolean hasGamePiece() {
        return intakeSensor1.getValue() < 50 || intakeSensor2.getValue() < 50;
    }

    public Command intakeCommand() {
        return Commands.either(reverseIntakeModeCommand(), intakeModeCommand(), this::hasGamePiece);
    }

    public Command intakeModeCommand() {
        return run(() -> {
            setIntakeMode(IntakeMode.INTAKE);
        });
    }

    public Command reverseIntakeModeCommand() {
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
        if (this.intakeMode == intakeMode) return;

        this.intakeMode = intakeMode;

        holdDelayTimer.reset();
        holdDelayTimer.start();

        shootingDelayTimer.reset();
        shootingDelayTimer.start();
    }

    public IntakeMode getIntakeMode() {
        return intakeMode;
    }

    @Override
    public void periodic() {
        switch (intakeMode) {
            case DISABLED:
                positionSolenoid.set(Value.kReverse);
                shootingSolenoid.set(Value.kReverse);
                // intakeMotor.stopMotor();
                if (holdDelayTimer.hasElapsed(1)) {
                    intakeMotor.stopMotor();
                } else {
                    intakeMotor.set(ControlMode.PercentOutput, stoppedSpeedReciever.getDouble());
                }
                break;
            case INTAKE:
                shootingSolenoid.set(Value.kReverse);
                positionSolenoid.set(Value.kForward);

                if (hasGamePiece()) {
                    intakeMotor.stopMotor();
                } else {
                    intakeMotor.set(ControlMode.PercentOutput, intakeSpeedReceiver.getDouble());
                }

                break;
            case REVERSE:
                positionSolenoid.set(Value.kReverse);
                shootingSolenoid.set(Value.kReverse);
                intakeMotor.set(ControlMode.PercentOutput, reverseSpeedReceiver.getDouble());
                break;
            case SHOOT:
                // Shooting should be extended, and position should be retracted.
                positionSolenoid.set(Value.kReverse);
                shootingSolenoid.set(Value.kForward);

                if (shootingDelayTimer.hasElapsed(shootingDelayReceiver.getDouble())) {
                    intakeMotor.set(ControlMode.PercentOutput, shootingSpeedReceiver.getDouble());
                }
                break;
        }

        // Logger.log("/IntakeSubsystem/IntakeMotorSupplyCurrent", intakeMotor.getSupplyCurrent());
    }

    public enum IntakeMode {
        DISABLED,
        INTAKE,
        REVERSE,
        SHOOT
    }
}
