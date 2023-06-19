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

    private IntakeMode intakeMode = IntakeMode.DISABLED;

    private LoggedReceiver intakeSpeedReceiver;
    private LoggedReceiver holdingSpeedReciever;
    private LoggedReceiver reverseSpeedReceiver;
    private LoggedReceiver slowReverseSpeedReceiver;
    private LoggedReceiver shootingSpeedReceiver;
    private LoggedReceiver handoffSpeedReceiver;

    private Timer holdDelayTimer = new Timer();

    AnalogInput intakeSensor1 = new AnalogInput(2);
    AnalogInput intakeSensor2 = new AnalogInput(3);

    public IntakeSubsystem() {
        intakeMotor.setNeutralMode(NeutralMode.Brake);
        intakeMotor.configVoltageCompSaturation(GlobalConstants.targetVoltage);
        intakeMotor.enableVoltageCompensation(true);
        intakeMotor.setInverted(false); // true at worlds

        SupplyCurrentLimitConfiguration supplyLimit = new SupplyCurrentLimitConfiguration(true, 20, 30, 0.1);

        intakeMotor.configSupplyCurrentLimit(supplyLimit);

        intakeSpeedReceiver = Logger.tunable("/IntakeSubsystem/IntakeSpeed", 0.75);
        slowReverseSpeedReceiver = Logger.tunable("/IntakeSubsystem/SlowReverseSpeed", -0.1);
        reverseSpeedReceiver = Logger.tunable("/IntakeSubsystem/ReverseSpeed", -0.4);
        shootingSpeedReceiver = Logger.tunable("/IntakeSubsystem/ShootingSpeed", -1.0);
        handoffSpeedReceiver = Logger.tunable("/IntakeSubsystem/HandoffSpeed", 0.99);
        holdingSpeedReciever = Logger.tunable("/IntakeSubsystem/HoldingSpeed", 0.1);

        setDefaultCommand(stopIntakeCommand());

        holdDelayTimer.restart();
    }

    public boolean hasGamePiece() {
        return intakeSensor1.getValue() < 50 || intakeSensor2.getValue() < 50;
    }

    public Command handoffCommand() {
        return startEnd(
                () -> {
                    setIntakeMode(IntakeMode.HANDOFF);
                },
                () -> {});
    }

    public Command intakeCommand() {
        return Commands.either(reverseIntakeModeCommand(), intakeModeCommand(), this::hasGamePiece);
    }

    public Command intakeModeCommand() {
        return startEnd(
                        () -> {
                            setIntakeMode(IntakeMode.INTAKE);
                        },
                        () -> {})
                .until(this::hasGamePiece);
    }

    public Command reverseIntakeModeCommand() {
        return startEnd(
                () -> {
                    setIntakeMode(IntakeMode.REVERSE);
                },
                () -> {});
    }

    public Command slowReverseIntakeModeCommand() {
        return startEnd(
                () -> {
                    setIntakeMode(IntakeMode.SLOW_REVERSE);
                },
                () -> {});
    }

    public Command shootCommand() {
        return startEnd(
                () -> {
                    setIntakeMode(IntakeMode.SHOOT);
                },
                () -> {});
    }

    public Command stopIntakeCommand() {
        return startEnd(
                () -> {
                    setIntakeMode(IntakeMode.DISABLED);
                },
                () -> {});
    }

    public void setIntakeMode(IntakeMode intakeMode) {
        if (this.intakeMode == intakeMode) return;

        this.intakeMode = intakeMode;

        holdDelayTimer.restart();
    }

    public IntakeMode getIntakeMode() {
        return intakeMode;
    }

    @Override
    public void periodic() {
        switch (intakeMode) {
            case DISABLED:
                positionSolenoid.set(Value.kReverse);

                if (holdDelayTimer.hasElapsed(2.7) && !hasGamePiece()) intakeMotor.stopMotor();
                else intakeMotor.set(ControlMode.PercentOutput, holdingSpeedReciever.getDouble());

                break;
            case INTAKE:
                positionSolenoid.set(Value.kForward);
                intakeMotor.set(ControlMode.PercentOutput, intakeSpeedReceiver.getDouble());
                break;
            case HANDOFF:
                positionSolenoid.set(Value.kReverse);
                intakeMotor.set(ControlMode.PercentOutput, handoffSpeedReceiver.getDouble());
                break;
            case REVERSE:
                positionSolenoid.set(Value.kReverse);
                intakeMotor.set(ControlMode.PercentOutput, reverseSpeedReceiver.getDouble());
                break;
            case SLOW_REVERSE:
                positionSolenoid.set(Value.kReverse);
                intakeMotor.set(ControlMode.PercentOutput, slowReverseSpeedReceiver.getDouble());
                break;
            case SHOOT:
                positionSolenoid.set(Value.kReverse);
                intakeMotor.set(ControlMode.PercentOutput, shootingSpeedReceiver.getDouble());
                break;
        }

        Logger.log("/IntakeSubsystem/IntakeSensor1", intakeSensor1.getValue() < 50);
        Logger.log("/IntakeSubsystem/IntakeSensor2", intakeSensor2.getValue() < 50);
    }

    public enum IntakeMode {
        DISABLED,
        INTAKE,
        REVERSE,
        SLOW_REVERSE,
        SHOOT,
        HANDOFF,
    }
}
