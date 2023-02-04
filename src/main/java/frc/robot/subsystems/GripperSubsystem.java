package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logging.LoggedReceiver;
import frc.lib.logging.Logger;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.GripperConstants;

public class GripperSubsystem extends SubsystemBase {
    private DoubleSolenoid gripperSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, GripperConstants.FORWARD_CHANNEL, GripperConstants.REVERSE_CHANNEL);
    private WPI_TalonSRX gripperMotor = new WPI_TalonSRX(GripperConstants.GRIPPER_MOTOR);

    private GripperState gripperState = GripperState.OPEN;

    private LoggedReceiver gripperIntakeSpeed = Logger.receive("/Gripper/Intake Speed", 0.2);
    private LoggedReceiver gripperEjectSpeed = Logger.receive("/Gripper/Intake Speed", -0.2);

    public GripperSubsystem() {
        gripperMotor.setNeutralMode(NeutralMode.Brake);
        gripperMotor.configVoltageCompSaturation(GlobalConstants.targetVoltage);
        gripperMotor.enableVoltageCompensation(true);
        gripperMotor.setInverted(true);

        SupplyCurrentLimitConfiguration supplyLimit = new SupplyCurrentLimitConfiguration(
                true,
                20,
                30,
                0.1);

        gripperMotor.configSupplyCurrentLimit(supplyLimit);
    }

    public Command openGripperCommand() {
        return runOnce(() -> setState(GripperState.OPEN));
    }

    public Command closeGripperCommand() {
        return runOnce(() -> setState(GripperState.CLOSED));
    }

    public Command ejectFromGripperCommand() {
        return runOnce(() -> setState(GripperState.EJECT));
    }

    @Override
    public void periodic() {
        switch (gripperState) {
            case OPEN:
                gripperSolenoid.set(Value.kReverse);
                gripperMotor.stopMotor();
                break;
            case CLOSED:
                gripperSolenoid.set(Value.kForward);
                gripperMotor.set(gripperIntakeSpeed.getDouble());
                break;
            case EJECT:
                gripperSolenoid.set(Value.kReverse);
                gripperMotor.set(gripperEjectSpeed.getDouble());
                break;
        }

        Logger.log("/Gripper/Current", gripperMotor.getSupplyCurrent());
    }

    private void setState(GripperState gripperState) {
        this.gripperState = gripperState;
    }

    private enum GripperState {
        OPEN,    // Open without spinning motor
        CLOSED,  // Closed and spin motor until supply limit
        EJECT    // Reverse motors and then open
    }
}
