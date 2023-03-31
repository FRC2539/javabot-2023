package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.logging.LoggedReceiver;
import frc.lib.logging.Logger;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class TestCommand extends CommandBase {
    DCMotor myDCMotor1 = DCMotor.getFalcon500(1).withReduction(ArmConstants.arm1GearRatio);
    DCMotor myDCMotor2 = DCMotor.getFalcon500(1).withReduction(ArmConstants.arm2GearRatio);

    ProfiledPIDController testController;
    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(4, 1);
    double position;
    ArmSubsystem armSubsystem;
    LoggedReceiver tuningReceiver;

    public TestCommand(ArmSubsystem armSubsystem) {
        testController = new ProfiledPIDController(2, 0, 0, constraints);
        tuningReceiver = Logger.tunable("/TestCommand/armValues", new double[] {0, 0, 0, 0});
        this.armSubsystem = armSubsystem;
    }

    @Override
    public void execute() {
        double[] angles = tuningReceiver.getDoubleArray();
        double[] feedforwardValues = armSubsystem.feedforwardPassthrough((feedforward) ->
                feedforward.calculateFeedforwardTorques(angles[0], angles[1], angles[2], angles[3], 0, 0));
        double motor1Voltage = myDCMotor1.getVoltage(feedforwardValues[0], angles[2]);
        double motor2Voltage = myDCMotor1.getVoltage(feedforwardValues[1], angles[3]);
        Logger.log("/TestCommand/calculatedValues", new double[] {motor1Voltage, motor2Voltage});
    }
}
