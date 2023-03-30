package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.logging.LoggedReceiver;
import frc.lib.logging.Logger;
import frc.robot.subsystems.ArmSubsystem;

public class TestCommand extends CommandBase {
    ProfiledPIDController testController;
    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(4, 1);
    double position;
    ArmSubsystem armSubsystem;
    LoggedReceiver tuningReceiver;

    public TestCommand(ArmSubsystem armSubsystem) {
        testController = new ProfiledPIDController(2, 0, 0, constraints);
        tuningReceiver = Logger.tunable("/TestCommand/armValues", new double[] {0, 0, 0});
        this.armSubsystem = armSubsystem;
    }

    @Override
    public void execute() {
        double[] angles = tuningReceiver.getDoubleArray();
        Logger.log(
                "/TestCommand/calculatedValues", armSubsystem.feedforwardPassthrough(angles[0], angles[1], 0, 0, 0, 0));
    }
}
