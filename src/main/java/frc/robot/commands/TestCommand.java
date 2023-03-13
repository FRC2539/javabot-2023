package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestCommand extends CommandBase {
    ProfiledPIDController testController;
    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(4, 1);
    double position;

    public TestCommand() {
        testController = new ProfiledPIDController(2, 0, 0, constraints);
    }

    @Override
    public void initialize() {
        testController.reset(new TrapezoidProfile.State(0, 0));
        position = 0;
        testController.setGoal(new TrapezoidProfile.State(10, 0));
    }

    @Override
    public void execute() {
        double pidCorrection = testController.calculate(position);
        double velocity = testController.getSetpoint().velocity + pidCorrection;
        position += velocity * 0.02;
        System.out.println("Velocity: " + velocity + "Position: " + position);
    }
}
