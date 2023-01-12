package frc.lib.loops;

import edu.wpi.first.wpilibj.TimesliceRobot;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class UpdateManager {
    private TimesliceRobot robot;

    public UpdateManager(TimesliceRobot robot) {
        this.robot = robot;
    }

    /**
     * @param subsystem The subsystem getting registered.
     * @param updateTimeslice The corresponding timeslice of the subsystem.
     */
    public void schedule(Subsystem subsystem, double updateTimeslice) {
        robot.schedule(() -> ((Updatable) subsystem).update(), updateTimeslice);
    }
}
