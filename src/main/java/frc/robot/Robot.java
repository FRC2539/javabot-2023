package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimesliceRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.logging.Logger;
import frc.lib.swerve.CTREConfigs;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.TimesliceConstants;

public class Robot extends TimesliceRobot {
    public static CTREConfigs ctreConfigs = new CTREConfigs();

    private RobotContainer robotContainer;

    private Command autonomousCommand;

    public Robot() {
        super(TimesliceConstants.ROBOT_PERIODIC_ALLOCATION, TimesliceConstants.CONTROLLER_PERIOD);
    }

    @Override
    public void robotInit() {
        // Disable default NetworkTables logging
        DataLogManager.logNetworkTables(false);

        // Begin controller inputs
        if (isReal()) {
            DriverStation.startDataLog(DataLogManager.getLog());
        }

        robotContainer = new RobotContainer(this);

        // Prevents the logging of many errors with our controllers
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        Logger.update();
    }

    @Override
    public void autonomousInit() {
        // Set april tags to use the correct origin (red or blue corner)
        FieldConstants.setAprilTagOrigin();

        autonomousCommand = robotContainer.getAutonomousCommand();

        // Schedule the chosen autonomous command
        if (autonomousCommand != null) autonomousCommand.schedule();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // Set april tags to use the correct origin (red or blue corner)
        FieldConstants.setAprilTagOrigin();

        // Prevent any autonomous code from overrunning into teleop
        if (autonomousCommand != null) autonomousCommand.cancel();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void disabledInit() {}

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}
}
