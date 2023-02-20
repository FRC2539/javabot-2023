package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.logging.Logger;
import frc.lib.swerve.CTREConfigs;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.GlobalConstants;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.LightsSubsystem.LEDSegment;
import frc.robot.subsystems.VisionSubsystem.LimelightMode;

public class Robot extends TimedRobot {
    public static CTREConfigs ctreConfigs = new CTREConfigs();

    public static Compressor compressor = new Compressor(GlobalConstants.PCM_ID, PneumaticsModuleType.REVPH);

    private RobotContainer robotContainer;

    private Command autonomousCommand;

    public Robot() {}

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

        compressor.enableAnalog(GlobalConstants.minimumPressure, GlobalConstants.maximumPressure);

        Logger.log("/Robot/Minimum Pressure", GlobalConstants.minimumPressure);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        Logger.log("/Robot/Battery Voltage", RobotController.getBatteryVoltage());
        Logger.log("/Robot/Pressure", compressor.getPressure());

        Logger.update();
    }

    @Override
    public void autonomousInit() {
        // Reset arm pid controllers
        robotContainer.getArmSubsystem().resetPIDControllers();

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

        // Reset arm pid controllers
        robotContainer.getArmSubsystem().resetPIDControllers();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {
        // Calibrate swerve encoders
        // robotContainer.getSwerveDriveSubsystem().calibrateIntegratedEncoders();

        // Set correct limelight mode (run while disabled for consistency)
        robotContainer.getVisionSubsystem().setLimelightMode(LimelightMode.APRILTAG);

        // Update the autonomous command with driver station configuration
        robotContainer.autonomousManager.update();

        // Indicate if the battery is at voltage
        if (RobotController.getBatteryVoltage() > GlobalConstants.batteryVoltageThreshold)
            LEDSegment.BatteryIndicator.setColor(LightsSubsystem.green.dim(0.25));
        else LEDSegment.BatteryIndicator.setFadeAnimation(LightsSubsystem.green.dim(0.25), 1);

        // Indicate if there is enough pressure in the pneumatic system
        if (compressor.getPressure() > GlobalConstants.minimumPressure)
            LEDSegment.PressureIndicator.setColor(LightsSubsystem.purple.dim(0.25));
        else LEDSegment.PressureIndicator.setFadeAnimation(LightsSubsystem.purple.dim(0.25), 1);

        // Verify that all absolute encoders are connected
        if (robotContainer.getArmSubsystem().isMastThroughBoreConnected())
            LEDSegment.MastEncoderIndicator.setColor(LightsSubsystem.white.dim(0.25));
        else LEDSegment.MastEncoderIndicator.fullClear();

        if (robotContainer.getArmSubsystem().isBoomThroughBoreConnected())
            LEDSegment.BoomEncoderIndicator.setColor(LightsSubsystem.white.dim(0.25));
        else LEDSegment.BoomEncoderIndicator.fullClear();

        if (robotContainer.getArmSubsystem().isWristThroughBoreConnected())
            LEDSegment.WristEncoderIndicator.setColor(LightsSubsystem.white.dim(0.25));
        else LEDSegment.WristEncoderIndicator.fullClear();

        // Passive Main LED Mode
        LEDSegment.MainStrip.setFadeAnimation(LightsSubsystem.orange, 0.5);
    }

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}
}
