package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.LightsSubsystem.LEDSegment;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.LimelightMode;

public class IndicateGridAimedCommand extends CommandBase {
    private VisionSubsystem visionSubsystem;

    private final int taps = 5;
    private LinearFilter txRollingAverage = LinearFilter.movingAverage(taps);

    private boolean hasVisionMeasurement = false;

    private final double angleThreshold = 2.0;
    private final double areaThreshold = 0.05;

    public IndicateGridAimedCommand(VisionSubsystem visionSubsystem, LightsSubsystem lightsSubsystem) {
        this.visionSubsystem = visionSubsystem;

        addRequirements(visionSubsystem, lightsSubsystem);
    }

    @Override
    public void initialize() {
        visionSubsystem.setBackLimelightMode(LimelightMode.RETROREFLECTIVEHIGH);

        hasVisionMeasurement = false;
    }

    @Override
    public void execute() {
        if (visionSubsystem.hasBackRetroreflectiveAngles() && visionSubsystem.isBackLimelightAtPipeline()) {
            // Initialize the moving average
            if (!hasVisionMeasurement) {
                initializeMovingAverage();

                hasVisionMeasurement = true;
            }

            var retroreflectiveAngles =
                    visionSubsystem.getBackRetroreflectiveAngles().get();

            double tx = txRollingAverage.calculate(retroreflectiveAngles.tx());
            double ta = retroreflectiveAngles.ta();

            if (Math.abs(tx) <= angleThreshold && ta >= areaThreshold) {
                LEDSegment.MainStrip.setColor(LightsSubsystem.green);
            } else {
                LEDSegment.MainStrip.setStrobeAnimation(LightsSubsystem.yellow, 0.3);
            }
        } else {
            LEDSegment.MainStrip.setStrobeAnimation(LightsSubsystem.yellow, 0.3);
        }
    }

    private void initializeMovingAverage() {
        // Fill the moving average filter to prevent averaging with 0
        var initial = visionSubsystem.getBackRetroreflectiveAngles().get().tx();
        for (int i = 0; i < taps; i++) txRollingAverage.calculate(initial);
    }

    @Override
    public void end(boolean interrupted) {
        txRollingAverage.reset();
    }
}
