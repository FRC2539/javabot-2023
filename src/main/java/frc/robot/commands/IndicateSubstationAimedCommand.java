package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.vision.BackLimelight;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.LightsSubsystem.LEDSegment;
import frc.robot.subsystems.VisionSubsystem;

public class IndicateSubstationAimedCommand extends CommandBase {
    private VisionSubsystem visionSubsystem;

    private final int taps = 5;
    private LinearFilter txRollingAverage = LinearFilter.movingAverage(taps);

    private boolean hasVisionMeasurement = false;

    private final double angleThreshold = 6.0;
    private final double areaThreshold = 0.05;

    public IndicateSubstationAimedCommand(VisionSubsystem visionSubsystem, LightsSubsystem lightsSubsystem) {
        this.visionSubsystem = visionSubsystem;

        addRequirements(visionSubsystem, lightsSubsystem);
    }

    @Override
    public void initialize() {
        visionSubsystem.getBackLimelight().setMode(BackLimelight.Mode.CONE);

        hasVisionMeasurement = false;
    }

    @Override
    public void execute() {
        if (visionSubsystem.getBackLimelight().hasLimelightRawAngles()) {
            // Initialize the moving average
            if (!hasVisionMeasurement) {
                initializeMovingAverage();

                hasVisionMeasurement = true;
            }

            var retroreflectiveAngles =
                    visionSubsystem.getBackLimelight().getLimelightRawAngles().get();

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
        var initial =
                visionSubsystem.getBackLimelight().getLimelightRawAngles().get().tx();
        for (int i = 0; i < taps; i++) txRollingAverage.calculate(initial);
    }

    @Override
    public void end(boolean interrupted) {
        txRollingAverage.reset();
    }
}
