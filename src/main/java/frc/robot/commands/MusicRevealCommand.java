package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.LightsSubsystem.LEDSegment;
import java.util.List;

// Sketchy Michael code. Not working atm
public class MusicRevealCommand extends CommandBase {
    private final Timer m_Part1 = new Timer();
    private final Timer m_part2 = new Timer();
    private final Timer m_part3 = new Timer();
    private final Timer m_part4 = new Timer();
    private final Timer m_part5 = new Timer();
    private final Timer m_totalTime = new Timer();

    // public List<Boolean> isLedOn;
    public int i = 0;
    // Get & Index
    public MusicRevealCommand(List<Boolean> isLedOn, LightsSubsystem lightsSubsystem) {
        addRequirements(lightsSubsystem);
        // this.isLedOn = isLedOn;
    }

    @Override
    public void initialize() {
        m_totalTime.reset();
        m_Part1.reset();
        m_part2.reset();
        m_part3.reset();
        m_part4.reset();
        m_part5.reset();
        m_part2.stop();
        m_part3.reset();
        m_part4.reset();
        m_part5.reset();
        m_Part1.start();
        m_totalTime.start();
        LEDSegment.MainStrip.setColor(LightsSubsystem.orange);
        i = 0;
    }

    @Override
    public void execute() {
        // if (isLedOn.get(i)) LEDSegment.MainStrip.setColor(LightsSubsystem.orange);
        // else LEDSegment.MainStrip.disableLEDs();

        if (m_totalTime.hasElapsed(9.8)) m_part2.start();
        if (m_totalTime.hasElapsed(58.79166666)) m_part3.start();
        if (m_totalTime.hasElapsed(93.75)) m_part4.start();
        if (m_totalTime.hasElapsed(108.83333333)) m_part5.start();
        i++;

        if (m_totalTime.hasElapsed(10.6)) m_Part1.stop();
        else if (m_Part1.advanceIfElapsed(3.6)) LEDSegment.MainStrip.setColor(LightsSubsystem.yellow);
        else if (m_Part1.hasElapsed(.65)) LEDSegment.MainStrip.setColor(LightsSubsystem.purple);
        else if (m_Part1.hasElapsed(.9)) LEDSegment.MainStrip.setColor(LightsSubsystem.yellow);
        // else LEDSegment.MainStrip.setColor((LightsSubsystem.purple));

        if (m_totalTime.hasElapsed(39.1)) m_part2.stop();
        else if (m_part2.advanceIfElapsed(7.2)) LEDSegment.MainStrip.setColor(LightsSubsystem.orange);
        else if (m_part2.hasElapsed(6.75)) LEDSegment.MainStrip.setColor(LightsSubsystem.red);
        else if (m_part2.hasElapsed(6.4)) LEDSegment.MainStrip.setColor(LightsSubsystem.blue);
        // else LEDSegment.MainStrip.setColor(LightsSubsystem.orange);

        if (m_totalTime.hasElapsed(93.75)) m_part3.stop();
        else if (m_part3.advanceIfElapsed(.44776)) LEDSegment.MainStrip.setColor(LightsSubsystem.orange);
        else if (m_part3.hasElapsed(0)) LEDSegment.MainStrip.setColor(LightsSubsystem.black);

        if (m_totalTime.hasElapsed(108.83333)) m_part4.stop();
        else if (m_part4.advanceIfElapsed(3.083333)) LEDSegment.MainStrip.setColor(LightsSubsystem.purple);
        else if (m_part4.hasElapsed(1.79104)) LEDSegment.MainStrip.setColor(LightsSubsystem.red);
        else if (m_part4.hasElapsed(1.34328)) LEDSegment.MainStrip.setColor(LightsSubsystem.yellow);
        else if (m_part4.hasElapsed(.89552)) LEDSegment.MainStrip.setColor(LightsSubsystem.orange);
        else if (m_part4.hasElapsed(.44776)) LEDSegment.MainStrip.setColor(LightsSubsystem.black);

        if (m_totalTime.hasElapsed(120)) m_part5.stop();
        else if (m_part5.advanceIfElapsed(.89552)) LEDSegment.MainStrip.setColor(LightsSubsystem.orange);
        else if (m_part5.hasElapsed(.44776)) LEDSegment.MainStrip.setColor(LightsSubsystem.black);
    }

    @Override
    public void end(boolean interrupted) {
        if (m_totalTime.hasElapsed(120)) m_totalTime.stop();
    }
}
