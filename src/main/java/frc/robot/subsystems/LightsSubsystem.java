package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightsConstants;

public class LightsSubsystem extends SubsystemBase {
    private static final CANdle candle = new CANdle(LightsConstants.CANDLE_PORT);

    public static final Color orange = new Color(255, 25, 0);
    public static final Color black = new Color(0, 0, 0);

    public LightsSubsystem() {
        CANdleConfiguration candleConfiguration = new CANdleConfiguration();
        candleConfiguration.statusLedOffWhenActive = true;
        candleConfiguration.disableWhenLOS = false;
        candleConfiguration.stripType = LEDStripType.RGB;
        candleConfiguration.brightnessScalar = 1.0;
        candleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;
        candle.configAllSettings(candleConfiguration, 100);
    }

    public void setBrightness(double percent) {
        candle.configBrightnessScalar(percent, 100);
    }

    public Command defaultCommand() {
        return runOnce(() -> {
            LEDSegment.MainStrip.setFlowAnimation(orange, 1);
        });
    }

    public Command clearSegmentCommand(LEDSegment segment) {
        return runOnce(() -> {
            segment.clearAnimation();
            segment.disableLEDs();
        });
    }

    public static enum LEDSegment {
        CANdle(0, 8, 0),
        MainStrip(8, 300, 1);

        public final int startIndex;
        public final int segmentSize;
        public final int animationSlot;

        // private enum constructor
        private LEDSegment(int startIndex, int segmentSize, int animationSlot) {
            this.startIndex = startIndex;
            this.segmentSize = segmentSize;
            this.animationSlot = animationSlot;
        }

        public void setColor(Color color) {
            clearAnimation();
            candle.setLEDs(color.red, color.green, color.blue, 0, startIndex, segmentSize);
        }

        private void setAnimation(Animation animation) {
            candle.animate(animation, animationSlot);
        }

        public void clearAnimation() {
            candle.clearAnimation(animationSlot);
        }

        public void disableLEDs() {
            setColor(black);
        }

        public void setFlowAnimation(Color color, double speed) {
            setAnimation(new ColorFlowAnimation(
                    color.red, color.green, color.blue, 0, speed, segmentSize, Direction.Forward, startIndex));
        }

        public void setFadeAnimation(Color color, double speed) {
            setAnimation(
                    new SingleFadeAnimation(color.red, color.green, color.blue, 0, speed, segmentSize, startIndex));
        }

        public void setBandAnimation(Color color, double speed) {
            setAnimation(new LarsonAnimation(
                    color.red, color.green, color.blue, 0, speed, segmentSize, BounceMode.Front, 3, startIndex));
        }

        public void setStrobeAnimation(Color color, double speed) {
            setAnimation(new StrobeAnimation(color.red, color.green, color.blue, 0, speed, segmentSize, startIndex));
        }

        public void setRainbowAnimation(double speed) {
            setAnimation(new RainbowAnimation(1, 0.5, segmentSize, false, startIndex));
        }
    }

    public static class Color {
        public int red;
        public int green;
        public int blue;

        public Color(int red, int green, int blue) {
            this.red = red;
            this.green = green;
            this.blue = blue;
        }
    }
}
