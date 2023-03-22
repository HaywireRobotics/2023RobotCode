package frc.robot.wrappers;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class PWMLEDs {

    private final PWMSparkMax leds;

    public static enum Modes {
        SOLID,
        BLINK,
        CYCLE,
    }
    public static enum Colors{
        HOT_PINK (0.75),
        DARK_RED (0.59),
        RED (0.61),
        ORANGE (0.63),
        GOLD (0.65),
        YELLOW (0.76),
        LAWN_GREEN (0.71),
        LIME (0.73),
        DARK_GREEN (0.75),
        GREEN (0.77),
        BLUE_GREEN (0.79),
        AQUA (0.81),
        SKY_BLUE (0.83),
        DARK_BLUE (0.85),
        BLUE (0.87),
        BLUE_VIOLET (0.89),
        VIOLET (0.91),
        WHITE (0.93),
        GRAY (0.95),
        DARK_GRAY (0.97),
        BLACK (0.99);

        public final double pwm;
        private Colors(double pwm) {
            this.pwm = pwm;
        }
    }

    private Timer timer = new Timer();

    private Modes mode = Modes.SOLID;
    private double blinkRate = 0.5;
    private double lastTime = 0.0;
    private Colors[] cycleColors = new Colors[0];
    private Colors staticColor = Colors.BLACK;

    private int colorIndex = 0;

    private final int port;

    public PWMLEDs(int port) {
        this.port = port;

        leds = new PWMSparkMax(port);

        timer.start();
    }

    public void update(){
        double currentTime = timer.get();
        switch (mode) {
            case SOLID:
                setAllToColor(staticColor);
                break;
            case BLINK:
                if (currentTime - lastTime > blinkRate) {
                    if (colorIndex == 0) {
                        setAllToColor(staticColor);
                        colorIndex = 1;
                    } else {
                        setAllToColor(Colors.BLACK);
                        colorIndex = 0;
                    }
                    lastTime = currentTime;
                }
                break;
            case CYCLE:
                if (currentTime - lastTime > blinkRate) {
                    setAllToColor(cycleColors[colorIndex]);
                    colorIndex++;
                    if (colorIndex >= cycleColors.length) {
                        colorIndex = 0;
                    }
                    lastTime = currentTime;
                }
                break;
        }
    }
    
    private void setAllToColor(Colors c) {
        leds.set(c.pwm);
    }

    public void setSolid(Colors c) {
        mode = Modes.SOLID;
        staticColor = c;
        colorIndex = 0;
    }

    public void setBlink(Colors c, double rate) {
        mode = Modes.BLINK;
        blinkRate = rate;
        colorIndex = 0;
    }

    public void setCycle(Colors[] c, double rate) {
        mode = Modes.CYCLE;
        blinkRate = rate;
        cycleColors = c;
        colorIndex = 0;
    }
}
