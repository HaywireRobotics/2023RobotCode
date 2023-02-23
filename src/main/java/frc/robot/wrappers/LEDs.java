package frc.robot.wrappers;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDs {
    
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;

    public static enum Modes {
        SOLID,
        BLINK,
        CYCLE,
    }

    private Timer timer = new Timer();

    private Modes mode = Modes.SOLID;
    private double blinkRate = 0.5;
    private double lastTime = 0.0;
    private Color[] cycleColors = new Color[0];
    private Color staticColor = new Color(0, 0, 0);

    private int colorIndex = 0;

    public LEDs(int port, int numLeds) {
        m_led = new AddressableLED(port);
        m_ledBuffer = new AddressableLEDBuffer(numLeds);
        m_led.setLength(m_ledBuffer.getLength());

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
                        setAllToColor(new Color(0, 0, 0));
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
    
    private void setAllToColor(Color c) {
        mode = Modes.SOLID;
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setLED(i, c);
        }
        m_led.setData(m_ledBuffer);
    }

    public void setSolid(Color c) {
        mode = Modes.SOLID;
        staticColor = c;
        colorIndex = 0;
    }
    public void setSolid(int r, int g, int b) {
        setSolid(new Color(r, g, b));
    }

    public void setBlink(Color c, double rate) {
        mode = Modes.BLINK;
        blinkRate = rate;
        colorIndex = 0;
    }
    public void setBlink(int r, int g, int b, double rate) {
        setBlink(new Color(r, g, b), rate);
    }

    public void setCycle(Color[] c, double rate) {
        mode = Modes.CYCLE;
        blinkRate = rate;
        cycleColors = c;
        colorIndex = 0;
    }
}
