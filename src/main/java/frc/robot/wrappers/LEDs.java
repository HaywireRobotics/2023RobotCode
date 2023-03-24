package frc.robot.wrappers;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;

public class LEDs {

    private final AddressableLED leds;
    private final AddressableLEDBuffer buffer;
    private Timer timer = new Timer();

    private double blinkRate = 0.5;
    private double lastTime = 0.0;
    private Color[] cycleColors = new Color[0];
    private int colorIndex = 0;
    // private Modes mode = Modes.SOLID;

    private final int numLeds = 150;

    private final Color coneColor = new Color(255, 150, 0);
    private final Color cubeColor = Color.kPurple;
    private Color gamepieceColor = coneColor;
    // starts ambiguous, will match alliance when driverstation connects
    public Color allianceColor = Color.kPurple;

    public LEDs(int port){
        leds = new AddressableLED(port);
        buffer = new AddressableLEDBuffer(numLeds);
        leds.setLength(buffer.getLength());

        leds.setData(buffer);
        leds.start();

        setSolid(allianceColor);
    }

    public void setAllToColor(Color c){
        for(int i = 0; i < buffer.getLength(); i++){
            buffer.setLED(i, c);
        }
        updateLEDs();
    }

    public void setSolid(Color c){
        setAllToColor(c);
        cycleColors = new Color[1];
        cycleColors[0] = c;
    }

    public void setCycle(Color[] c, double t){
        setAllToColor(c[0]);
        cycleColors = c;
        blinkRate = t;
    }

    public void setBlink(Color c){
        setAllToColor(c);
        cycleColors = new Color[2];
        cycleColors[0] = c;
        cycleColors[1] = Color.kBlack;
    }

    public void update(){
        double currentTime = timer.get();

        if (currentTime - lastTime > blinkRate) {
            setAllToColor(cycleColors[colorIndex]);
            colorIndex++;
            if (colorIndex >= cycleColors.length) {
                colorIndex = 0;
            }
            lastTime = currentTime;
        }

        lastTime = currentTime;
    }
    private void updateLEDs(){
        leds.setData(buffer);
        leds.start();
    }

    public void setAllianceColor(Color color) {
        allianceColor = color;
        setSolid(allianceColor);
    }

    public void toggleColor() {
        if (gamepieceColor == coneColor) {
            gamepieceColor = cubeColor;
        } else {
            gamepieceColor = coneColor;
        }
        setSolid(gamepieceColor);
    }

    public void setSolidGamePieceColor() {
        setSolid(gamepieceColor);
    }

    public void blinkGamePieceColor() {
        setBlink(gamepieceColor);
    }
}
