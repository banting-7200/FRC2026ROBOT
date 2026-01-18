package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilites.LEDRequest;

//LED PRIORITY LIST

// TODO Implement Elastic notifications
// -1 - Robot Disabled

// 0 - Has gamePiece / Done command

// 1 - Aligning to aprilTag (Fine adjustment/PID)

// 2 - Aligning to aprilTag/GamePiece (Path following)

// 3 - Ready to do something

// 4 - Creep Drive mode

// 5 - Normal driving

public class LightsSubsystem extends SubsystemBase {

    private List<LEDRequest> requests = new ArrayList<>();
    private LEDRequest currentRequest = new LEDRequest(LEDRequest.LEDState.OFF).withBlinkRate(0)
            .withColour(Color.kWhite).withPriority(Integer.MAX_VALUE);
    private AddressableLED ledInstance;
    private AddressableLEDBuffer bufferInstance;
    private int rainbowFirstPixelHue = 0;
    private double lastReadTimestamp = Timer.getFPGATimestamp();
    private boolean lightsAreOn = false;

    public LightsSubsystem(int lightPort, int lightCount) {
        ledInstance = new AddressableLED(lightPort);
        bufferInstance = new AddressableLEDBuffer(lightCount);
        ledInstance.setLength(lightCount);
        ledInstance.start();
    }

    public void requestLEDState(LEDRequest newRequest) {
        requests.add(newRequest);
    }

    public LEDRequest getLEDRequest(){
        return currentRequest;
    }

    public void run() {

        if (requests.isEmpty()) {
            off();
        } else if (requests.size() == 1) {
            currentRequest = requests.get(0);
        } else {
            // no idea if this works, get the LEDState with the lowest int for Priority
            currentRequest = requests.stream().min(Comparator.comparingInt(LEDRequest::getPriority))
                    .orElse(requests.get(0));
        }

        switch (currentRequest.getState()) {
            case OFF:
                off();
                break;
            case SOLID:
                solidColor(currentRequest.getColour());
                break;
            case BLINK:
                blink(currentRequest.getBlinkRate(), currentRequest.getColour());
                break;
            case RAINBOW:
                rainbow();
                break;
            default:
            break;
        }
        requests.clear();
    }

    private void blink(double blinkRate, Color color) {
        double now = Timer.getFPGATimestamp();
        if (now - lastReadTimestamp > blinkRate / 2) {
            lastReadTimestamp = now;
            lightsAreOn = !lightsAreOn;
        }

        if (lightsAreOn) {
            for (int x = 0; x < bufferInstance.getLength(); x++) {
                bufferInstance.setLED(x, color);
            }
        } else {
            off();
        }
        ledInstance.setData(bufferInstance);
    }

    private void solidColor(Color color) {
        for (int x = 0; x < bufferInstance.getLength(); x++) {
            bufferInstance.setLED(x, color);
        }
        ledInstance.setData(bufferInstance);
    }

    private void rainbow() {
        for (var i = 0; i < bufferInstance.getLength(); i++) {
            int hue = (rainbowFirstPixelHue + (i * 180 / bufferInstance.getLength())) % 180;
            bufferInstance.setHSV(i, hue, 255, 128);
        }
        rainbowFirstPixelHue += 3;
        rainbowFirstPixelHue %= 180;
        ledInstance.setData(bufferInstance);
    }

    private void off() {
        currentRequest.withColour(Color.kBlack);
        for (int i = 0; i < bufferInstance.getLength(); i++) {
            bufferInstance.setLED(i, Color.kBlack);
        }
        ledInstance.setData(bufferInstance);
    }
}
