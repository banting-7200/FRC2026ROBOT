package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilites.LEDRequest;
import frc.robot.Utilites.LEDRequest.LEDState;
import java.util.PriorityQueue;
import java.util.stream.IntStream;

// LED PRIORITY LIST

// TODO Implement Elastic notifications
// -1 - Robot Disabled

// 0 - Has gamePiece / Done command

// 1 - Aligning to aprilTag (Fine adjustment/PID)

// 2 - Aligning to aprilTag/GamePiece (Path following)

// 3 - Ready to do something

// 4 - Creep Drive mode

// 5 - Normal driving

public class LightsSubsystem extends SubsystemBase {

  private PriorityQueue<LEDRequest> requests = new PriorityQueue<LEDRequest>();
  private AddressableLED ledInstance;
  private AddressableLEDBuffer bufferInstance;
  private int rainbowFirstPixelHue = 0;
  private double lastReadTimestamp = Timer.getFPGATimestamp();
  private boolean lightsAreOn = false;
  private Color currentColour = null;
  private LEDState currentState = LEDState.OFF;
  private LEDRequest currentRequest = null;

  public LightsSubsystem(int lightPort, int lightCount, AddressableLED.ColorOrder colourOrder) {
    ledInstance = new AddressableLED(lightPort);
    bufferInstance = new AddressableLEDBuffer(lightCount);
    ledInstance.setColorOrder(colourOrder);
    ledInstance.setLength(lightCount);
    ledInstance.start();
  }

  public LightsSubsystem(int lightPort, int lightCount) {
    this(lightPort, lightCount, ColorOrder.kRGB);
  }

  public void requestLEDState(LEDRequest newRequest) {
    requests.add(newRequest);
  }

  /**
   * Get the currently set colour
   *
   * @return Current colour. If current state is not {@code SOLID} or {@code OFF} then {@code null}
   *     is returned.
   */
  public Color getColour() {
    return currentColour;
  }

  /**
   * Get current colour as a Hex string (e.g. for use with dashboards)
   *
   * @return Current colours as an array of hex strings
   */
  public String[] getHexColour() {
    if (currentState == LEDState.RAINBOW) {
      // Send a static rainbow array
      return new String[] {
        "#e81416", "#ffa500", "#faeb36", "#79c314", "#487de7", "#4b369d", "#70369d"
      };
    } else {
      return new String[] {currentColour.toHexString()};
    }
  }

  public LEDState getLEDState() {
    return currentState;
  }

  /**
   * Set all LEDs on the strip to one colour.
   *
   * @param colour Colour for all LEDs in the strip
   */
  private void setAll(Color colour) {
    IntStream.range(0, bufferInstance.getLength()).forEach(i -> bufferInstance.setLED(i, colour));
  }

  public void run() {
    // Pop the highest priority (i.e. lowest numerical priority value) item off the queue
    currentRequest = requests.poll();

    if (currentRequest == null) {
      // Queue is empty, LEDs go off
      solidColor(Color.kBlack);
      currentState = LEDState.OFF;
      return;
    }

    switch (currentRequest.getState()) {
      case SOLID:
        solidColor(currentRequest.getColour());
        currentState = LEDState.SOLID;
        break;
      case BLINK:
        blink(currentRequest.getBlinkRate(), currentRequest.getColour());
        currentState = LEDState.BLINK;
        break;
      case RAINBOW:
        rainbow();
        currentState = LEDState.RAINBOW;
        break;
      default:
        solidColor(Color.kBlack);
        currentState = LEDState.OFF;
        break;
    }

    requests.clear();
  }

  private void blink(double blinkRate, Color colour) {
    double now = Timer.getFPGATimestamp();
    if (now - lastReadTimestamp > blinkRate / 2) {
      lastReadTimestamp = now;
      lightsAreOn = !lightsAreOn;
    }

    if (lightsAreOn) {
      solidColor(colour);
    } else {
      solidColor(Color.kBlack);
    }

    ledInstance.setData(bufferInstance);
  }

  private void solidColor(Color colour) {
    setAll(colour);
    ledInstance.setData(bufferInstance);
    currentColour = colour;
  }

  private void rainbow() {
    for (var i = 0; i < bufferInstance.getLength(); i++) {
      int hue = (rainbowFirstPixelHue + (i * 180 / bufferInstance.getLength())) % 180;
      bufferInstance.setHSV(i, hue, 255, 128);
    }
    rainbowFirstPixelHue += 3;
    rainbowFirstPixelHue %= 180;
    ledInstance.setData(bufferInstance);
    currentColour = null;
  }
}
