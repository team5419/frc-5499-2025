package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Lights extends SubsystemBase {
  private final AddressableLED leds;
  private AddressableLEDBuffer buffer;

  // // all hues at maximum saturation and half brightness
  private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 255);

  // // Our LED strip has a density of 120 LEDs per meter
  private static final Distance kLedSpacing = Meters.of(1 / 30.0);

  // // Create a new pattern that scrolls the rainbow pattern across the LED strip, moving at a
  // speed
  // // of 1 meter per second.
  private final LEDPattern m_scrollingRainbow =
      m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);

  public Lights() {
    leds = new AddressableLED(RobotMap.LED_STRIP);
    buffer = new AddressableLEDBuffer(30);

    leds.setLength(buffer.getLength());
    leds.setData(buffer);
    leds.start();
  }

  @Override
  public void periodic() {
    // Update the buffer with the rainbow animation
    m_scrollingRainbow.applyTo(buffer);
    // for (int i = 0; i < buffer.getLength(); i++) {
    //   buffer.setHSV(i, 90, 255, 255);
    // }
    // Set the LEDs
    // set all leds to purple
    // for (int i = 0; i < buffer.getLength(); i++) {
    //   buffer.setHSV(i, 270, 255, 255);
    //   System.out.println("setting led " + i + " to purple");
    // }
    // LEDPattern blue = LEDPattern.solid(Color.kCornflowerBlue);
    // blue.applyTo(buffer);
    leds.setData(buffer);
  }
}
