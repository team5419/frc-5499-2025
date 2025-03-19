package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import org.littletonrobotics.junction.Logger;

public class LightsSubsystem extends SubsystemBase {
  private final AddressableLED leds_left;
  // private final AddressableLED leds_right;
  private AddressableLEDBuffer left_buffer;
  // private AddressableLEDBuffer right_buffer;

  // // all hues at maximum saturation and half brightness
  private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 255);

  // // Our LED strip has a density of 30 LEDs per meter
  private static final Distance kLedSpacing = Meters.of(1 / 30.0);

  // // Create a new pattern that scrolls the rainbow pattern across the LED strip, moving at a
  // speed
  // // of 1 meter per second.
  private final LEDPattern m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);

  public LightsSubsystem() {
    leds_left = new AddressableLED(RobotMap.LED_LEFT);
    // leds_right = new AddressableLED(RobotMap.LED_RIGHT);

    left_buffer = new AddressableLEDBuffer(30);
    // right_buffer = new AddressableLEDBuffer(30);

    leds_left.setLength(left_buffer.getLength());
    // leds_right.setLength(right_buffer.getLength());

    leds_left.setData(left_buffer);
    // leds_right.setData(right_buffer);

    leds_left.start();
    // leds_right.start();
  }

  @Override
  public void periodic() {
    m_scrollingRainbow.applyTo(left_buffer);
    // m_scrollingRainbow.applyTo(right_buffer);

    leds_left.setData(left_buffer);
    // leds_right.setData(right_buffer);

    Logger.recordOutput("Lights Subsystem/Left LED Buffer", left_buffer.toString());
    // Logger.recordOutput("Lights Subsystem/Right LED Buffer", right_buffer.toString());
  }
}
