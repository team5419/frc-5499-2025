package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Map;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class LightsSubsystem extends SubsystemBase {
  private final AddressableLED leds_left;
  // private final AddressableLED leds_right;
  private AddressableLEDBuffer left_buffer;
  // private AddressableLEDBuffer right_buffer;

  // // Our LED strip has a density of 30 LEDs per meter
  private static final Distance ledSpacing = Meters.of(1 / 30.0);

  // ---------- LED Patterns ----------
  // Default pattern
  private final LEDPattern defaultPattern = LEDPattern.solid(Color.fromHSV(0, 0, 100));

  // Rainbow pattern
  private final LEDPattern rainbowPattern = LEDPattern.rainbow(255, 255);

  // Disabled gradient
  private final LEDPattern disabledPattern = LEDPattern.gradient(
    GradientType.kContinuous,
    Color.fromHSV(23, 100, 0),
    Color.fromHSV(48, 100, 0)
  );

  // L1 pattern
  private final LEDPattern l1Pattern = LEDPattern.steps(Map.of(
    0, Color.fromHSV(23, 100, 100),
    0.33, Color.fromHSV(0, 0, 100),
    0.66, Color.fromHSV(0, 0, 100)
  ));

  // L2 pattern
  private final LEDPattern l2Pattern = LEDPattern.steps(Map.of(
    0, Color.fromHSV(0, 0, 100),
    0.33, Color.fromHSV(32, 100, 100),
    0.66, Color.fromHSV(0, 0, 100)
  ));

  // L3 pattern
  private final LEDPattern l3Pattern = LEDPattern.steps(Map.of(
    0, Color.fromHSV(0, 0, 100),
    0.33, Color.fromHSV(0, 0, 100),
    0.66, Color.fromHSV(48, 100, 100)
  ));

  public static enum LightsState {
    DISABLED, IDLE, INTAKE, CLIMBING, L1, L2, L3,
  }

  private LightsState currentState = LightsState.IDLE;

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

  public void setState(LightsState state) {
    currentState = state;
  }

  private LEDPattern getScrollPattern(LEDPattern pattern) {
    return pattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), ledSpacing);
  }

  @Override
  public void periodic() {
    LEDPattern currentPattern = defaultPattern;
    switch (currentState) {
      case DISABLED: currentPattern = getScrollPattern(disabledPattern);
      case IDLE: currentPattern = getScrollPattern(rainbowPattern);
      case L1: currentPattern = l1Pattern;
      case L2: currentPattern = l2Pattern;
      case L3: currentPattern = l3Pattern;
      default: currentPattern = defaultPattern;
    }

    currentPattern.applyTo(left_buffer);
    // scrollingRainbowPattern.applyTo(right_buffer);

    leds_left.setData(left_buffer);
    // leds_right.setData(right_buffer);
  }
}
