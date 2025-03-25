package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Map;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class LightsSubsystem extends SubsystemBase {
  private final AddressableLED leds;
  private AddressableLEDBuffer buffer;

  // Our LED strip has a density of 30 LEDs per meter
  private static final Distance ledSpacing = Meters.of(1 / 30.0);

  private boolean isEnabledPrev = false;

  // ---------- LED Patterns ----------
  // Default pattern
  private final LEDPattern defaultPattern = LEDPattern.solid(Color.fromHSV(0, 0, 100));

  // Rainbow pattern
  private final LEDPattern rainbowPattern = LEDPattern.rainbow(255, 255);

  // Disabled gradient
  private final LEDPattern disabledPattern = LEDPattern.gradient(
    GradientType.kContinuous,
    Color.kRed,
    Color.kOrange
  );

  // L1 pattern
  private final LEDPattern l1Pattern = LEDPattern.steps(Map.of(
    0, Color.kRed,
    0.33, Color.kOrange,
    0.66, Color.kOrange
  ));

  // L2 pattern
  private final LEDPattern l2Pattern = LEDPattern.steps(Map.of(
    0, Color.kOrange,
    0.33, Color.kRed,
    0.66, Color.kOrange
  ));

  // L3 pattern
  private final LEDPattern l3Pattern = LEDPattern.steps(Map.of(
    0, Color.kOrange,
    0.33, Color.kOrange,
    0.66, Color.kRed
  ));

  public static enum LightsState {
    DISABLED, IDLE, INTAKE, CLIMBING, L1, L2, L3,
  }

  private LightsState currentState = LightsState.DISABLED;

  public LightsSubsystem() {
    leds = new AddressableLED(RobotMap.LED_STRIP);

    buffer = new AddressableLEDBuffer(30);

    leds.setLength(buffer.getLength());

    leds.setData(buffer);

    leds.start();
  }

  public void setState(LightsState state) {
    currentState = state;
  }

  private LEDPattern getScrollPattern(LEDPattern pattern) {
    return pattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), ledSpacing);
  }

  @Override
  public void periodic() {
    LEDPattern pattern = defaultPattern;

    isEnabledPrev = DriverStation.isEnabled();
    if (isEnabledPrev != DriverStation.isEnabled()) {
      currentState = DriverStation.isEnabled() ? LightsState.IDLE : LightsState.DISABLED;
    }

    switch (currentState) {
      case DISABLED:
        pattern = getScrollPattern(disabledPattern);
        break;
      case L1:
        pattern = l1Pattern;
        break;
      case L2:
        pattern = l2Pattern;
        break;
      case L3:
        pattern = l3Pattern;
        break;
      default:
        pattern = getScrollPattern(rainbowPattern);
        break;
    }

    pattern.applyTo(buffer);

    leds.setData(buffer);
  }
}
