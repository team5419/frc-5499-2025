package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Lights extends SubsystemBase {
  private final AddressableLED leds = new AddressableLED(RobotMap.LED_STRIP);
  private AddressableLEDBuffer buffer = new AddressableLEDBuffer(30);

  public Lights() {
    leds.setLength(buffer.getLength());

    leds.setData(buffer);
    leds.start();
  }
}
