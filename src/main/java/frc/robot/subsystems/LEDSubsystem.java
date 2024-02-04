/**********************************
	   _      ___      ____
	 /' \   /'___`\   /'___\
	/\_, \ /\_\ \\ \ /\ \__/
	\/_/\ \\/_/// /__\ \  _``\
	   \ \ \  // /_\ \\ \ \L\ \
	    \ \_\/\______/ \ \____/
		 \/_/\/______/   \/____/

    Team 126 2024 Code       
	    Go get em gaels!

***********************************/


package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase{
  AddressableLED m_led;
  AddressableLEDBuffer m_ledBuffer;
  int m_rainbowFirstPixelHue;

  public LEDSubsystem() {
      m_led = new AddressableLED(9);

      // LED's go into PWM #9
      m_ledBuffer = new AddressableLEDBuffer(150);

      m_led.setLength(m_ledBuffer.getLength());
  
      // Set the data
      m_led.setData(m_ledBuffer);
      m_led.start();
  }  

  @Override
  public void periodic() {
      boolean foo=true;
      
      if (foo) {
          rainbow();
      } else { 
          for (var i = 0; i < m_ledBuffer.getLength(); i++) {
              // Sets the specified LED to the RGB values for red
              m_ledBuffer.setRGB(i, 255, 0, 0);
          }
          m_led.setData(m_ledBuffer);
      }    
  }

  public void rainbow() {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
          m_ledBuffer.setHSV(i, hue, 255, 128);
      }
      m_rainbowFirstPixelHue += 3;
      m_rainbowFirstPixelHue %= 180;
      m_led.setData(m_ledBuffer);
  }
}