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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase{
  private final AddressableLED m_led = new AddressableLED(9);
  // LED's go into PWM #9
  private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);
  private int m_rainbowFirstPixelHue;

  @Override
  public void periodic() {
    if (DriverStation.isEnabled()) {
      //Alliance ally = DriverStation.getAlliance();
      Alliance ally = Alliance.Red;
      if (ally.equals(Alliance.Red)) {
        m_ledBuffer.setRGB(0,255,0,0);
        // If alliance = red, set strips to red
      }
      else if (ally.equals(Alliance.Blue)) { // should only have pipelines 0 & 1
        m_ledBuffer.setRGB(0,0,0,255);
        // If allience = blue, set strips blue
      }
    }
  }


  @Override
  public void simulationPeriodic() {

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