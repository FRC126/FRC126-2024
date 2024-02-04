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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**********************************************************************************
 **********************************************************************************/

 public class LEDSubsystem extends SubsystemBase {
    AddressableLED m_led;
    AddressableLEDBuffer m_ledBuffer;
    final int LENGTH=150;
    int m_rainbowFirstPixelHue,
        rotateLED=0,
        delay=0;

    public static enum LEDModes{None,BrakeMode,DriveMode,SlowMode,AimingSpeaker,AimingAmp,
                                ShootingSpeaker,ShootingAmp,Climbing,AutoDrive,
                                AutoTurn,RunPickup, Rainbow, GaelForce};
    LEDModes LEDMode = LEDModes.GaelForce;

    /**********************************************************************************
     **********************************************************************************/

    public LEDSubsystem() {
        m_led = new AddressableLED(9);

        // LED's go into PWM #9
        m_ledBuffer = new AddressableLEDBuffer(LENGTH);

        m_led.setLength(m_ledBuffer.getLength());
    
        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
    }  

    /**********************************************************************************
     **********************************************************************************/

    public void setMode(LEDModes mode) {

        if (LEDMode == LEDModes.None) {
            LEDMode=mode;
        } else if (LEDMode == LEDModes.BrakeMode ||
                   LEDMode == LEDModes.DriveMode || 
                   LEDMode == LEDModes.GaelForce || 
                   LEDMode == LEDModes.SlowMode ) {
            // Allow other modes to override the drive mode
            LEDMode=mode;
        }    
    }

    /**********************************************************************************
     **********************************************************************************/

    public LEDModes getMode() {
        return(LEDMode);
    }

    /**********************************************************************************
     **********************************************************************************/

    public void doLights() {
        if (--delay < 0) { delay = 0; }

        switch (LEDMode) {
            case None:            { setColorSliding(0,0,0); break; }
            case BrakeMode:       { setColorSliding(255,0,0); break; }
            case DriveMode:           { setColorSliding(0,255,0); break; }
            case SlowMode:        { setColorSliding(255,255,0); break; }
            case AimingSpeaker:   { setColorSliding(0,255,0,0,0,255); break; }
            case AimingAmp:       { setColorSliding(0,255,0,255,0,0); break; }
            case ShootingSpeaker: { setColorSliding(255,0,255,0,0,255); break; }
            case ShootingAmp:     { setColorSliding(255,0,255,255,0,0); break; }
            case Climbing:        { break; }
            case AutoDrive:       { setColorSliding(255,153,204,0,0,255); break; }
            case AutoTurn:        { setColorSliding(255,153,204,255,0,0); break; }
            case RunPickup:       { break; }
            case Rainbow:         { rainbow(); break; }
            case GaelForce:       { setColorSliding(0, 102, 51, 255, 255, 0); break; }
        }
    }

    /**********************************************************************************
     **********************************************************************************/

     @Override
    public void periodic() {
    }

    /**********************************************************************************
     **********************************************************************************/
    
    public void setColorSliding(int red, int green, int blue) {
        setColorSliding(red, green, blue, red, green, blue);
    }

    /**********************************************************************************
     **********************************************************************************/
    
    public void setColorSliding(int red, int green, int blue, 
                                int redSlide, int greenSlide, int blueSlide) {
        if (delay > 0) { return; }

        for (int i = 0; i < LENGTH; i++) {
            m_ledBuffer.setRGB(i, red, green, blue);
        }

        colorSliders(rotateLED, redSlide, greenSlide, blueSlide);
        updateLights();
    }    

    /**********************************************************************************
     **********************************************************************************/

    public void colorSliders(int start, int red, int green, int blue) {
        colorSlider(rotateLED, red, green, blue);
        colorSlider((rotateLED+(LENGTH/4))%LENGTH, red, green, blue);
        colorSlider((rotateLED+(LENGTH/2))%LENGTH, red, green, blue);
        colorSlider((rotateLED+(LENGTH/2)+(LENGTH/4))%LENGTH, red, green, blue);
    }

    /**********************************************************************************
     * 1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19 
     * .9  .8  .7  .6  .5  .4  .3  .2  .1  0  .1  .2  .3  .4  .5  .6  .7  .8  .9
     **********************************************************************************/

    public void colorSlider(int start, int red, int green, int blue) {
        double step=1.0;
        for ( int i=start; i <= start+10; i++, step-=.1) {
            m_ledBuffer.setRGB(((start+i)%LENGTH), (int)(red*step), (int)(green*step), (int)(blue*step));
        }
        for ( int i=start+11; i <= start+19; i++, step+=.1) {
            m_ledBuffer.setRGB(((start+i)%LENGTH), (int)(red*step), (int)(green*step), (int)(blue*step));
        }
    }

    /**********************************************************************************
     **********************************************************************************/

    public void rainbow() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
            m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        m_rainbowFirstPixelHue += 3;
        m_rainbowFirstPixelHue %= 180;
        m_led.setData(m_ledBuffer);
    }

    /**********************************************************************************
     **********************************************************************************/

    public void updateLights() {
        rotateLED = (rotateLED + 1) % LENGTH;
        delay=5;

        m_led.setData(m_ledBuffer);
    }
}