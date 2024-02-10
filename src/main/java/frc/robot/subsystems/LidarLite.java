/**********************************
	   _      ___      ____
	 /' \   /'___`\   /'___\
	/\_, \ /\_\ /\ \ /\ \__/
	\/_/\ \\/_/// /__\ \  _``\
	   \ \ \  // /_\ \\ \ \L\ \
	    \ \_\/\______/ \ \____/
		 \/_/\/_____/   \/___/

    Team 126 2024 Code       
	Go get em gaels!

***********************************/

package frc.robot.subsystems;

import frc.robot.commands.*;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**********************************************************************************
 **********************************************************************************/

public class LidarLite extends SubsystemBase {
    /*
    * Adjust the Calibration Offset to compensate for differences in each unit.
    * You can also use the offset to zero out the distance between the sensor and edge of the robot.
    */
    private static final int CALIBRATION_OFFSET = -5;
    private double[] dataSeries;
    private static final int HEIGHT_IN = 83;
    private static final int SUBHEIGHT_IN = 24;
    private Counter counter;
    private double distanceAvg=0;

	/**********************************************************************************
	 **********************************************************************************/
	
    public double getDistanceAvg() {
        return distanceAvg;
    } 

	/**********************************************************************************
     * Create an object for a LIDAR-Lite attached to some digital input on the roboRIO
     * 
     * @param source The DigitalInput or DigitalSource where the LIDAR-Lite is attached (ex: new DigitalInput(9))
	 **********************************************************************************/
	
    public LidarLite (DigitalSource source) {
        CommandScheduler.getInstance().registerSubsystem(this);
        setDefaultCommand(new DistanceMeasure(this));

        counter = new Counter(source);
        counter.setMaxPeriod(1.0);
        // Configure for measuring rising to falling pulses
        counter.setSemiPeriodMode(true);
        counter.reset();
        dataSeries = new double[10];
    }

	/**********************************************************************************
	 **********************************************************************************/

    @Override
    public void periodic() {
        measureDistance();  
    }

	/**********************************************************************************
     * Take a measurement and return the distance in cm
     * 
     * @return Distance in cm
	 **********************************************************************************/
	
    public double measureDistance() {
        double cm;
        /* If we haven't seen the first rising to falling pulse, then we have no measurement.
        * This happens when there is no LIDAR-Lite plugged in, btw.
        */
        if (counter.get() < 1) {
            return 0;
        }
        /* getPeriod returns time in seconds. The hardware resolution is microseconds.
        * The LIDAR-Lite unit sends a high signal for 10 microseconds per cm of distance.
        */
        cm = (counter.getPeriod() * 1000000.0 / 10.0) + CALIBRATION_OFFSET;
        
        for (int x=1; x<10; x=x+1) {
            dataSeries[x-1] = dataSeries[x];
        }
        dataSeries[9] = cm;
        
        double sum=0;
        double count=0;
        for (int x=0; x<10; x=x+1) {
            if (dataSeries[x] != 0) {
                count++;
                sum += dataSeries[x];
            } 
        }
            
        distanceAvg = sum / count;
        double distanceInch = distanceAvg*0.39370079;
        SmartDashboard.putNumber("LidarLite Distance",distanceInch);
        SmartDashboard.putNumber("Thrower Angle", calcAngle(distanceInch, HEIGHT_IN,SUBHEIGHT_IN));

        return distanceAvg;
    }
    public static double calcAngle(double x, double height, double subHeight) {
        
        return Math.toDegrees(Math.atan((height - subHeight)/(x)));
    }
}
