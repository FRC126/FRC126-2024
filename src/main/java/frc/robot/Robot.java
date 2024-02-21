// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

package frc.robot;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.DriverStation;

// Navx-MXP Libraries and Connection Library
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
    // Global Robot Variables
    public int RobotID = 1;  
    public static boolean useNavx=true;

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // NavX-MXP
    public static AHRS navxMXP;

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // Automation Variables
    public static targetTypes targetType = Robot.targetTypes.NoTarget;
    public static boolean autoMoveThrower = false;

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // Auto Routines
    public static boolean isAutoCommand=false;
    public static SequentialCommandGroup autoCommand;

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // Subsystems
    public static Controllers oi;
    public static Log log;
    public static InternalData internalData;
    public static SwerveDrive swerveDrive;
    public static Thrower thrower;    
    public static Climber climber;
    public static Pickup pickup;
    public static LEDSubsystem Leds;
    // Lidar Light Distance Measure
    public static LidarLite lidar;
    // Lime Light
    public static LimeLight limeLight;

	public static UsbCamera driveCam;
	public static VideoSink server;

    public static enum targetTypes{
        NoTarget(-1),TargetSeek(0), TargetOne(1), TargetTwo(2), TargetThree(3), TargetFour(4);
        private final int pipeline;
        private targetTypes(int v) {pipeline = v;}
        public int getPipeline() {
            return pipeline;
        }
    };
    public static enum allianceColor{Red,Blue};

    // Autonomous related functions
    public static SequentialCommandGroup autonomous;
    public static boolean autoMove=false;

    int selectedAutoPosition;
	int selectedAutoFunction;
    int selectedautoFollow;
    int selectedAllianceColor;
	
    private final SendableChooser<Integer> autoFunction = new SendableChooser<>();
    private final SendableChooser<Integer> autoPosition = new SendableChooser<>();
    private final SendableChooser<Integer> autoFollow = new SendableChooser<>();
    private final SendableChooser<Integer> allianceColor = new SendableChooser<>();

    public static final String PICKUP_MOTOR_SPEED_STRING = "Pickup Motor Speed";
    public static final String COMPETITION_ROBOT = "Competition Robot";
    
 	  /************************************************************************
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
	   ************************************************************************/
    @Override
    public void robotInit() {
        // Set the robot id for use by RobotMap
        RobotMap.setRobot(RobotID);

        // Enable the command scheduler
        CommandScheduler.getInstance().enable();

        // Create and register the robot Subsystems
        oi = new Controllers();
        log = new Log();
        internalData = new InternalData();

        // Swerve drive subsystem 
        swerveDrive = new SwerveDrive();

        // Thrower Devices
        thrower = new Thrower();
  
        // Pickup SubSystem
        pickup = new Pickup();

        // LED Subsystem
        Leds = new LEDSubsystem();

        // Limelight subsystem
        limeLight = new LimeLight();
       
        // Navx Subsystem
        try {
            navxMXP = new AHRS(SPI.Port.kMXP);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }
    
        // Initialize the built in gyro
        internalData.initGyro();
        internalData.resetGyro();

        // create the lidarlite class
        lidar = new LidarLite();

        // Server for the drive camera
        //driveCam = CameraServer.startAutomaticCapture();
		//server = CameraServer.getServer();
        //driveCam.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
		//server.setSource(driveCam);

        // Dashboard Cooser for the Autonomous mode move
        autoFunction.setDefaultOption("Speaker Shot",0);
        autoFunction.addOption("Amplifier",1);
        SmartDashboard.putData("Auto Target",autoFunction);

        // Dashboard Cooser for the Autonomous mode position
        autoPosition.setDefaultOption("Position 0",0);
        autoPosition.addOption("Position 1",1);
        autoPosition.addOption("Position 2",2);
        SmartDashboard.putData("Auto Robot Position",autoPosition);

        // Dashboard Cooser for the Autonomous mode position
        allianceColor.setDefaultOption("Red Alliance",0);
        allianceColor.addOption("Blue Alliance",1);
        SmartDashboard.putData("Alliance Color",allianceColor);
        
        autoFollow.setDefaultOption("1 note",0);
        autoFollow.addOption("2 note",1);
        autoFollow.addOption("3 note",2);
        SmartDashboard.putData("Auto Follow Choices",autoFollow);
        SmartDashboard.putNumber(PICKUP_MOTOR_SPEED_STRING,1);
        SmartDashboard.putBoolean(COMPETITION_ROBOT, false);
           

        Log.print(0, "Git Info", "branch: %s buildDate: %s gitDate: %s sha: %s".formatted(
            BuildConstants.GIT_BRANCH,
            BuildConstants.BUILD_DATE,
            BuildConstants.GIT_DATE,
            BuildConstants.GIT_SHA
            ));
        Log.print(0, "Robot", "Robot Init Complete");
        // Put information above onto Smart Dashboard
        SmartDashboard.putString("Git Branch", BuildConstants.GIT_BRANCH);
        SmartDashboard.putString("Build Date", BuildConstants.BUILD_DATE);
    }

 	  /************************************************************************
	   * This function is run once each time the robot enters autonomous mode. 
     ************************************************************************/
    @Override
    public void autonomousInit() {
        Log.print(0, "Robot", "Robot Autonomous Init");

        Robot.stopAutoCommand();
		Robot.swerveDrive.cancel();

        try {
			selectedAutoPosition = (int) autoPosition.getSelected();
		} catch(NullPointerException e) {
			selectedAutoPosition = 0;
		}
		try {
			selectedAutoFunction = (int)autoFunction.getSelected();
		} catch(NullPointerException e) {
			selectedAutoFunction = 0;
		}
		try {
			selectedautoFollow = (int)autoFollow.getSelected();
		} catch(NullPointerException e) {
			selectedautoFollow = 0;
		}
		try {
			selectedAllianceColor = (int)allianceColor.getSelected();
		} catch(NullPointerException e) {
			selectedAllianceColor = 0;
		}

        switch (selectedAutoFunction) {
            case 0:
                // Speaker Shot
                //autonomous = new AutoShootSpeaker(selectedAutoPosition, selectedautoFollow);    
                SmartDashboard.putString("AutoCommand","Speaker");
                break;
            case 1:
                // Amplifier Shot
                //autonomous = new AutoShootAmp(selectedAutoPosition, selectedautoFollow);    
                SmartDashboard.putString("AutoCommand","AMP");
                break;
        }

        autonomous.schedule();
    }

    /************************************************************************
     * This function is called periodically during autonomous.
    ************************************************************************/
    @Override
    public void autonomousPeriodic() {
        Robot.Leds.forceMode(LEDSubsystem.LEDModes.None);
        CommandScheduler.getInstance().run();
        Robot.Leds.doLights();
    }

    /************************************************************************
     * This function is called once each time the robot enters teleoperated mode.
    ************************************************************************/
    @Override
    public void teleopInit() { 
        Log.print(0, "Robot", "Robot Teleop Init");
  
        if(autonomous != null){
            // Cancel the auto command if it was created
	          autonomous.cancel();
        }

        Robot.stopAutoCommand();

		Robot.swerveDrive.cancel();
    }

    /************************************************************************
     * This function is called periodically during teleoperated mode.
    ************************************************************************/
    @Override
    public void teleopPeriodic() {
        Robot.Leds.forceMode(LEDSubsystem.LEDModes.None);
        CommandScheduler.getInstance().run();
        Robot.Leds.doLights();
    }

    /************************************************************************
     * This function is called once each time the robot enters test mode.  
    ************************************************************************/
    @Override
    public void testInit() {
        Log.print(0, "Robot", "Robot Test Init");

		Robot.swerveDrive.cancel();
    }  

    /************************************************************************
     * This function is called periodically during test mode.
    ************************************************************************/
   @Override
    public void testPeriodic() {
        CommandScheduler.getInstance().run();
    }

    /************************************************************************
	 ************************************************************************/

    static public boolean checkAutoCommand() {
		if (Robot.internalData.isAuto()) {
            return true;
        }
		
		if (Robot.isAutoCommand) {
			return true;
		}	

        return false;
    }    


    /************************************************************************
	 ************************************************************************/

    static public boolean doAutoCommand() {
		if (Robot.internalData.isAuto()) {
            return false;
        }
		
		if (Robot.isAutoCommand) {
			return false;
		}	

        if (Robot.autoMove) { 
    		Robot.swerveDrive.cancel();
        }

	    Robot.isAutoCommand = true;

   		SmartDashboard.putBoolean("RobotIsAutoCommand",Robot.isAutoCommand);
		return true;
	}

    /************************************************************************
	 ************************************************************************/

	static public void stopAutoCommand() {
        if (Robot.isAutoCommand) {
            Robot.autoCommand.cancel();
		}	
		Robot.isAutoCommand=false;

        if (Robot.internalData.isAuto()) {
            return;
        }

        SmartDashboard.putBoolean("RobotIsAutoCommand",Robot.isAutoCommand);

        if (Robot.autoMove) { 
        	Robot.swerveDrive.cancel();
        }
        Robot.autoMove=false;
	}		

    /************************************************************************
	 ************************************************************************/

    static public double boundSpeed(double speedIn, double highSpeed, double lowSpeed ) {
        double speedOut=speedIn;

        if (speedIn < 0) {
            if (speedIn < highSpeed)  { speedOut = highSpeed; }
            if (speedIn > lowSpeed) { speedOut = lowSpeed; }  
        } else {
            if (speedIn > highSpeed)  { speedOut = highSpeed; }
            if (speedIn < lowSpeed) { speedOut = lowSpeed; }  
        }

        return(speedOut);
    }   
}
