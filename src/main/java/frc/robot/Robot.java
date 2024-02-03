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
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.DriverStation;

// Navx-MXP Libraries and Connection Library
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import com.ctre.phoenix6.hardware.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
    // How many inches we want to drive
    public static final String DISTANCE_DESIRED = "Distance Desired";
    public static final String DISTANCE_ACHIEVED = "Distance Achieved";
    public static final String GEAR_RATIO = "Gear Ratio";
    public static final String WITHIN_FIVE = "Within Five";

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    public static final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
    public static final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
    public static final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

    //public static JoystickWrapper driveJoystick = new JoystickWrapper(Robot.oi.driveController, 0.15);

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // Swerve Motors
    public static CANSparkMax swerveFrontRightDriveMotor = new CANSparkMax(RobotMap.swerveFrontRightDriveCanID, CANSparkMax.MotorType.kBrushless);
    public static CANSparkMax swerveFrontRightTurnMotor = new CANSparkMax(RobotMap.swerveFrontRightTurnCanID, CANSparkMax.MotorType.kBrushless);

    public static CANSparkMax swerveFrontLeftDriveMotor = new CANSparkMax(RobotMap.swerveFrontLeftDriveCanID, CANSparkMax.MotorType.kBrushless);
    public static CANSparkMax swerveFrontLeftTurnMotor = new CANSparkMax(RobotMap.swerveFrontLeftTurnCanID, CANSparkMax.MotorType.kBrushless);

    public static CANSparkMax swerveRearLeftDriveMotor = new CANSparkMax(RobotMap.swerveRearLeftDriveCanID, CANSparkMax.MotorType.kBrushless);
    public static CANSparkMax swerveRearLeftTurnMotor = new CANSparkMax(RobotMap.swerveRearLeftTurnCanID, CANSparkMax.MotorType.kBrushless);

    public static CANSparkMax swerveRearRightDriveMotor = new CANSparkMax(RobotMap.swerveRearRightDriveCanID, CANSparkMax.MotorType.kBrushless);
    public static CANSparkMax swerveRearRightTurnMotor = new CANSparkMax(RobotMap.swerveRearRightTurnCanID, CANSparkMax.MotorType.kBrushless);

    // Built in Motor Encoders
    public static RelativeEncoder swerveFrontRightDriveRelativeEncoder = Robot.swerveFrontRightDriveMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42  );
    public static RelativeEncoder swerveFrontRightTurnRelativeEncoder = Robot.swerveFrontRightTurnMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42  );

    public static RelativeEncoder swerveFrontLeftDriveRelativeEncoder = Robot.swerveFrontLeftDriveMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42  );
    public static RelativeEncoder swerveFrontLeftTurnRelativeEncoder = Robot.swerveFrontLeftTurnMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42  );

    public static RelativeEncoder swerveRearLeftDriveRelativeEncoder = Robot.swerveRearLeftDriveMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42  );
    public static RelativeEncoder swerveRearLeftTurnRelativeEncoder = Robot.swerveRearLeftTurnMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42  );

    public static RelativeEncoder swerveRearRightDriveRelativeEncoder = Robot.swerveRearRightDriveMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42  );
    public static RelativeEncoder swerveRearRightTurnRelativeEncoder = Robot.swerveRearRightTurnMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42  );

    // Swerve Drive CAN Coders
    public static CANcoder SwerveFrontRightEncoder = new CANcoder(RobotMap.SwerveFrontRightEncoderCanID);
    public static CANcoder SwerveFrontLeftEncoder = new CANcoder(RobotMap.SwerveFrontLeftEncoderCanID);
    public static CANcoder SwerveRearRightEncoder = new CANcoder(RobotMap.SwerveRearRightEncoderCanID);
    public static CANcoder SwerveRearLeftEncoder = new CANcoder(RobotMap.SwerveRearLeftEncoderCanID);

    // Pickup CAN Motors
    public static CANSparkMax PickupMotor = new CANSparkMax(RobotMap.PickupCanID, CANSparkMax.MotorType.kBrushless);
    public static RelativeEncoder PickupMotorEncoder = Robot.PickupMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42  );

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // Prototype Motors
    public static CANSparkMax ProtoMotorOne = new CANSparkMax(RobotMap.protoMotorOneCanID, CANSparkMax.MotorType.kBrushless);
    public static CANSparkMax ProtoMotorTwo = new CANSparkMax(RobotMap.protoMotorTwoCanID, CANSparkMax.MotorType.kBrushless);

    public static RelativeEncoder ProtoMotorOneRelativeEncoder = Robot.ProtoMotorOne.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42  );
    public static RelativeEncoder ProtoMotorTwoRelativeEncoder = Robot.ProtoMotorTwo.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42  );

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // Thrower Motors

    public static TalonFX throwerTalonOne = new TalonFX(RobotMap.talonMotorOneCanID);
    public static TalonFX throwerTalonTwo = new TalonFX(RobotMap.talonMotorTwoCanID);

    public static CANSparkMax throwerTriggerMotor = new CANSparkMax(RobotMap.throwerTriggerMotorCanID, CANSparkMax.MotorType.kBrushless);
    public static CANSparkMax throwerClimberMotorLeft = new CANSparkMax(RobotMap.throwerClimberMotorLeftCanID, CANSparkMax.MotorType.kBrushless);
    public static CANSparkMax throwerClimberMotorRight = new CANSparkMax(RobotMap.throwerClimberMotorRightCanID, CANSparkMax.MotorType.kBrushless);

    public static RelativeEncoder throwerTriggerMotorRelativeEncoder = Robot.throwerTriggerMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42  );
    public static RelativeEncoder throwerClimberMotorLeftRelativeEncoder = Robot.throwerClimberMotorLeft.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42  );
    public static RelativeEncoder throwerClimberMotorRightRelativeEncoder = Robot.throwerClimberMotorRight.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42  );

    public static DigitalInput throwerBottomLimit;
    public static DigitalInput throwerTopLimit;

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // NavX-MXP
    public static AHRS navxMXP;

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // Automation Variables
    public static double robotTurn = 0;
	public static double robotDrive = 0;
    public static boolean shootNow = false;
    public static boolean pickupNow = false;
    public static targetTypes targetType = Robot.targetTypes.NoTarget;
    public static int objectId=1;

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
    public static Prototype prototype;
    public static Thrower thrower;    
    public static Pickup pickup;

	public static UsbCamera driveCam;
	public static VideoSink server;
    public static SequentialCommandGroup autonomous;

    // Lidar Light Distance Measure
    public static LidarLite distance;

    // Lime Light
    public static LimeLight limeLight;

    // Global Robot Variables
    public int RobotID = 0;

    public static boolean ignoreEncoders=false;
    public static boolean autoMove=false;

    //public static enum targetHeights{LowTarget,HighTarget};
    public static enum targetTypes{NoTarget,TargetSeek};
    public static enum allianceColor{Red,Blue};
	public static double voltageThreshold = 10.0;

    // For use with limelight class
    public static double ThrowerRPM=0;

    int selectedAutoPosition;
	int selectedAutoFunction;
    int selectedautoFollow;
    int selectedAllianceColor;
	
    private final SendableChooser<Integer> autoFunction = new SendableChooser<>();
    private final SendableChooser<Integer> autoPosition = new SendableChooser<>();
    private final SendableChooser<Integer> autoFollow = new SendableChooser<>();
    private final SendableChooser<Integer> allianceColor = new SendableChooser<>();
    
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
        throwerBottomLimit = new DigitalInput(8);
        throwerTopLimit = new DigitalInput(7);

        // Prototype SubSystem
        prototype = new Prototype();
        pickup = new Pickup();


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

        // create the lidarlite class on DIO 5
        distance = new LidarLite(new DigitalInput(5));

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
        SmartDashboard.putNumber(DISTANCE_DESIRED, 24);
        SmartDashboard.putNumber(GEAR_RATIO, 8.14);

        SmartDashboard.putBoolean(WITHIN_FIVE, false);

        Log.print(0, "Git Info", "branch: %s buildDate: %s gitDate: %s sha: %s".formatted(
            BuildConstants.GIT_BRANCH,
            BuildConstants.BUILD_DATE,
            BuildConstants.GIT_DATE,
            BuildConstants.GIT_SHA
            ));
        Log.print(0, "Robot", "Robot Init Complete");
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
        CommandScheduler.getInstance().run();
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
        CommandScheduler.getInstance().run();
        //driveWithJoystick(true);
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
