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

    Team 126 2023 Code       
	Go get em gaels!

***********************************/

package frc.robot;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource;

import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder;

// import com.ctre.phoenix.motorcontrol.can.*;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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

    // Thrower Motors
    // public static TalonFX throwerMotor1 = new TalonFX(RobotMap.throwerMotorCanID1);
    // public static TalonFX throwerMotor2 = new TalonFX(RobotMap.throwerMotorCanID2);

    // Driver Base Motors
    public static CANSparkMax leftDriveMotor1 = new CANSparkMax(RobotMap.leftDriveMotorCanID1, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static CANSparkMax leftDriveMotor2 = new CANSparkMax(RobotMap.leftDriveMotorCanID2, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static CANSparkMax rightDriveMotor1 = new CANSparkMax(RobotMap.rightDriveMotorCanID1, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static CANSparkMax rightDriveMotor2 = new CANSparkMax(RobotMap.rightDriveMotorCanID2,  CANSparkMaxLowLevel.MotorType.kBrushless); 

    public static DutyCycleEncoder leftDriveEncoder = new DutyCycleEncoder(0);
    public static DutyCycleEncoder rightDriveEncoder = new DutyCycleEncoder(1);

    public static SparkMaxAbsoluteEncoder left1DriveEncoder = Robot.leftDriveMotor1.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    public static SparkMaxAbsoluteEncoder left2DriveEncoder = Robot.leftDriveMotor2.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    public static SparkMaxAbsoluteEncoder right1DriveEncoder = Robot.rightDriveMotor1.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    public static SparkMaxAbsoluteEncoder right2DriveEncoder = Robot.rightDriveMotor2.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    public static CANSparkMax TowerArmMotorLeft = new CANSparkMax(RobotMap.TowerArmMotorLeftID, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static CANSparkMax TowerArmMotorRight = new CANSparkMax(RobotMap.TowerArmMotorRightID, CANSparkMaxLowLevel.MotorType.kBrushless);

    // Lidar Light Distance Measure
    public static LidarLite distance;

    // NavX-MXP
    public static AHRS ahrs;

    // Automation Variables
    public static double robotTurn = 0;
	public static double robotDrive = 0;
    public static boolean shootNow = false;
    public static boolean pickupNow = false;
    public static targetTypes targetType = Robot.targetTypes.NoTarget;
    public static int objectId=1;

    // Auto Routines
    public static boolean isThrowCommand=false;
    public static boolean isAutoBalance=false;
    public static boolean isAutoClimbBalance=false;
    public static boolean isAutoMoveLeft=false;
    public static boolean isAutoMoveRight=false;
    public static boolean isAutoMoveSide=false;

    // Subsystems
    public static Controllers oi;
    public static Log log;
    public static InternalData internalData;
    public static WestCoastDrive driveBase;
    public static TowerArm robotArm;
    public static Grabber robotGrabber;
    public static PixyVision pixyVision;
    public static LimeLight limeLight;
	public static UsbCamera driveCam;
	public static VideoSink server;
    public static SequentialCommandGroup autonomous;
    public static boolean intakeRunning=false;
    public static boolean throwerRunning=false;

    // Global Robot Variables
    public int RobotID = 0;

    //public static DigitalInput rightClimbLimit;
	//public static DigitalInput leftClimbLimit;

    public static enum targetHeights{LowTarget,HighTarget};
    public static enum targetTypes{NoTarget,BallSeek,TargetSeek, PixyTargetSeek};
    public static enum allianceColor{Red,Blue};
	public static double voltageThreshold = 10.0;

    public static Compressor compressor;

    // For use with limelight class
    public static double ThrowerRPM=0;

    int selectedAutoPosition;
	int selectedAutoFunction;
    int selectedAutoBalance;
	
    private final SendableChooser<Integer> autoFunction = new SendableChooser<>();
    private final SendableChooser<Integer> autoPosition = new SendableChooser<>();
    private final SendableChooser<Integer> autoBalance = new SendableChooser<>();
    
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
        driveBase = new WestCoastDrive();
        robotArm = new TowerArm();
        robotGrabber = new Grabber();

        // create the lidarlite class on DIO 5
        // distance = new LidarLite(new DigitalInput(5));

        // Not using the PIXY right now
        //pixyVision = new PixyVision();

        // Not using the limelight right now
        // limeLight = new LimeLight();

        // Limit switches on the climbers
        // rightClimbLimit = new DigitalInput(0);
        // leftClimbLimit = new DigitalInput(1);

        
        try {
            ahrs = new AHRS(SPI.Port.kMXP);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }
    
        // Instantiate the compress, CANID 2, Rev Robotics PCM
        compressor = new Compressor(2, PneumaticsModuleType.REVPH);
        compressor.enableDigital();

        // Initialize the built in gyro
        internalData.initGyro();
        internalData.resetGyro();

        // Start the camera server for the drive camera
        driveCam = CameraServer.startAutomaticCapture();
		server = CameraServer.getServer();
        driveCam.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
		server.setSource(driveCam);

        // Dashboard Cooser for the Autonomous mode move
        autoFunction.setDefaultOption("Floor Cone",0);
        autoFunction.addOption("Mid Cone",1);
        autoFunction.addOption("High Cone",2);
        autoFunction.addOption("No Cone",3);
        SmartDashboard.putData("Auto Choices",autoFunction);

        // Dashboard Cooser for the Autonomous mode position
        autoPosition.setDefaultOption("Left Position",0);
        autoPosition.addOption("Center Position",1);
        autoPosition.addOption("Right Position",2);
        SmartDashboard.putData("Auto Position",autoPosition);

        autoBalance.setDefaultOption("Do Nothing",0);
        autoBalance.addOption("Balance",1);
        autoBalance.addOption("Leave Saftey Zone",2);
        SmartDashboard.putData("Auto Choices",autoBalance);

        Log.print(0, "Robot", "Robot Init Complete");
    }

 	  /************************************************************************
	   * This function is run once each time the robot enters autonomous mode. 
     ************************************************************************/
    @Override
    public void autonomousInit() {
        Log.print(0, "Robot", "Robot Autonomous Init");

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
			selectedAutoBalance = (int)autoBalance.getSelected();
		} catch(NullPointerException e) {
			selectedAutoBalance = 0;
		}

        switch (selectedAutoPosition) {
            case 0:
            {
                // Left Position
                if ( selectedAutoBalance==1 ) { selectedAutoBalance=0; }
                switch (selectedAutoFunction) {
                    case 0:
                        //autonomous = new AutoConeLow(selectedAutoBalance);    
                        //SmartDashboard.putString("AutoCommand","One Ball");
                        break;
                    case 1:
                        //autonomous = new AutoConeMid(selectedAutoBalance);    
                        break;
                    case 2:
                        //autonomous = new AutoConeHigh(selectedAutoBalance);    
                        break;
                    case 3:
                        break;    
                }
                break;
            }
            case 1:
            {
                // Center Position
                switch (selectedAutoFunction) {
                    case 0:
                        //autonomous = new AutoConeLow(selectedAutoBalance);    
                        //SmartDashboard.putString("AutoCommand","One Ball");
                        break;
                    case 1:
                        //autonomous = new AutoConeMid(selectedAutoBalance);    
                        break;
                    case 2:
                        //autonomous = new AutoConeHigh(selectedAutoBalance);    
                        break;
                    case 3:
                        //autonomous = new AutoBalance();    
                        break;    
                }
            }
            break;
            case 2:
            {
                // Right Position
                if ( selectedAutoBalance==1 ) { selectedAutoBalance=0; }
                switch (selectedAutoFunction) {
                    case 0:
                        //autonomous = new AutoConeLow(selectedAutoBalance);    
                        //SmartDashboard.putString("AutoCommand","One Ball");
                        break;
                    case 1:
                        //autonomous = new AutoConeMid(selectedAutoBalance);    
                        break;
                    case 2:
                        //autonomous = new AutoConeHigh(selectedAutoBalance);    
                        break;
                    case 3:
                        break;    
                }
            }
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
    }

 	  /************************************************************************
     * This function is called periodically during teleoperated mode.
	   ************************************************************************/
    @Override
    public void teleopPeriodic() {
        CommandScheduler.getInstance().run();
    }

 	  /************************************************************************
     * This function is called once each time the robot enters test mode.  
	   ************************************************************************/
    @Override
    public void testInit() {
        Log.print(0, "Robot", "Robot Test Init");
    }  

 	  /************************************************************************
     * This function is called periodically during test mode.
	   ************************************************************************/
   @Override
    public void testPeriodic() {
        CommandScheduler.getInstance().run();
    }
}
