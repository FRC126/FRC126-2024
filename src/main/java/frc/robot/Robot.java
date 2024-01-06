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
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DigitalInput;

// Navx-MXP Libraries and Connection Library
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

//import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;

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

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // Drive Base Motors
    public static CANSparkMax leftLeadMotor = new CANSparkMax(RobotMap.leftDriveMotorCanID1, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static CANSparkMax rightLeadMotor = new CANSparkMax(RobotMap.rightDriveMotorCanID1, CANSparkMaxLowLevel.MotorType.kBrushless);
    private static CANSparkMax leftFollowMotor = new CANSparkMax(RobotMap.leftDriveMotorCanID2, CANSparkMaxLowLevel.MotorType.kBrushless);
    private static CANSparkMax rightFollowMotor = new CANSparkMax(RobotMap.rightDriveMotorCanID2,  CANSparkMaxLowLevel.MotorType.kBrushless); 

    static {
        leftFollowMotor.follow(leftLeadMotor);
        rightFollowMotor.follow(rightLeadMotor);
    }

    private SparkMaxPIDController m_leftMotorPidController;
    private SparkMaxPIDController m_rightMotorPidController;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

    // Drive base encoders
    public static DutyCycleEncoder leftDriveEncoder = new DutyCycleEncoder(0);
    public static DutyCycleEncoder rightDriveEncoder = new DutyCycleEncoder(1);

    public static RelativeEncoder left1RelativeEncoder = Robot.leftLeadMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42  );
    public static RelativeEncoder left2RelativeEncoder = Robot.leftFollowMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42  );
    public static RelativeEncoder right1RelativeEncoder = Robot.rightLeadMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42  );
    public static RelativeEncoder right2RelativeEncoder = Robot.rightFollowMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42  );

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // Tower Arm Motor
    public static CANSparkMax TowerArmMotor = new CANSparkMax(RobotMap.TowerArmMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static RelativeEncoder TowerArmRelativeEncoder = Robot.TowerArmMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42  );
    public static DigitalInput towerArmRetracedLimit;

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // ArmExtension Motor
    public static CANSparkMax ArmExtensionMotor = new CANSparkMax(RobotMap.ArmExtensionMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static RelativeEncoder ArmExtensionRelativeEncoder = Robot.ArmExtensionMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42  );
    public static DigitalInput armExtensionBottomLimit;
    
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // Grabber Motor
    public static CANSparkMax GrabberMotor = new CANSparkMax(RobotMap.GrabberMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static RelativeEncoder GrabberRelativeEncoder = Robot.GrabberMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42  );
    public static DigitalInput grabberRetracedLimit;

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // Catapult Motor
    public static CANSparkMax CatapultMotor = new CANSparkMax(RobotMap.catapultMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static RelativeEncoder CatapultRelativeEncoder = Robot.CatapultMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42  );
    public static DigitalInput catapultBottomLimit;
    
  /////////////////////////////////////////////////////////////////////////////////////////////////////
    // Pickup Motor
    public static CANSparkMax pickupMotor = new CANSparkMax(RobotMap.pickupMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static RelativeEncoder pickupRelativeEncoder = Robot.pickupMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42  );
    
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
    
    public static DoubleSolenoid PickupSolenoid = new DoubleSolenoid(2, PneumaticsModuleType.REVPH,15,14);	

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // Subsystems
    public static Controllers oi;
    public static Log log;
    public static InternalData internalData;
    public static WestCoastDrive driveBase;
    public static TowerArm robotTowerArm;
    public static Grabber robotGrabber;
    public static ArmExtension robotArmExtension;
    public static Catapult robotCatapult;

    public static Catapult robotBrakes;
    public static Pickup robotPickup;

	public static UsbCamera driveCam;
	public static VideoSink server;
    public static SequentialCommandGroup autonomous;

    public static LimeLight limeLight;

    // Global Robot Variables
    public int RobotID = 0;

    public static boolean ignoreEncoders=false;
    public static boolean autoMove=false;

    //public static enum targetHeights{LowTarget,HighTarget};
    public static enum targetTypes{NoTarget,TargetSeek};
    public static enum allianceColor{Red,Blue};
	public static double voltageThreshold = 10.0;

    public static Compressor compressor;

    // For use with limelight class
    public static double ThrowerRPM=0;

    int selectedAutoPosition;
	int selectedAutoFunction;
    int selectedAutoBalance;
    int selectedAllianceColor;
	
    private final SendableChooser<Integer> autoFunction = new SendableChooser<>();
    private final SendableChooser<Integer> autoPosition = new SendableChooser<>();
    private final SendableChooser<Integer> autoBalance = new SendableChooser<>();
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
        driveBase = new WestCoastDrive();

        // Brakes
        robotBrakes = new Catapult();

        // Pickup
        robotPickup = new Pickup();

        // Initilize Tower Arm
        robotTowerArm = new TowerArm();
        towerArmRetracedLimit = new DigitalInput(6);

        // Initilize Grabber    
        robotGrabber = new Grabber();
        grabberRetracedLimit = new DigitalInput(7);
        
        // Initilize Arm Extension    
        robotArmExtension = new ArmExtension();
		armExtensionBottomLimit = new DigitalInput(8);

        // create the lidarlite class on DIO 5
        // distance = new LidarLite(new DigitalInput(5));

        // Not using the PIXY right now
        //pixyVision = new PixyVision();

      // Catapult Limit switch on DIO port 9
		catapultBottomLimit = new DigitalInput(5);

        // Not using the limelight right now
        // limeLight = new LimeLight();
       
        robotCatapult = new Catapult();

        try {
            navxMXP = new AHRS(SPI.Port.kMXP);
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
        autoFunction.setDefaultOption("High Cone",2);
        autoFunction.addOption("Mid Cone",1);
        autoFunction.addOption("Floor Cone",0);
        autoFunction.addOption("No Cone",3);
        SmartDashboard.putData("Auto Cone Choices",autoFunction);

        // Dashboard Cooser for the Autonomous mode position
        autoPosition.setDefaultOption("Inside Position",0);
        autoPosition.addOption("Center Position",1);
        autoPosition.addOption("Outisde Position",2);
        SmartDashboard.putData("Auto Robot Position",autoPosition);

        // Dashboard Cooser for the Autonomous mode position
        allianceColor.setDefaultOption("Red Alliance",0);
        allianceColor.addOption("Blue Alliance",1);
        SmartDashboard.putData("Alliance Color",allianceColor);
        
        autoBalance.setDefaultOption("Leave SZ, Get Cube",2);
        autoBalance.addOption("Balance",1);
        autoBalance.addOption("Do Nothing",5);
        SmartDashboard.putData("Auto Action Choices",autoBalance);

        Log.print(0, "Robot", "Robot Init Complete");
    }

 	  /************************************************************************
	   * This function is run once each time the robot enters autonomous mode. 
     ************************************************************************/
    @Override
    public void autonomousInit() {
        Log.print(0, "Robot", "Robot Autonomous Init");

        Robot.stopAutoCommand();

        Robot.robotTowerArm.cancel();
		Robot.robotGrabber.cancel();
		Robot.robotArmExtension.cancel();
		Robot.driveBase.cancel();

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
			selectedAutoBalance = 5;
		}
		try {
			selectedAllianceColor = (int)allianceColor.getSelected();
		} catch(NullPointerException e) {
			selectedAllianceColor = 0;
		}

        switch (selectedAutoPosition) {
            case 0:
            {
                // Inside Position
                if (selectedAutoBalance==1) selectedAutoBalance=5;
                if (selectedAutoBalance==2) { selectedAutoBalance=3; }
                
                switch (selectedAutoFunction) {
                    case 0:
                        // Floor Cone
                        autonomous = new AutoPlaceConeLow(selectedAutoBalance);    
                        SmartDashboard.putString("AutoCommand","Left - Cone Low - No Balance");
                        break;
                    case 1:
                        // Mid Cone
                        autonomous = new AutoPlaceConeMid(selectedAutoBalance);    
                        SmartDashboard.putString("AutoCommand","Left - Cone Mid - No Balance");
                        break;
                    case 2:
                        // High Cone
                        autonomous = new AutoPlaceConeHigh(selectedAutoBalance);    
                        SmartDashboard.putString("AutoCommand","Left - Cone High - No Balance");
                        break;
                    case 3:
                        // No Cone
                        SmartDashboard.putString("AutoCommand","Left - No Cone - No Balance");
                        break;    
                }
            }
            break;
            case 1:
            {
                // Center Position
                if (selectedAutoBalance==2) selectedAutoBalance=5;

                switch (selectedAutoFunction) {
                    case 0:
                        // Floor Cone
                        autonomous = new AutoPlaceConeLow(selectedAutoBalance);    
                        SmartDashboard.putString("AutoCommand","Center - Cone Low");
                        break;
                    case 1:
                        // Mid Cone
                        autonomous = new AutoPlaceConeMid(selectedAutoBalance);    
                        SmartDashboard.putString("AutoCommand","Center - Cone Mid");
                        break;
                    case 2:
                        // High Cone
                        autonomous = new AutoPlaceConeHigh(selectedAutoBalance);    
                        SmartDashboard.putString("AutoCommand","Center - Cone High");
                        break;
                    case 3:
                        // No Cone
                        if (selectedAutoBalance==1) {
                            autonomous = new AutoClimbBalance();    
                            SmartDashboard.putString("AutoCommand","Center - No Cone - Balance");
                        }    
                        break;    
                }
            }
            break;
            case 2:
            {
                // Outside Position
                if (selectedAutoBalance==1) { selectedAutoBalance=5; }
                if (selectedAllianceColor == 1 && selectedAutoBalance == 3) { selectedAutoBalance=4; }

                switch (selectedAutoFunction) {
                    case 0:
                        // Floor Cone
                        autonomous = new AutoPlaceConeLow(selectedAutoBalance);    
                        SmartDashboard.putString("AutoCommand","Right - Cone Low - No Balance");
                        break;
                    case 1:
                        // Mid Cone
                        autonomous = new AutoPlaceConeMid(selectedAutoBalance);    
                        SmartDashboard.putString("AutoCommand","Right - Cone Mid - No Balance");
                        break;
                    case 2:
                        // High Cone
                        autonomous = new AutoPlaceConeHigh(selectedAutoBalance);    
                        SmartDashboard.putString("AutoCommand","Right - Cone High - No Balance");
                        break;
                    case 3:
                        // No Cone
                        SmartDashboard.putString("AutoCommand","Right - No Cone - No Balance");
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

        Robot.stopAutoCommand();

        Robot.robotTowerArm.cancel();
		Robot.robotGrabber.cancel();
		Robot.robotArmExtension.cancel();
		Robot.driveBase.cancel();
    }

    /************************************************************************
     * This function is called periodically during teleoperated mode.
    ************************************************************************/
    @Override
    public void teleopPeriodic() {
        CommandScheduler.getInstance().run();

        setPid(leftLeadMotor.getPIDController(), "Left");
        setPid(rightLeadMotor.getPIDController(), "Right");
    }

    private void setupPidController(SparkMaxPIDController pidController, String leftRight) {
        kP = 0.1;
        kI = 1e-4;
        kD = 1;
        kIz = 0;
        kFF = 0;
        kMaxOutput = 1;
        kMinOutput = -1;

        // set PID coefficients
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setIZone(kIz);
        pidController.setFF(kFF);
        pidController.setOutputRange(kMinOutput, kMaxOutput);

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain " + leftRight, kP);
        SmartDashboard.putNumber("I Gain " + leftRight, kI);
        SmartDashboard.putNumber("D Gain " + leftRight, kD);
        SmartDashboard.putNumber("I Zone " + leftRight, kIz);
        SmartDashboard.putNumber("Feed Forward " + leftRight, kFF);
        SmartDashboard.putNumber("Max Output " + leftRight, kMaxOutput);
        SmartDashboard.putNumber("Min Output " + leftRight, kMinOutput);
        SmartDashboard.putNumber("Set Rotations " + leftRight, 0);
    }

    private void setPid(SparkMaxPIDController pidController, String leftRight) {
        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain " + leftRight, 0);
        double i = SmartDashboard.getNumber("I Gain " + leftRight, 0);
        double d = SmartDashboard.getNumber("D Gain " + leftRight, 0);
        double iz = SmartDashboard.getNumber("I Zone " + leftRight, 0);
        double ff = SmartDashboard.getNumber("Feed Forward " + leftRight, 0);
        double max = SmartDashboard.getNumber("Max Output " + leftRight, 0);
        double min = SmartDashboard.getNumber("Min Output " + leftRight, 0);
        double rotations = SmartDashboard.getNumber("Set Rotations " + leftRight, 0);

        // if PID coefficients on SmartDashboard have changed, write new values to
        // controller
        if ((p != kP)) {
            pidController.setP(p);
            kP = p;
        }
        if ((i != kI)) {
            pidController.setI(i);
            kI = i;
        }
        if ((d != kD)) {
            pidController.setD(d);
            kD = d;
        }
        if ((iz != kIz)) {
            pidController.setIZone(iz);
            kIz = iz;
        }
        if ((ff != kFF)) {
            pidController.setFF(ff);
            kFF = ff;
        }
        if ((max != kMaxOutput) || (min != kMinOutput)) {
            pidController.setOutputRange(min, max);
            kMinOutput = min;
            kMaxOutput = max;
        }

        pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);

        SmartDashboard.putNumber("SetPoint " + leftRight, rotations);
    }

    /************************************************************************
     * This function is called once each time the robot enters test mode.  
    ************************************************************************/
    @Override
    public void testInit() {
        Log.print(0, "Robot", "Robot Test Init");

		Robot.robotTowerArm.cancel();
		Robot.robotGrabber.cancel();
		Robot.robotArmExtension.cancel();
		Robot.driveBase.cancel();
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
		Robot.robotTowerArm.cancel();
		Robot.robotGrabber.cancel();
		Robot.robotArmExtension.cancel();
        Robot.robotPickup.cancel();
        if (Robot.autoMove) { Robot.driveBase.cancel(); }

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

		Robot.robotTowerArm.cancel();
		Robot.robotGrabber.cancel();
		Robot.robotArmExtension.cancel();
        Robot.robotPickup.cancel();
        if (Robot.autoMove) { Robot.driveBase.cancel(); }
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
