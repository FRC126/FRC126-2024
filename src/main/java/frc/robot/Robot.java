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
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.subsystems.*;
//import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DriverStation;

// Navx-MXP Libraries and Connection Library
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

//import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import com.ctre.phoenix6.hardware.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    public static final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
    public static final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
    public static final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

    //public static Drivetrain driveTrain = new Drivetrain();

    public static JoystickWrapper driveJoystick = new JoystickWrapper(Robot.oi.driveController, 0.15);

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
        swerveDrive = new SwerveDrive();

            // Not using the limelight right now
        // limeLight = new LimeLight();
       
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
                        //autonomous = new AutoPlaceConeLow(selectedAutoBalance);    
                        SmartDashboard.putString("AutoCommand","Left - Cone Low - No Balance");
                        break;
                    case 1:
                        // Mid Cone
                        //autonomous = new AutoPlaceConeMid(selectedAutoBalance);    
                        SmartDashboard.putString("AutoCommand","Left - Cone Mid - No Balance");
                        break;
                    case 2:
                        // High Cone
                        //autonomous = new AutoPlaceConeHigh(selectedAutoBalance);    
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
                        //autonomous = new AutoPlaceConeLow(selectedAutoBalance);    
                        SmartDashboard.putString("AutoCommand","Center - Cone Low");
                        break;
                    case 1:
                        // Mid Cone
                        //autonomous = new AutoPlaceConeMid(selectedAutoBalance);    
                        SmartDashboard.putString("AutoCommand","Center - Cone Mid");
                        break;
                    case 2:
                        // High Cone
                        //autonomous = new AutoPlaceConeHigh(selectedAutoBalance);    
                        SmartDashboard.putString("AutoCommand","Center - Cone High");
                        break;
                    case 3:
                        // No Cone
                        if (selectedAutoBalance==1) {
                            //autonomous = new AutoClimbBalance();    
                            SmartDashboard.putString("AutoCommand","Center - No Cone - Balance");
                        }    
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

    /************************************************************************
	 ************************************************************************/
    private void driveWithJoystick(boolean fieldRelative) {
        double y1 = driveJoystick.getLeftStickY();
        double x1 = driveJoystick.getLeftStickX();
        double x2 = driveJoystick.getRightStickX();

		Robot.swerveDrive.Drive(y1, x1, x2);

		// Get the x speed. We are inverting this because Xbox controllers return
		// negative values when we push forward.
		final var xSpeed =
			Robot.xspeedLimiter.calculate(MathUtil.applyDeadband(y1, 0.02)) * Drivetrain.kMaxSpeed;

		// Get the y speed or sideways/strafe speed. We are inverting this because
		// we want a positive value when we pull to the left. Xbox controllers
		// return positive values when you pull to the right by default.
		final var ySpeed =
			Robot.yspeedLimiter.calculate(MathUtil.applyDeadband(x1, 0.02)) * Drivetrain.kMaxSpeed;

		// Get the rate of angular rotation. We are inverting this because we want a
		// positive value when we pull to the left (remember, CCW is positive in
		// mathematics). Xbox controllers return positive values when you pull to
		// the right by default.
		final var rot =
			Robot.rotLimiter.calculate(MathUtil.applyDeadband(x2, 0.02)) * Drivetrain.kMaxAngularSpeed;

		//driveTrain.drive(xSpeed, ySpeed, rot, true, getPeriod());	       
  }

}
