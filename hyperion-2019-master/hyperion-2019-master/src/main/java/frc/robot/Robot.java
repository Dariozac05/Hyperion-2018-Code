/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*
 * THE OFFICIAL TEAM HYPERION ROBOT CODE 2018
 * This code was written by Carlos Saucedo, Zachary Moroski & Dario Zaccagnino.
 * This robot utilizes Mecanum wheels.
*/

package frc.robot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
//import java.io.Console;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.PIDController;
//import edu.wpi.first.wpilibj.PIDInterface;
//import edu.wpi.cscore.UsbCamera; //For the alternate camera code


@SuppressWarnings("deprecation")
public class Robot extends SampleRobot{
        /*
         * Variables for autonomous cases. 
         * initPosition values: L, M, R (fairly straightforward)
         * initGoal values: c, w, F, N (c -> scale; w -> switch; F -> Move Foward; N -> Do Nothing)
		 * 
         */
        char initPosition = 'M';
        char initGoal = 'w';
        
	    //Channels for motors/joysticks
		private static final int kFrontLeftChannel = 5;
		private static final int kRearLeftChannel = 1;
		private static final int kFrontRightChannel = 6;
		private static final int kRearRightChannel = 4;
		private static final int kLeftJoystickChannel = 0;
		private static final int kRightJoystickChannel = 1;
		//private static final int kDarioJoystickChannel = 2;
		private static final int kFrontRightEncoderA = 6;
		private static final int kFrontRightEncoderB = 7;
		private static final int kRearRightEncoderA = 2;
		private static final int kRearRightEncoderB = 3;
		private static final int kFrontLeftEncoderA = 4;
		private static final int kFrontLeftEncoderB = 5;
		private static final int kRearLeftEncoderA = 0;
		private static final int kRearLeftEncoderB = 1;
		private static final int kSlideMotor1 = 2;
		private static final int kSlideMotor2 = 3;
		private static final int kArmMotor = 8;
		private static final int kArmEncoderA = 8;
		private static final int kArmEncoderB = 9;
		private static final int kLeftArmMotor = 7;
		private static final int kRightArmMotor = 9;
		private double wheelCircumference = 25.132741228700002267; //Circumference (in inches)
		private double e_distancePerPulse = (wheelCircumference/1440) * 4; //Distance per pulse (circumference/pulses per revolution)
		private double slideMovementScaleTime = 7.0; //TO-DO
		private double slideMovementSwitchTime = 3.0; //TO-DO
		
		//MecanumDrive constructor
		private MecanumDrive m_robotDrive;

		//Controllers/etc
		private Timer m_timer = new Timer();
		private Joystick m_rightJoystick;
		private Joystick m_leftJoystick;
		//private Joystick m_darioJoystick;
		
		//Drive Train Components
		private SpeedController m_frontLeft;
		private SpeedController m_frontRight;
		private SpeedController m_rearLeft;
		private SpeedController m_rearRight;
		private SpeedController m_slideMotor1;
		private SpeedController m_slideMotor2;
		private SpeedController m_ArmMotor;
		private SpeedController m_LeftArmMotor;
		private SpeedController m_RightArmMotor;
			
		//Sensor declarations
		private ADXRS450_Gyro onboardGyro;
		private MecanumEncoder m_robotEncoder;
		private Encoder e_frontRight;
		private Encoder e_rearRight;
		private Encoder e_frontLeft;
		private Encoder e_rearLeft;
		private Encoder e_ArmEncoder;

		//PIDControllers
		private PIDController PIDArmController;
		//private PIDController PIDElevatorController;
		
	public void robotInit() {
		//Constructors for joysticks and motor controllers
				m_rightJoystick = new Joystick(kRightJoystickChannel);
				m_leftJoystick = new Joystick(kLeftJoystickChannel);
				m_frontLeft = new VictorSP(kFrontLeftChannel);
				m_frontRight = new VictorSP(kFrontRightChannel);
				m_rearLeft = new VictorSP(kRearLeftChannel);
				m_rearRight = new VictorSP(kRearRightChannel);
				m_slideMotor1 = new VictorSP(kSlideMotor1);
				m_slideMotor2 = new VictorSP(kSlideMotor2);
				m_slideMotor2.setInverted(true);
				m_ArmMotor = new VictorSP(kArmMotor);
				m_LeftArmMotor = new VictorSP(kLeftArmMotor);
				m_RightArmMotor = new VictorSP(kRightArmMotor);
				
				
				
				
				//Constructor for RobotDrive
				//Note: X is forward-backward, Y is left-right(strafe), Z is rotation
				m_robotDrive = new MecanumDrive(m_frontLeft, m_rearLeft, m_frontRight, m_rearRight);
				
				//Encoders
				e_frontRight = new Encoder(kFrontRightEncoderA, kFrontRightEncoderB, false, Encoder.EncodingType.k1X);
				e_rearRight = new Encoder(kRearRightEncoderA, kRearRightEncoderB, false, Encoder.EncodingType.k1X);
				e_frontLeft = new Encoder(kFrontLeftEncoderA, kFrontLeftEncoderB, true, Encoder.EncodingType.k1X);
				e_rearLeft = new Encoder(kRearLeftEncoderA, kRearLeftEncoderB, true, Encoder.EncodingType.k1X);
				e_frontRight.setDistancePerPulse(e_distancePerPulse);
				e_rearRight.setDistancePerPulse(e_distancePerPulse);
				e_frontLeft.setDistancePerPulse(e_distancePerPulse);
				e_rearLeft.setDistancePerPulse(e_distancePerPulse);
				//Arm Encoder
				e_ArmEncoder = new Encoder(kArmEncoderA, kArmEncoderB, true, Encoder.EncodingType.k1X);
				//PID Arm Controller (P Gain, I Gain, D Gain, Input Source, Output Source)
				PIDArmController = new PIDController(0, 0, 0, e_ArmEncoder, m_ArmMotor);
				
				//Drive Encoders
				m_robotEncoder = new MecanumEncoder(e_frontRight, e_rearRight, e_frontLeft, e_rearLeft);
				
				//Reset Encoders
				m_robotEncoder.reset();

				//Gyro
				 onboardGyro = new  ADXRS450_Gyro();
				 onboardGyro.calibrate();//Calibrates the gyro
				 onboardGyro.reset();//Sets gyro to 0 degrees
				 
				 /*
				  * Camera: try catch statement so that code does not crash if camera does not work.
				  */
				 CameraServer camera = CameraServer.getInstance();
				 camera.startAutomaticCapture();
			}
	
	/**
	 * 
     * This function is called once each time the robot enters autonomous mode.
     */
    public void autonomous() { 
		m_timer.reset();
		m_timer.start();
        	String gameData;
    		gameData = DriverStation.getInstance().getGameSpecificMessage();
    		System.out.println("Field Data: " + gameData);
    		switch(initPosition) {
    		
    		//Alternate, fail-safe code.
    		case 'N':
    			moveForward(0);
    			break;
    		//Swwitches b/w starting positions	
            case 'L':
                switch(initGoal){
					//Move Foward
					case 'F':
    					moveForward(100);
					break;
					//switches b/w scale/switch
                    case 'c':
                        if(gameData.charAt(1) == 'L') {//DONE
                        	moveSlideUp(0.5, slideMovementScaleTime);
                            moveForward(36.0);
                            turnLeft();
                            moveForward(38.37);
                            turnRight();
                            moveForward(273.35);
                            turnRight();
                            
                        } else if(gameData.charAt(1) == 'R') {//First scale is on right
                        	moveSlideUp(0.5, slideMovementScaleTime);
                            moveForward(36);
                            turnLeft();
                            moveForward(38.37);
                            turnRight();
                            moveForward(196.22);
                            turnRight();
                            moveForward(238.74);
                            turnLeft();
                            moveForward(74.43);
                            turnLeft();
                    }   
                    break;
                    //switches b/w scale/switch
                    case 'w':
                        if(gameData.charAt(0) == 'L') {//DONE
                        	moveSlideUp(0.5, 3.0);
                            moveForward(106 - 25);
                        
                        } else if(gameData.charAt(0) == 'R') {//DONE
                        	moveSlideUp(0.5, slideMovementSwitchTime);
                            moveForward(24.0);
                            turnRight();
                            moveForward(86.0);
                            turnLeft();
                            moveForward(89.0 - 25);
                    }
                    break;
                }
            break;
            
            case 'M':
                switch(initGoal){
					//Move Foward
					case 'F':
    					moveForward(100);
    				break;
					//switches b/w scale/switch
                    case 'c':
                        if(gameData.charAt(1) == 'L') {//DONE
                        	moveSlideUp(0.5, slideMovementScaleTime);
    			            moveForward(36.0);
    			            turnLeft();
    			            moveForward(134.75);
    			            turnRight();
    			            moveForward(270.75);
    			            turnRight();
    			
    		          } else if(gameData.charAt(1) == 'R') {//DONE
    		        	  	moveSlideUp(0.5, slideMovementScaleTime);
    			            moveForward(36.0);
    			            turnRight();
    			            moveForward(134.75);
    			            turnLeft();
    			            moveForward(270.75);
    			            turnLeft();
    		        }        
                    break;
                    //switches b/w scale/switch
                    case 'w':
                        if(gameData.charAt(0) == 'L') {//DONE
                     	    moveSlideUp(0.5, slideMovementSwitchTime);
                            moveForward(12.0);
                            turnLeft();
                            moveForward(50.0);
                            turnRight();
                            moveForward(82.0 - 2.0 + 12.0 - 22);
                    } else if(gameData.charAt(0) == 'R') {//DONE
                    	   moveSlideUp(0.5, slideMovementSwitchTime);
                           moveForward(12.0);
                           turnRight();
                           moveForward(50.0);
                           turnLeft();
                           moveForward(82.0 - 2.0 + 12.0);
                    }        
                    break;
                }
    		break;
            
            case 'R':
                switch(initGoal){
					//Move Foward
					case 'F':
    					moveForward(100);
					break;
					//switches b/w scale/switch
                    case 'c':
                        if(gameData.charAt(1) == 'L') {//DONE
                        	moveSlideUp(0.5, slideMovementScaleTime);
                            moveForward(36);
                            turnRight();
                            moveForward(38.37);
                            turnLeft();
                            moveForward(196.22);
                            turnLeft();
                            moveForward(238.74);
                            turnRight();
                            moveForward(74.43);
                            turnRight();
                            
                        } else if(gameData.charAt(1) == 'R') {//DONE
                        	moveSlideUp(0.5, slideMovementScaleTime);
                            moveForward(36.0);
                            turnRight();
                            moveForward(38.37);
                            turnLeft();
                            moveForward(273.75);
                            turnLeft();
                            
                    }   
                    break;
                    //switches b/w scale/switch
                    case 'w':
                        if(gameData.charAt(0) == 'L') {//DONE
                            moveForward(24.0);
                            turnLeft();
                            moveForward(86.0);
                            turnRight();
                            moveForward(82.0 - 25);
                        
                        } else if(gameData.charAt(0) == 'R') {//DONE
                            moveForward(106.0 - 25);
                    }
                    break;
                }
            break;
        }
    }
    /**
     * This function is called once each time the robot enters teleop mode.
     */
    public void operatorControl() {
        while (isOperatorControl() && isEnabled()) {
			System.out.println("Encoder: " + e_ArmEncoder);
			PIDArmController.enable();
			//Input Range is encoder range
			PIDArmController.setInputRange(0, 150);
			PIDArmController.setOutputRange(0, 100);

        	//Mecanum drive using 2 joysticks
        	m_robotDrive.driveCartesian(m_rightJoystick.getX(), -m_rightJoystick.getY(), m_leftJoystick.getX(),0.0);
        	
        	//Slide rail controls
        	if(m_rightJoystick.getRawButton(3)) {
        		moveSlideUp(0.75);
        	}else if(m_rightJoystick.getRawButton(2)) {
				moveSlideDown(0.5);
        	}else if(!m_rightJoystick.getRawButton(3) || !m_rightJoystick.getRawButton(2)) {
        		stopSlide();
        	}
        	
			//Arm Rotate Controlls
			/*Arms always start up
			Up value is 0*
			Scale/Switch Value is 30
			Down value 100
			Middle Value is 90 */
        	if(m_leftJoystick.getRawButtonPressed(3)) {	
				//Middle Value
				PIDArmController.setSetpoint(90);
        	}else if(m_leftJoystick.getRawButtonPressed(2)) {
				//Down Value
				PIDArmController.setSetpoint(100);
			} else if(m_leftJoystick.getRawButtonPressed(4)) {
				//Up Value
				PIDArmController.setSetpoint(0);
			} else {
				//Middle Value
				PIDArmController.setSetpoint(90);
			}

			//Intake Controls
			if(m_leftJoystick.getTrigger()) {
				//Intake
				Intake(0.75);
			} else if(m_rightJoystick.getTrigger()) {
				//Outtake
				Outtake(-0.75);
			} else {
				//Keep intake moving
				Intake(0.2);
			}
            Timer.delay(0.02);
        }
    }
    
    public void test() {
    	turnRight();
    	Timer.delay(1.0);
    	turnLeft();
    	Timer.delay(0.5);
    	}
    /**
     * Moves the robot forward a set distance in units.
     * 
     * @param distance Distance, in units.
     */
    public void moveForward(double distance) {
        if(isAutonomous() || isTest()){
            double initDistance = m_robotEncoder.getDistance();
    	    while((m_robotEncoder.getDistance() < initDistance + distance) && ((isAutonomous() || isTest()) && isEnabled())) {
    	    	System.out.println("Encoder:  " + m_robotEncoder.getDistance());
    		m_robotDrive.driveCartesian(0.0, 0.5, 0.0, 0.0);
    		Timer.delay(0.02);
    	   }
    	   m_robotDrive.driveCartesian(0.0, -0.1, 0.0, 0.0);
    	   Timer.delay(0.2);
    	   m_robotDrive.driveCartesian(0.0, 0.0, 0.0, 0.0);
        }
    }
    
    /**
     * Moves the robot backward a set distance in units.
     * 
     * @param distance Distance, in units.
     */
    public void moveBackward(double distance) {
        if(isAutonomous() || isTest()){
            double initDistance = m_robotEncoder.getDistance();
    	    while(m_robotEncoder.getDistance() > initDistance - distance && ((isAutonomous() || isTest()) && isEnabled())) {
      		    m_robotDrive.driveCartesian(0.0, -0.5, 0.0, 0.0);
    		  Timer.delay(0.02);
    	   }  
    	   m_robotDrive.driveCartesian(0.0, 0.1, 0.0, 0.0);
    	   Timer.delay(0.2);
    	   m_robotDrive.driveCartesian(0.0, 0.0, 0.0, 0.0);
        }
    }
    
    /**
     * Strafes the robot to the right a set distance.
     * @param distance Distance to strafe.
     */
    public void strafeRight(double distance) {//DOES NOT WORK
    	if(isAutonomous() || isTest()) {
    		m_robotEncoder.reset();
    		double initX = m_robotEncoder.getPositionX();
    		  while(m_robotEncoder.getPositionX() < initX + distance) {
        		    m_robotDrive.driveCartesian(0.25, 0.0, 0.0, 0.0);
        		    Timer.delay(0.02);
    		  }
    		  m_robotDrive.driveCartesian(-0.1, 0.0, 0.0, 0.0);
    		  Timer.delay(0.02);
    		  m_robotDrive.driveCartesian(0.0, 0.0, 0.0, 0.0);
    	}  
    }
   
    /**
     * Strafes the robot to the left a set distance.
     * @param distance Distance to strafe.
     */
    public void strafeLeft(double distance) {//DOES NOT WORK
    	if(isAutonomous() || isTest()) {
    		m_robotEncoder.reset();
    		double initX = m_robotEncoder.getPositionX();
    		  while(m_robotEncoder.getPositionX() > initX - distance) {
        		    m_robotDrive.driveCartesian(-0.25, 0.0, 0.0, 0.0);
        		    Timer.delay(0.02);
    		  }
    		  m_robotDrive.driveCartesian(0.1, 0.0, 0.0, 0.0);
    		  Timer.delay(0.02);
    		  m_robotDrive.driveCartesian(0.0, 0.0, 0.0, 0.0);
    	}  
    }  
    
    /**
     * Turns the robot to the right 90 degrees
     */
    public void turnRight() {
    	if(isAutonomous() || isTest()) {
    		double initBearing = onboardGyro.getAngle();
        	while(onboardGyro.getAngle() < initBearing + 90 && ((isAutonomous() || isTest()) && isEnabled())) {
        		System.out.println("Gyro: " + onboardGyro.getAngle());
        		m_robotDrive.driveCartesian(0.0, 0.0, 0.5,0.0);
        		Timer.delay(0.02);
        	}
        	m_robotDrive.driveCartesian(0.0, 0.0, -0.5 , 0.0);
    		Timer.delay(0.2);
        	m_robotDrive.driveCartesian(0.0, 0.0, 0.0, 0.0);
    	}
    }
    
    /**
     * Turns the robot to the left 90 degrees.
     */
    public void turnLeft() {
        if(isAutonomous() || isTest()){
            double initBearing = onboardGyro.getAngle();
    	while(onboardGyro.getAngle() > initBearing - 90 && ((isAutonomous() || isTest()) && isEnabled())) {
    		System.out.println("Gyro: " + onboardGyro.getAngle());
    		m_robotDrive.driveCartesian(0.0, 0.0, -0.5 , 0.0);
    		Timer.delay(0.02);
    	}
    	m_robotDrive.driveCartesian(0.0,0.0,0.5,0.0);
		Timer.delay(0.2);
    	m_robotDrive.driveCartesian(0.0, 0.0, 0.0, 0.0);
        }
    }
	//Arm Intake Controls
	//Intake
	public void Intake(double speed) {
		m_LeftArmMotor.set(speed);
		m_RightArmMotor.set(-speed);
	}

	public void Intake(double speed, double time) {
    	if(isAutonomous() || isTest()) {
    		Intake(speed);
    		Timer.delay(time);
    		stopIntake();
    	}
	}
	
	//Outtake
	public void Outtake(double speed) {
		m_LeftArmMotor.set(-speed);
		m_RightArmMotor.set(speed);
	}

	public void Outtake(double speed, double time) {
    	if(isAutonomous() || isTest()) {
    		Outtake(speed);
    		Timer.delay(time);
    		stopIntake();
    	}
	}

	public void stopIntake() {
		m_LeftArmMotor.set(0.0);
		m_RightArmMotor.set(0.0);
	}


    //Slide rail controls
    /**
     * Moves the slide up at a set speed.
     * @param speed Motor speed
     */
    public void moveSlideUp(double speed) {
 	   m_slideMotor1.set(speed);
 	   m_slideMotor2.set(-speed);
    }
    
    /**
     * Moves the slide down a set speed and time.
     * @param speed Motor speed
     * @param time Time to move slide
     */
    public void moveSlideUp(double speed, double time) {
    	if(isAutonomous() || isTest()) {
    		moveSlideUp(speed);
    		Timer.delay(time);
    		stopSlide();
    	}
    }
    
    /**
     * Moves the slide down at a set speed.
     * @param speed Motor speed
     */
    public void moveSlideDown(double speed) {
 	   m_slideMotor1.set(-speed);
 	   m_slideMotor2.set(speed);
    }
    
    /**
     * Moves the slide down at a set speed and time.
     * @param speed Motor speed
     * @param time Time to move slide
     */
    public void moveSlideDown(double speed, double time) {
    	if(isAutonomous() || isTest()) {
    		moveSlideDown(speed);
    		Timer.delay(time);
        	stopSlide();
    	}
    }
    
    /**
     * Stops the slide.
     */
    public void stopSlide() {
       m_slideMotor1.set(0.0);
  	   m_slideMotor2.set(0.0);
    }
  
}