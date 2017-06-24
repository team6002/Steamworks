
package org.usfirst.frc.team6002.robot;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team6002.robot.subsystems.Drive;
import org.usfirst.frc.team6002.robot.subsystems.GearArm;
import org.usfirst.frc.team6002.robot.subsystems.Climber;
import org.usfirst.frc.team6002.robot.subsystems.Shooter;
import org.usfirst.frc.team6002.robot.subsystems.Conveyor;
import org.usfirst.frc.team6002.auto.AutoModeExecuter;
import org.usfirst.frc.team6002.robot.ControlBoard;
import org.usfirst.frc.team6002.lib.util.CrashTracker;
import org.usfirst.frc.team6002.lib.util.DriveSignal;
import org.usfirst.frc.team6002.robot.CheesyDriveHelper;
import org.usfirst.frc.team6002.robot.commands.ExampleCommand;
import org.usfirst.frc.team6002.robot.subsystems.ExampleSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	public static final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
	public static OI oi;
    private Compressor compressor_;
    
	//S U B S Y S T E M
	Drive mDrive = Drive.getInstance();
	GearArm mGearArm = new GearArm();
	Climber mClimber = new Climber(); 
	Shooter mShooter = new Shooter();
	Conveyor mConveyor = new Conveyor();
	
	//other parts of robot
	ControlBoard mControls = ControlBoard.getInstance();
	CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
	AutoModeExecuter mAutoModeExecuter = new AutoModeExecuter();
	
//	SmartDashboardInteractions mSmartDashBoardInteractions = new SmartDashboardInteractions();
	
	// Enabled looper is called at 100Hz whenever the robot is enabled
    Looper mEnabledLooper = new Looper();
    // Disabled looper is called at 100Hz whenever the robot is disabled
    Looper mDisabledLooper = new Looper();
	
    Timer timer;
    
	Command autonomousCommand;
	SendableChooser<Command> chooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
//		oi = new OI();
//		chooser.addDefault("Default Auto", new ExampleCommand());
//		// chooser.addObject("My Auto", new MyAutoCommand());
//		SmartDashboard.putData("Auto mode", chooser);
		timer = new Timer();
		timer.start();
		
		try{
			CameraServer.getInstance().startAutomaticCapture(0);
			CameraServer.getInstance().startAutomaticCapture(1);
//			AxisCamera gearCamera = CameraServer.getInstance().addAxisCamera("axis-camera.local");
//			gearCamera.setResolution(640, 480);
//			CameraServer.getInstance().startAutomaticCapture(dev);
			mEnabledLooper.register(mDrive.getLoop());
//			mSmartDashBoardInteractions.initWithDefaults();
			outputAllToSmartDashboard();
			compressor_ = new Compressor(Constants.kCompressorId);
			compressor_.setClosedLoopControl(true);
			mDrive.setHighGear(false);
			mDrive.resetEncoders();
			mGearArm.closeClaw();
			mGearArm.homeGearArm();
			timer.start();
		}catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
		}
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		mDrive.resetEncoders();
		mGearArm.closeClaw();
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
//		autonomousCommand = chooser.getSelected();
//
//		/*
//		 * String autoSelected = SmartDashboard.getString("Auto Selector",
//		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
//		 * = new MyAutoCommand(); break; case "Default Auto": default:
//		 * autonomousCommand = new ExampleCommand(); break; }
//		 */
//
//		// schedule the autonomous command (example)
//		if (autonomousCommand != null)
//			autonomousCommand.start();
		try {
//            CrashTracker.logAutoInit();
//            if (mAutoModeExecuter != null) {
//                mAutoModeExecuter.stop();
//            }
//            mAutoModeExecuter = null;
//            // Configure loopers
//            mDisabledLooper.stop();
//            mEnabledLooper.start();
//
//            mAutoModeExecuter = new AutoModeExecuter();
//            mAutoModeExecuter.setAutoMode(mSmartDashBoardInteractions.getSelectedAutonMode());
//            mAutoModeExecuter.start();
			
			//Auto options
			//BLUE_MIDDLE_GEAR_AND_SHOOT()
			//RED_MIDDLE_GEAR_AND_SHOOT()
			//BLUE_STARTING_RIGHT_SIDE_BOILER()
			//RED_STARTING_RIGHT_SIDE_BOILER()
			//BASELINE()
            mDrive.setHighGear(false);
            mDrive.resetEncoders();
            BLUE_MIDDLE_GEAR_AND_SHOOT();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
	}
	public void BLUE_MIDDLE_GEAR_AND_SHOOT(){
		System.out.println("Starting blue side middle gear and shoot!");
		mDrive.resetEncoders();
		mGearArm.CaptureAndLift();
		mDrive.driveSetInches(76, 0.6);//0.7
		System.out.println("Done Driving Forward");
		mGearArm.DropGear();
		mDrive.driveSetInches(-12, 0.6);//1
		mDrive.gyroTurn(-114);
		mDrive.driveSetInches(120, 0.8);//1.2
		shoot();
		System.out.println("Autonomous completed fully");
		//chassis.setDrivePIDSetpoint(chassis.convertInchesToTicks(77.5));
	}
	public void RED_MIDDLE_GEAR_AND_SHOOT(){
		System.out.println("Starting red side middle gear and shoot!");
		mDrive.resetEncoders();
		mGearArm.CaptureAndLift();
		mDrive.driveSetInches(76, 0.6);//0.7
		System.out.println("Done Driving Forward");
		mGearArm.DropGear();
		mDrive.driveSetInches(-12, 0.6);//1
		mDrive.gyroTurn(114);
		mDrive.driveSetInches(129, 0.8);//1.2
		shoot();
		System.out.println("Autonomous completed fully");
	}
	public void RED_STARTING_RIGHT_SIDE_BOILER() {
		System.out.println("Starting RED_STARTING_LEFT_SIDE_BOILER");
		mDrive.resetEncoders();
		double startTime = timer.get();
		//shoot for 8 seconds
		while(timer.get() - startTime < 8) {
			shoot();
		}
		dontShoot();
		System.out.println("start back");
		mDrive.driveSetInches(-14, 0.6);
		System.out.println("done back");
		System.out.println("start turn");
		mDrive.gyroTurn(-170);
		System.out.println("done turn");
		System.out.println("start drive");
		mDrive.resetEncoders();
		System.out.println("Done resetting encoders");
		mDrive.driveSetInches(60, 0.6);
		System.out.println("boiler done");
	}
	
	public void BLUE_STARTING_RIGHT_SIDE_BOILER() {
		System.out.println("Starting BLUE_STARTING_RIGHT_SIDE_BOILER");
		mDrive.resetEncoders();
		double startTime = timer.get();
		//shoot for 8 seconds
		while(timer.get() - startTime < 8) {
			shoot();
		}
		dontShoot();
//		System.out.println("start back");
//		mDrive.driveSetInches(-14, 0.6);
//		System.out.println("end back");
//		System.out.println("start turn");
//		mDrive.gyroTurn(170);
//		System.out.println("done turn");
//		System.out.println("start drive");
//		mDrive.resetEncoders();
//		mDrive.driveSetInches(60, 0.6);
//		System.out.println("boiler done");
	}
	
	public void BASELINE(){
		System.out.println("Starting Baseline");
		mDrive.driveSetInches(80, 0.7);
		System.out.println("Autonomous completed fully");
	}
//	public void RED_SHOOT_AND_BASELINE(){
//		System.out.println("Starting Red Side Shoot And Baseline");
//		shoot();
//		Shoot for 7 seconds;
//		dontShoot();
//		mDrive.driveSetInches(-5, 0.6);
//		mDrive.gyroTurn(-115);
//		mDrive.driveSetInches(80, 0.8);
//		System.out.println("Autonomous completed fully");
//	}
//	public void Blue_SHOOT_AND_BASELINE(){
//		System.out.println("Starting Red Side Shoot And Baseline");
//		shoot();
//		Shoot for 7 seconds;
//		dontShoot();
//		mDrive.driveSetInches(-5, 0.6);
//		mDrive.gyroTurn(115);
//		mDrive.driveSetInches(80, 0.8);
//		System.out.println("Autonomous completed fully");
//	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		try {
//            outputAllToSmartDashboard();
//            updateDriverFeedback();
			if(mShooter.isReady()){
				mShooter.serializerOn();
				mConveyor.conveyorOn();
			}
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    private static final String COLOR_BOX_COLOR_KEY = "color_box_color";
    private static final String COLOR_BOX_TEXT_KEY = "color_box_text";

    private void updateDriverFeedback() {
 //       boolean hasBall = mSuperstructure.getIntake().hasBall();
 //       mHasBallLightOutput.set(hasBall);
 //       if (hasBall) {
  //          SmartDashboard.putString(COLOR_BOX_COLOR_KEY, "#00ff00");
 //           SmartDashboard.putString(COLOR_BOX_TEXT_KEY, "HAVE BALL");
 //       } else {
 //           SmartDashboard.putString(COLOR_BOX_COLOR_KEY, "#ff0000");
 //           SmartDashboard.putString(COLOR_BOX_TEXT_KEY, "NO BALL");
 //       }
    }
	

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
//		if (autonomousCommand != null)
//			autonomousCommand.cancel();
		try{
			// Configure loopers
			dontShoot();
            mDisabledLooper.stop();
            mEnabledLooper.start();
            mDrive.setOpenLoop(DriveSignal.NEUTRAL);
            mDrive.setBrakeMode(false);
            mDrive.resetEncoders();
		}catch (Throwable t){
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		
		try{
			double throttle = mControls.getThrottle();
            double turn = mControls.getTurn();
            
            
//                mDrive.setBrakeMode(false);
//                mDrive.setHighGear(!mControls.getLowGear());
                mDrive.setOpenLoop(mCheesyDriveHelper.cheesyDrive(throttle, turn, mControls.getQuickTurn()));
                mDrive.setHighGear(!mControls.getLowGear());
                if(mControls.getLowerGearArm() || mControls.getCoLowerGearArm()){
                	mGearArm.switchGetGearToggle();
                }
                if(mControls.getDropGear()){
                	mGearArm.setWantsToDrop(true);
                }
                if(mControls.getShooter()){
                	mShooter.switchToggle();
                }
                if(mControls.getClimber() < 0){
                	mClimber.climberSet(-mControls.getClimber());
                }else mClimber.climberOff();
                if(mShooter.getToggle()){
                	shoot();
                }else dontShoot();
                if(mControls.getReverseShooter() || mControls.getCoReverseShooter()){
                	mShooter.reverseShooter();
                	mConveyor.conveyorReverse();
                }
                if(mControls.getManualClose()){
                	mGearArm.setClawToggle(true);;
                }
                mGearArm.update();
                		
                
                
	}catch (Throwable t) {
        CrashTracker.logThrowableCrash(t);
        throw t;
        
	}
}
	public void shoot(){
		mShooter.shooterOn();
		if(mShooter.isReady()){
			mShooter.serializerOn();
			mConveyor.conveyorOn();
		}
	}
	public void dontShoot(){
		mShooter.shooterOff();
		mShooter.serializerOff();
		mConveyor.conveyorOff();
	}
	public void outputAllToSmartDashboard(){
		mGearArm.outputToSmartDashboard();
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}
