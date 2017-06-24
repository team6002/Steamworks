package org.usfirst.frc.team6002.robot.subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;
import com.kauailabs.navx.frc.AHRS;

import java.util.Set;

import org.usfirst.frc.team6002.lib.util.AdaptivePurePursuitController;
import org.usfirst.frc.team6002.lib.util.DriveSignal;
import org.usfirst.frc.team6002.lib.util.Path;
import org.usfirst.frc.team6002.lib.util.RigidTransform2d;
import org.usfirst.frc.team6002.lib.util.Rotation2d;
import org.usfirst.frc.team6002.lib.util.SynchronousPID;
import org.usfirst.frc.team6002.robot.Constants;
import org.usfirst.frc.team6002.robot.Kinematics;
import org.usfirst.frc.team6002.robot.Loop;
import org.usfirst.frc.team6002.robot.RobotState;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Drive extends Subsystem {

	protected static final int kVelocityControlSlot = 0;
    private static Drive instance_ = new Drive();
    private double mLastHeadingErrorDegrees = 0;
    private boolean isHighGear = false;
    public static Drive getInstance() {
    	return instance_;
    }
    
    public enum DriveControlState{
    	OPEN_LOOP, BASE_LOCKED, VELOCITY_SETPOINT, VELOCITY_HEADING_CONTROL, PATH_FOLLOWING_CONTROL;
    }
    public static class VelocityHeadingSetpoint {
        private final double leftSpeed_;
        private final double rightSpeed_;
        private final Rotation2d headingSetpoint_;

        // Constructor for straight line motion
        public VelocityHeadingSetpoint(double leftSpeed, double rightSpeed, Rotation2d headingSetpoint) {
            leftSpeed_ = leftSpeed;
            rightSpeed_ = rightSpeed;
            headingSetpoint_ = headingSetpoint;
        }

        public double getLeftSpeed() {
            return leftSpeed_;
        }

        public double getRightSpeed() {
            return rightSpeed_;
        }

        public Rotation2d getHeading() {
            return headingSetpoint_;
        }
    }
    
    private CANTalon leftMaster_, leftSlave_, rightMaster_, rightSlave_;
    private AHRS gyro_;
    private DoubleSolenoid shifter_;
    private boolean isBrakeMode_ = true;
    private AdaptivePurePursuitController pathFollowingController_;
    private DriveControlState driveControlState_;
    private VelocityHeadingSetpoint velocityHeadingSetpoint_;
    private SynchronousPID velocityHeadingPid_;
    
    public Loop getLoop() {
        return mLoop;
    }
    
    private final Loop mLoop = new Loop() {
        @Override
        public void onStart() {
            setOpenLoop(DriveSignal.NEUTRAL);
            setBrakeMode(false);
        }

        @Override
        public void onLoop() {
            synchronized (Drive.this) {
//                if (stopOnNextCount_ && getSeesLineCount() > lastSeesLineCount_) {
//                    poseWhenStoppedOnLine_ = RobotState.getInstance().getLatestFieldToVehicle().getValue();
//                    stopOnNextCount_ = false;
//                    stop();
//                }
                switch (driveControlState_) {
                case OPEN_LOOP:
                    return;
                case BASE_LOCKED:
                    return;
                case VELOCITY_SETPOINT:
                    // Talons are updating the control loop state
                    return;
                case VELOCITY_HEADING_CONTROL:
                    updateVelocityHeadingSetpoint();
                    return;
				case PATH_FOLLOWING_CONTROL:
                    updatePathFollower();
                    if (isFinishedPath()) {
                        stop();
                    }
                    break;
                default:
                    System.out.println("Unexpected drive control state: " + driveControlState_);
                    break;
                }
            }
        }

        @Override
        public void onStop() {
            setOpenLoop(DriveSignal.NEUTRAL);
        }
    };
    
    private Drive(){
    	leftMaster_ = new CANTalon(Constants.kLeftDriveMasterId);
    	leftSlave_ = new CANTalon(Constants.kLeftDriveSlaveId);
    	rightMaster_ = new CANTalon(Constants.kRightDriveMasterId);
        rightSlave_ = new CANTalon(Constants.kRightDriveSlaveId);
        gyro_ = new AHRS(SPI.Port.kMXP);
        shifter_ = new DoubleSolenoid(0,1);
        // Get status at 100Hz
        leftMaster_.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);
        rightMaster_.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);
        
        leftMaster_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        leftMaster_.set(0);
        leftSlave_.changeControlMode(CANTalon.TalonControlMode.Follower);
        leftSlave_.set(Constants.kLeftDriveMasterId);
        rightMaster_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        rightMaster_.set(0);
        rightSlave_.changeControlMode(CANTalon.TalonControlMode.Follower);
        rightSlave_.set(Constants.kRightDriveMasterId);
        setBrakeMode(true);
        
     // Set up the encoders
        leftMaster_.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        if (leftMaster_.isSensorPresent(
                CANTalon.FeedbackDevice.QuadEncoder) != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
            DriverStation.reportError("Could not detect left drive encoder!", false);
        }
        leftMaster_.reverseSensor(true);
        leftMaster_.reverseOutput(false);
        leftSlave_.reverseOutput(false);
        rightMaster_.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        if (rightMaster_.isSensorPresent(
                CANTalon.FeedbackDevice.QuadEncoder) != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
            DriverStation.reportError("Could not detect right drive encoder!", false);
        }
        rightMaster_.reverseSensor(true);
        rightMaster_.reverseOutput(true);
        rightSlave_.reverseOutput(false);

        leftMaster_.configEncoderCodesPerRev(4096);
        rightMaster_.configEncoderCodesPerRev(4096);
        
        leftMaster_.configNominalOutputVoltage(+0.0f,  -0.0f);
        leftMaster_.configPeakOutputVoltage(+12.0f, -12.0f);
        rightMaster_.configNominalOutputVoltage(+0.0f,  -0.0f);
        rightMaster_.configPeakOutputVoltage(+12.0f, -12.0f);
        // Load velocity control gains
        leftMaster_.setPID(Constants.kDriveVelocityKp, Constants.kDriveVelocityKi, Constants.kDriveVelocityKd,
                Constants.kDriveVelocityKf, Constants.kDriveVelocityIZone, Constants.kDriveVelocityRampRate,
                kVelocityControlSlot);
        rightMaster_.setPID(Constants.kDriveVelocityKp, Constants.kDriveVelocityKi, Constants.kDriveVelocityKd,
                Constants.kDriveVelocityKf, Constants.kDriveVelocityIZone, Constants.kDriveVelocityRampRate,
                kVelocityControlSlot);
//       
//        velocityHeadingPid_ = new SynchronousPID(Constants.kDriveHeadingVelocityKp, Constants.kDriveHeadingVelocityKi,
//                Constants.kDriveHeadingVelocityKd);
//        velocityHeadingPid_.setOutputRange(-30, 30);
//        
        setOpenLoop(DriveSignal.NEUTRAL);
    }
    public void setHighGear(boolean high_gear){
    	isHighGear = high_gear;
    	if(isHighGear == true){
    		shifter_.set(DoubleSolenoid.Value.kForward);
    	}else{
    		shifter_.set(DoubleSolenoid.Value.kReverse);
    	}
    }
    protected synchronized void setLeftRightPower(double left, double right) {
        leftMaster_.set(left);
        rightMaster_.set(-right);
    }
    
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (driveControlState_ != DriveControlState.OPEN_LOOP) {
            leftMaster_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
            rightMaster_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
            driveControlState_ = DriveControlState.OPEN_LOOP;
        }
        setLeftRightPower(signal.leftMotor, signal.rightMotor);
    }
    
    public AHRS getGyro(){
    	return gyro_;
    }
    public synchronized Rotation2d getGyroAngle() {
        return Rotation2d.fromDegrees(gyro_.getAngle());
    }
    
    public synchronized void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        configureTalonsForSpeedControl();
        driveControlState_ = DriveControlState.VELOCITY_SETPOINT;
        updateVelocitySetpoint(left_inches_per_sec, right_inches_per_sec);
    }

    public synchronized void setVelocityHeadingSetpoint(double forward_inches_per_sec, Rotation2d headingSetpoint) {
        if (driveControlState_ != DriveControlState.VELOCITY_HEADING_CONTROL) {
            configureTalonsForSpeedControl();
            driveControlState_ = DriveControlState.VELOCITY_HEADING_CONTROL;
            velocityHeadingPid_.reset();
        }
        velocityHeadingSetpoint_ = new VelocityHeadingSetpoint(forward_inches_per_sec, forward_inches_per_sec,
                headingSetpoint);
        updateVelocityHeadingSetpoint();
    }
    private synchronized void updateVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        if (driveControlState_ == DriveControlState.VELOCITY_HEADING_CONTROL
                || driveControlState_ == DriveControlState.VELOCITY_SETPOINT
                || driveControlState_ == DriveControlState.PATH_FOLLOWING_CONTROL) {
            leftMaster_.set(inchesPerSecondToRpm(left_inches_per_sec));
            rightMaster_.set(inchesPerSecondToRpm(right_inches_per_sec));
        } else {
            System.out.println("Hit a bad velocity control state");
            leftMaster_.set(0);
            rightMaster_.set(0);
        }
    }

    private void updateVelocityHeadingSetpoint() {
        Rotation2d actualGyroAngle = getGyroAngle();

        mLastHeadingErrorDegrees = velocityHeadingSetpoint_.getHeading().rotateBy(actualGyroAngle.inverse())
                .getDegrees();

        double deltaSpeed = velocityHeadingPid_.calculate(mLastHeadingErrorDegrees);
        updateVelocitySetpoint(velocityHeadingSetpoint_.getLeftSpeed() + deltaSpeed / 2,
                velocityHeadingSetpoint_.getRightSpeed() - deltaSpeed / 2);
    }
    private void updatePathFollower() {
        RigidTransform2d robot_pose = RobotState.getInstance().getLatestFieldToVehicle().getValue();
        RigidTransform2d.Delta command = pathFollowingController_.update(robot_pose, Timer.getFPGATimestamp());
        Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);

        // Scale the command to respect the max velocity limits
        double max_vel = 0.0;
        max_vel = Math.max(max_vel, Math.abs(setpoint.left));
        max_vel = Math.max(max_vel, Math.abs(setpoint.right));
        if (max_vel > Constants.kPathFollowingMaxVel) {
            double scaling = Constants.kPathFollowingMaxVel / max_vel;
            setpoint = new Kinematics.DriveVelocity(setpoint.left * scaling, setpoint.right * scaling);
        }
        updateVelocitySetpoint(setpoint.left, setpoint.right);
    }
    public synchronized void followPath(Path path, boolean reversed) {
        if (driveControlState_ != DriveControlState.PATH_FOLLOWING_CONTROL) {
            configureTalonsForSpeedControl();
            driveControlState_ = DriveControlState.PATH_FOLLOWING_CONTROL;
            velocityHeadingPid_.reset();
        }
        pathFollowingController_ = new AdaptivePurePursuitController(Constants.kPathFollowingLookahead,
                Constants.kPathFollowingMaxAccel, Constants.kLooperDt, path, reversed, 0.25);
        updatePathFollower();
    }
    public synchronized boolean isFinishedPath() {
        return (driveControlState_ == DriveControlState.PATH_FOLLOWING_CONTROL && pathFollowingController_.isDone())
                || driveControlState_ != DriveControlState.PATH_FOLLOWING_CONTROL;
    }
    
    /** Functions for hacked autonomous
     *	
     */
    public void driveSetInches(int distance, double desiredSpeed){
    	double desiredEncoderTicks = distance / 0.00306640625;
    	double speed = desiredSpeed;
    	double minimumSpeed = 0.3;
    	double slowDownPoint = 6500; // within about 13 inches, start slow down
    	double gyroAdjustmentFactor = 0.09;
    	double gyroAdjustment = 0;
    	gyro_.reset();
    	rightMaster_.setPosition(0);
    	resetEncoders();
    	double currentPosition = rightMaster_.getEncPosition();
    	if(distance > 0){
    		//move forward
    		while(currentPosition < desiredEncoderTicks){
    			if(desiredEncoderTicks-currentPosition < slowDownPoint){
    				speed = desiredSpeed*((desiredEncoderTicks-currentPosition) / slowDownPoint);
    				if(speed < minimumSpeed){
    					speed = minimumSpeed;
    				}
    			}
    			gyroAdjustment = gyro_.getAngle() * gyroAdjustmentFactor;
    			System.out.println("Current:" + currentPosition + " Target:" + desiredEncoderTicks + " Speed:" + speed + " Correction" + gyroAdjustment 
    					+ "speed + gyroAdjustment" + (speed + gyroAdjustment));
    			setMotors(speed, speed + gyroAdjustment);
    			currentPosition = rightMaster_.getEncPosition();
    		}
    	}
    	else {
    		//move backwards
    		while(currentPosition > desiredEncoderTicks){
    			if(currentPosition - desiredEncoderTicks < slowDownPoint){
    				speed = desiredSpeed*((currentPosition - desiredEncoderTicks) / slowDownPoint);
    				if(speed < minimumSpeed){
    					speed = minimumSpeed;
    				}
    			}
    			System.out.println("Current:" + currentPosition + " Target:" + desiredEncoderTicks + " Speed:" + speed);
    			setMotors(-speed, -speed);
    			currentPosition = rightMaster_.getEncPosition();
    		}
    	}
    	setMotors(0.0, 0.0);
    	System.out.println("Current:" + currentPosition + " Target:" + desiredEncoderTicks + " Speed:" + speed);
    	System.out.println("Complete!");	
    }
    public void gyroTurn(double targetAngle){
    	gyro_.reset();
    	double desiredSpeed = 0.5;
    	double speed = desiredSpeed;
    	double minimumSpeed = 0.3;
    	double currentAngle = gyro_.getAngle();
    	double slowDownPoint = 30;// within 25 degrees, slow down
    	if(targetAngle > 0){
    		while(currentAngle < targetAngle){
    			if(targetAngle - currentAngle < slowDownPoint){
    				speed = desiredSpeed*((targetAngle - currentAngle) / slowDownPoint);
    				if(speed < minimumSpeed) speed = minimumSpeed;
    			}
    			setMotors(speed, -speed);
    			currentAngle = gyro_.getAngle();
    			System.out.println("CurrentAngle:" + currentAngle + " TargetAngle:" + targetAngle + " Speed:" + speed);
			}
    	}else{
    		while(currentAngle > targetAngle){
    			if(currentAngle - targetAngle < slowDownPoint){
    				speed = desiredSpeed*((currentAngle - targetAngle ) / slowDownPoint) ;
    				if(speed < minimumSpeed) speed = minimumSpeed;
    			}
    			setMotors(-speed, speed);
    			currentAngle = gyro_.getAngle();
    			System.out.println("CurrentAngle:" + currentAngle + " TargetAngle:" + targetAngle + " Speed:" + speed);
    		}
		}
    	setMotors(0.0, 0.0);
    	System.out.println("Final CurrentAngle:" + currentAngle + " TargetAngle:" + targetAngle + " Speed:" + speed);
    }
    public void setMotors(double speed, double speed2){
    	leftMaster_.set(speed);
    	rightMaster_.set(-speed2);
    }
    /**
     * Path Markers are an optional functionality that name the various
     * Waypoints in a Path with a String. This can make defining set locations
     * much easier.
     * 
     * @return Set of Strings with Path Markers that the robot has crossed.
     */
    public synchronized Set<String> getPathMarkersCrossed() {
        if (driveControlState_ != DriveControlState.PATH_FOLLOWING_CONTROL) {
            return null;
        } else {
            return pathFollowingController_.getMarkersCrossed();
        }
    }
   
    
    public synchronized void resetEncoders() {
    	leftMaster_.changeControlMode(TalonControlMode.PercentVbus);
    	rightMaster_.changeControlMode(TalonControlMode.PercentVbus);
        leftMaster_.setPosition(0);
        rightMaster_.setPosition(0);

        leftMaster_.setEncPosition(0);
        rightMaster_.setEncPosition(0);
    }
    private void configureTalonsForSpeedControl() {
        if (driveControlState_ != DriveControlState.VELOCITY_HEADING_CONTROL
                && driveControlState_ != DriveControlState.VELOCITY_SETPOINT
                && driveControlState_ != DriveControlState.PATH_FOLLOWING_CONTROL) {
            leftMaster_.changeControlMode(CANTalon.TalonControlMode.Speed);
            leftMaster_.setProfile(kVelocityControlSlot);
//            leftMaster_.setAllowableClosedLoopErr(Constants.kDriveVelocityAllowableError);
            rightMaster_.changeControlMode(CANTalon.TalonControlMode.Speed);
            rightMaster_.setProfile(kVelocityControlSlot);
//            rightMaster_.setAllowableClosedLoopErr(Constants.kDriveVelocityAllowableError);
//            setHighGear(true);
            setBrakeMode(true);
        }
    }
    public double getLeftDistanceInches() {
        return rotationsToInches(leftMaster_.getPosition());
    }

    public double getRightDistanceInches() {
        return rotationsToInches(rightMaster_.getPosition());
    }
    private static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }
    private static double inchesToRotations(double inches) {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
    }
    
    public void setBrakeMode(boolean on) {
        if (isBrakeMode_ != on) {
            leftMaster_.enableBrakeMode(on);
            leftSlave_.enableBrakeMode(on);
            rightMaster_.enableBrakeMode(on);
            rightSlave_.enableBrakeMode(on);
            isBrakeMode_ = on;
        }
    }
    
    private static double rotationsToInches(double rotations) {
        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }
    
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }
    /**
     * VelocityHeadingSetpoints are used to calculate the robot's path given the
     * speed of the robot in each wheel and the polar coordinates. Especially
     * useful if the robot is negotiating a turn and to forecast the robot's
     * location.
     */
    
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
}