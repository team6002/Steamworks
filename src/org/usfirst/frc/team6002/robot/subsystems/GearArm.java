package org.usfirst.frc.team6002.robot.subsystems;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team6002.robot.Robot;
import org.usfirst.frc.team6002.robot.Constants;
//import com.ctre.CANTalon;
//import com.ctre.CANTalon.TalonControlMode;

/**
 *
 */
public class GearArm extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
//	CANTalon gearArmMotor;
	DoubleSolenoid gearClaw, gearArmSolenoid;
	DigitalInput gearLimitSwitch;
	Counter gearSwitchCheck;
	private boolean wantsToDrop;
	private boolean clawStatus;
	private boolean hasGear;
	private boolean clawToggle;
	private boolean getGearToggle;
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	
    }
    private enum SystemState{
    	HOME,	//Arm in the up position
    	HELD, // Arm in home position with gear
    	LOWERED, //Arm lowered to the ground
    	RELEASED, //Gear is on peg
    	CAPTURED //Arm lowered to the ground with gear
    }
    private enum WantedState{
    	WANT_TO_CAPTURE_GEAR, //Wants to capture gear and raise arm
    	WANT_TO_PLACE_GEAR, //Wants to release gear on peg
    	WANT_TO_PREPARE_FOR_CAPTURE //Wants to get arm in position for capture
    }
    private SystemState mSystemState = SystemState.HELD;
    private WantedState mWantedState = WantedState.WANT_TO_PREPARE_FOR_CAPTURE;
    
    private boolean tightGearPickUp = false;
    public void update() {
    	if(getGetGearToggle()){
    		if(mSystemState == SystemState.HOME || mSystemState == SystemState.RELEASED || mSystemState == SystemState.HELD){
    			lowerGearArm();
    			if(mSystemState == SystemState.HELD){
    				mSystemState = SystemState.CAPTURED;
    			}else mSystemState = SystemState.LOWERED;
    		}
    		else if(mSystemState == SystemState.LOWERED || mSystemState == SystemState.CAPTURED){
    			homeGearArm();
    			if(mSystemState == SystemState.LOWERED){
    				mSystemState = SystemState.HOME;
    			}else{
    				mSystemState = SystemState.HELD;
    			}
    		}
    		setGetGearToggle(false);
    	}else if(wantsToDrop == true){	
//    		lowerGear();
    		DropGear();
    		setClawStatus(false);
    	}else if(gearLimitSwitch.get() == false && mSystemState == SystemState.LOWERED){
    		Timer.delay(0.1);
    		CaptureAndLift();
    		setClawStatus(true);
    	}else if(clawToggle && mSystemState == SystemState.LOWERED){
    		closeClaw();
    		setClawStatus(true);
    		setClawToggle(false);
    		mSystemState = SystemState.CAPTURED;
    	}
    	outputToSmartDashboard();
//    	System.out.println(mSystemState);
    }
    public void outputToSmartDashboard(){
    	SmartDashboard.putString("SystemState", mSystemState.name());
    	SmartDashboard.putBoolean("IsGearIn", gearLimitSwitch.get());
    	SmartDashboard.putBoolean("HOLD", gearHold());
    	SmartDashboard.putBoolean("LOWERED", loweredArm());
    	SmartDashboard.putBoolean("CLOSED CLAW", clawStatus);
//    	SmartDashboard.putBoolean("TightGearSwitch", getTightGearPickUp());
    }
    public boolean gearHold(){
    	return mSystemState == SystemState.HELD || mSystemState == SystemState.CAPTURED;
    }
    public boolean loweredArm(){
    	return mSystemState == SystemState.LOWERED;
    }
    public void CaptureAndLift(){
    	closeClaw();
    	setClawStatus(true);
		if(tightGearPickUp){
//			backup
			homeGearArm();
			tightGearPickUp = false; //Clear out indicator
			mSystemState = SystemState.HELD;
			System.out.println("backup");
		}else{
			mSystemState = SystemState.CAPTURED;
		}
    }
    public boolean clawStatus(){
    	return clawStatus;
    }
    public void setClawStatus(boolean val){
    	clawStatus = val;
    }
    public void DropGear(){
    	openClaw();
		wantsToDrop = false;
		mSystemState = SystemState.RELEASED;
    }
    public GearArm(){
       	getGearToggle = false;
       	hasGear = false;
       	clawToggle = false;
    	gearLimitSwitch = new DigitalInput(0);
    	gearSwitchCheck = new Counter(gearLimitSwitch);
    	gearClaw = new DoubleSolenoid(2,3);
    	gearArmSolenoid = new DoubleSolenoid(4,5);
    }
    public void homeGearArm(){
    	gearArmSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
    public void lowerGearArm(){
    	gearArmSolenoid.set(DoubleSolenoid.Value.kForward);
    }
	public void switchGetGearToggle(){
		getGearToggle = !getGearToggle;
	}
	public void setGetGearToggle(boolean val){
		getGearToggle = val;
	}
	public boolean getGetGearToggle(){
		return getGearToggle;
	}
	public void openClaw(){
		gearClaw.set(DoubleSolenoid.Value.kReverse);
	}
	public void closeClaw(){
		gearClaw.set(DoubleSolenoid.Value.kForward);
	}
	public boolean getClawToggle(){
		return clawToggle;
	}
	public void setClawToggle(boolean value){
		clawToggle = value;
	}
	public void switchClawToggle(){
		clawToggle = !clawToggle;
	}
	public void setTightPickUp(boolean val){
		tightGearPickUp = val;
	}
	public void switchTightPickUp(){
		tightGearPickUp = !tightGearPickUp;
	}
	private boolean getTightGearPickUp(){
		return tightGearPickUp;
	}
	public DigitalInput gearLimitSwitch(){
		return gearLimitSwitch;
	}
	public void resetSwitchCounter(){
		gearSwitchCheck.reset();
	}
	public boolean isGearIn(){
		return gearSwitchCheck.get() > 0;
	}
	public boolean wantsToDrop(){
		return wantsToDrop();
	}
	public void setWantsToDrop(boolean val){
		wantsToDrop = val;
	}
	
}