package org.usfirst.frc.team6002.robot;

import edu.wpi.first.wpilibj.Joystick;
import org.usfirst.frc.team6002.lib.util.LatchedBoolean;

public class ControlBoard {
	private static ControlBoard mInstance = new ControlBoard();
	private static LatchedBoolean mGearEdge = new LatchedBoolean();
	private static LatchedBoolean mCoGearEdge = new LatchedBoolean();
	private static LatchedBoolean mShooterEdge = new LatchedBoolean();
	
	public static ControlBoard getInstance(){
		return mInstance;
	}
	
	private Joystick mXbox;
	private Joystick mCoXbox;
	
	private ControlBoard() {
		mXbox = new Joystick(0);
		mCoXbox = new Joystick(1);
	}
	
	//DRIVER CONTROLS
	public double getThrottle(){
		return -mXbox.getRawAxis(1);
	}
	public double getTurn() {
		return mXbox.getRawAxis(4);
	}
	public boolean getQuickTurn(){
		return mXbox.getRawAxis(3) > 0.1;
	}
	public boolean getLowGear(){
		return mXbox.getRawAxis(2) > 0.1;
	}
	public boolean getLowerGearArm(){
		return mGearEdge.update(mXbox.getRawButton(1));
	}
	public boolean getDropGear(){
		return mXbox.getRawButton(2);
	}
	public boolean getShooter(){
		return mShooterEdge.update(mXbox.getRawButton(6));
	}
	public boolean getReverseShooter(){
		return mXbox.getRawButton(3);
	}
//	public boolean getTightGearPickUp(){
//		return mXbox.getRawButton(5);
//	}
	public boolean getManualClose(){
		return mXbox.getRawButton(5);
	}
	//CODRIVER CONTROLS
	public double getClimber(){
		return mCoXbox.getRawAxis(1);
	}
	public boolean getCoLowerGearArm(){
		return mCoGearEdge.update(mCoXbox.getRawButton(1));
	}
	public boolean getCoReverseShooter(){
		return mCoXbox.getRawButton(3);
	}
}
