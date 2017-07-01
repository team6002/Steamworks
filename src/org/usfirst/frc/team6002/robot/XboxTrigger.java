package org.usfirst.frc.team6002.robot;

import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.GenericHID;

public class XboxTrigger {
	private GenericHID joystick;
	private int axisNum;
	private double threshold;
	private boolean prevVal;
	private boolean currVal;

	public XboxTrigger(GenericHID joystick, int axisNum){
		this.joystick = joystick;
		this.axisNum = axisNum;
		prevVal = false;
		currVal = false;
	}

	public void updateCurrentValue(){
		if(joystick.getRawAxis(axisNum) > threshold){
			currVal = true;
		}
		else{
			currVal = false;
		}
	}

	public boolean edgeTrigger(){
		return currVal == true && prevVal == false;
	}

	public void updatePreviousValue(){
		prevVal = currVal;
	}

	public double getAxisValue(){
		return joystick.getRawAxis(axisNum);
	}
}