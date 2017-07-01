package org.usfirst.frc.team6002.robot;

import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class XboxButton {
	private JoystickButton button;
	private boolean prevVal;
	private boolean currVal;

	public XboxButton(JoystickButton button){
		this.button = button;
		prevVal = false;
		currVal = false;
	}

	public JoystickButton getJoystickButton(){
		return button;
	}

	public void updateCurrentValue(){
		currVal = button.get();
	}

	public boolean edgeTrigger(){
		return currVal == true && prevVal == false;
	}

	public void updatePreviousValue(){
		prevVal = currVal;
	}

	public boolean getButtonValue(){
		return button.get();
	}
}
