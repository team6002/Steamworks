package org.usfirst.frc.team6002.robot.subsystems;

import org.usfirst.frc.team6002.robot.Constants;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * This subystem defines the intake and conveyor belt functions.
 */
public class Conveyor extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	private VictorSP intakeMotor;
	private VictorSP conveyorMotor;
	private boolean intakeToggle;
    private boolean reverseToggle;
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public Conveyor(){
    	conveyorMotor = new VictorSP(Constants.kConveyorId);
    }
    public void conveyorOn(){
    	conveyorMotor.set(Constants.kConveyorVoltage);
    }
    public void conveyorOff(){
    	conveyorMotor.set(0.0);
    }
    public void conveyorReverse(){
    	conveyorMotor.set(-0.5);
    }
}

