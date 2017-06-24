package org.usfirst.frc.team6002.robot.subsystems;

import org.usfirst.frc.team6002.robot.Constants;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *	defines the motor used for climbing
 */
public class Climber extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	private VictorSP climberMotor;
	private VictorSP climberMotor2;
	private boolean climberToggle;
	
	public Climber(){
		climberMotor = new VictorSP(Constants.kClimberId); //check port number
		climberMotor2 = new VictorSP(Constants.kClimber2Id);
		climberToggle = false;
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    public void climberSet(double speed){
    	climberMotor.set(-speed);
    	climberMotor2.set(speed);
    }
    public void climberOn(){
    	climberMotor.set(-1);
    	climberMotor2.set(1);
    }
    public void climberOff(){
    	climberMotor.set(0.0);
    	climberMotor2.set(0.0);
    }
    public void switchToggle(){
    	climberToggle = !climberToggle;
    }
    
    public boolean getToggle(){
    	return climberToggle;
    }
    
    public void setToggle(boolean val){
    	climberToggle = val;
    }
}

