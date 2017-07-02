package org.usfirst.frc.team6002.robot;

public class Constants {
	
	public static double kLooperDt = 0.01;
	
	//DRIVE ID
	public static int kLeftDriveMasterId = 1;
	public static int kLeftDriveSlaveId = 2;
	public static int kRightDriveMasterId = 3;
	public static int kRightDriveSlaveId = 4;
	
	//SHOOTER ID
	public static int kMasterShooterId = 16;
	public static int kSlaveShooterId = 17;
	
	//DRIVE PID
	public static double kDriveVelocityKp = 0.0;
	public static double kDriveVelocityKi = 0.0;
	public static double kDriveVelocityKd = 0.0;
	public static double kDriveVelocityKf = 1.87362/4;
	public static int kDriveVelocityIZone = 0;
	public static double kDriveVelocityRampRate = 0.0;
	
	//SHOOTER PID
	public static double kFShooterVelocity = 2.8416;
	public static double kPShooterVelocity = 10;//50;
	public static double kIShooterVelocity = 0.0;
	public static double kDShooterVelocity = 0.0;
	
	//Wheels
	public static double kDriveWheelDiameterInches = 4;
	public static double kTrackScrubFactor = 0.5;
	public static double kTrackLengthInches = 8.265;
    public static double kTrackWidthInches = 23.8;
    public static double kTrackEffectiveDiameter = (kTrackWidthInches * kTrackWidthInches
            + kTrackLengthInches * kTrackLengthInches) / kTrackWidthInches;
	
	// Path following constants
    public static double kPathFollowingLookahead = 24.0; // inches
    public static double kPathFollowingMaxVel = 120.0; // inches/sec
    public static double kPathFollowingMaxAccel = 80.0; // inches/sec^2
    
    //Compressor
    public static int kCompressorId = 0;
    
    //PWM
    public static int kClimberId = 1;
    public static int kClimber2Id = 4;//second climber on comp robot
    public static int kSerializerId = 0;
    public static int kConveyorId = 3;
    
    //SHOOTER and SERIALIZZER CONSTANTS
    public static double kSerializerVoltage = 0.6;
	public static double kShooterSpeed = 875;

	//CONVEYOR CONSTANTS
	public static double kConveyorVoltage = 0.45;
}
