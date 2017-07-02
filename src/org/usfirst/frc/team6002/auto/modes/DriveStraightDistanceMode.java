package org.usfirst.frc.team6002.auto.modes;


import org.usfirst.frc.team6002.auto.AutoModeBase;
import org.usfirst.frc.team6002.auto.AutoModeEndedException;
import org.usfirst.frc.team6002.auto.actions.*;

//import com.team254.frc2016.subsystems.Superstructure;
//import org.usfirst.frc.team6002.robot.subsystems.Superstructure.WantedState;
import org.usfirst.frc.team6002.lib.util.Path;
import org.usfirst.frc.team6002.lib.util.Translation2d;
import org.usfirst.frc.team6002.lib.util.Path.Waypoint;

import java.util.ArrayList;
import java.util.List;

/**
 * Test autonomomus that follows a rightangle path
 */
public class DriveStraightDistanceMode extends AutoModeBase {

    private final double kDistance = 15.0;
    private final double kVelocity = 10.0;
//    private final double kDistanceToDrive = 5;

    

    @Override
    protected void routine() throws AutoModeEndedException {
    	runAction(new DriveStraightAction (kDistance, kVelocity));
    }
}

