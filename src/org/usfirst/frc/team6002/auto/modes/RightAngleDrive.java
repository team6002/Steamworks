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
public class RightAngleDrive extends AutoModeBase {

    private final double kDistanceToCDF = 10.0;
//    private final double kDistanceToDrive = 5;

    

    @Override
    protected void routine() throws AutoModeEndedException {

        List<Waypoint> first_path = new ArrayList<>();
        first_path.add(new Waypoint(new Translation2d(0, 0), 1.0));
        first_path.add(new Waypoint(new Translation2d(kDistanceToCDF, 0), 1.0));
        first_path.add(new Waypoint(new Translation2d(0,0), 1.0));
        
        runAction(new FollowPathAction(new Path(first_path), false));
        
    }
}

