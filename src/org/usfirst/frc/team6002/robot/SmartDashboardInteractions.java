package org.usfirst.frc.team6002.robot;

import org.usfirst.frc.team6002.auto.AutoModeBase;
import org.usfirst.frc.team6002.auto.AutoModeEndedException;
import org.usfirst.frc.team6002.auto.modes.*;
import org.usfirst.frc.team6002.robot.subsystems.ShooterAimingParameters;
import org.usfirst.frc.team6002.lib.util.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.json.simple.JSONArray;

/**
 * Controls the interactive elements of SmartDashboard.
 *
 * Keeps the network tables keys in one spot and enforces autonomous mode
 * invariants.
 */
public class SmartDashboardInteractions {
    private static final String AUTO_OPTIONS = "auto_options";
    private static final String SELECTED_AUTO_MODE = "selected_auto_mode";
    private static final String SELECTED_AUTO_LANE = "selected_auto_lane";

    private static final AutonOption DEFAULT_MODE = AutonOption.TEST_DRIVE;
    private static final AutonLane DEFAULT_LANE = AutonLane.LANE_4;

    public void initWithDefaults() {

        JSONArray autoOptionsArray = new JSONArray();
        for (AutonOption autonOption : AutonOption.values()) {
            autoOptionsArray.add(autonOption.name);
        }
        SmartDashboard.putString(AUTO_OPTIONS, autoOptionsArray.toString());
        SmartDashboard.putString(SELECTED_AUTO_MODE, DEFAULT_MODE.name);
        SmartDashboard.putString(SELECTED_AUTO_LANE, DEFAULT_LANE.numberString);
    }

    public AutoModeBase getSelectedAutonMode() {
        String autoModeString = SmartDashboard.getString(SELECTED_AUTO_MODE, DEFAULT_MODE.name);
        AutonOption selectedOption = DEFAULT_MODE;
        for (AutonOption autonOption : AutonOption.values()) {
            if (autonOption.name.equals(autoModeString)) {
                selectedOption = autonOption;
                break;
            }
        }

        String autoLaneString = SmartDashboard.getString(SELECTED_AUTO_LANE, DEFAULT_LANE.numberString);
        AutonLane selectedLane = DEFAULT_LANE;
        for (AutonLane autonLane : AutonLane.values()) {
            if (autonLane.numberString.equals(autoLaneString)) {
                selectedLane = autonLane;
            }
        }

        return createAutoMode(selectedOption, selectedLane);
    }

    /**
     * I don't trust {@link SendableChooser} to manage {@link AutoModeBase}
     * objects directly, so use this enum to project us from WPILIb.
     */
    enum AutonOption {
    	RIGHT_ANGLE_DRIVE("drive"),
    	DRIVE_STRAIGHT_DISTANCE("driveDistance"),
//        STAY_HIGH_ONE_BALL_DRIVE_BACK("No Drop Drive Back"), //
//        STAY_HIGH_ONE_BALL("No Drop Stay"), //
//        GET_LOW_ONE_BALL("Portcullis - STOP"), //
//        GET_LOW_COME_BACK_LEFT("Portcullis - Come back left"), //
//        GET_LOW_COME_BACK_RIGHT("Portcullis - Come back right"), //
//        CDF_ONE_BALL("CDF - Stop"), //
//        CDF_COME_BACK_LEFT("CDF - Come back left"), //
//        CDF_COME_BACK_RIGHT("CDF - Come back right"), //
//        TWO_BALL_LOW_BAR("Low Bar - Two Ball"), //
//        TWO_BALL_CLASS_BD("Class B/D - Two Ball"), //
        STAND_STILL("Stand Still"), //
        TEST_DRIVE("TEST ONLY Driving");

        public final String name;

        AutonOption(String name) {
            this.name = name;
        }
    }

    enum AutonLane {
        LANE_1(160, "1"), LANE_2(205, "2"), LANE_3(160, "3"), LANE_4(155, "4"), LANE_5(220, "5");

        public final double distanceToDrive;
        public final String numberString;

        AutonLane(double distanceToDrive, String numberString) {
            this.distanceToDrive = distanceToDrive;
            this.numberString = numberString;
        }
    }

    private ShooterAimingParameters getAimingHintForLane(AutonLane lane) {
        if (lane == AutonLane.LANE_1) {
            return new ShooterAimingParameters(160.0, Rotation2d.fromDegrees(-45), -1);
        } else if (lane == AutonLane.LANE_2) {
            return new ShooterAimingParameters(150.0, Rotation2d.fromDegrees(-30), -1);
        } else if (lane == AutonLane.LANE_3) {
            return new ShooterAimingParameters(140.0, Rotation2d.fromDegrees(-15), -1);
        } else if (lane == AutonLane.LANE_4) {
            return new ShooterAimingParameters(140.0, Rotation2d.fromDegrees(10), -1);
        } else { /* if (lane == AutonLane.LANE_5) */
            return new ShooterAimingParameters(140.0, Rotation2d.fromDegrees(25), -1);
        }
    }

    private AutoModeBase createAutoMode(AutonOption autonOption, AutonLane autonLane) {
        switch (autonOption) {
        case RIGHT_ANGLE_DRIVE:
        	return new RightAngleDrive();
        case DRIVE_STRAIGHT_DISTANCE:
        	return new DriveStraightDistanceMode();
        case TEST_DRIVE:
            return new AutoModeBase() {
                @Override
                protected void routine() throws AutoModeEndedException {
                    throw new RuntimeException("Expected exception!!!");
                }
            };

        case STAND_STILL: // fallthrough
        default:
            System.out.println("ERROR: unexpected auto mode: " + autonOption);
            return new StandStillMode();
        }
    }
}
