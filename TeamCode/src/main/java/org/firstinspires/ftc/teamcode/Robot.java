package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;

/**
 * This is the Robot class. This will make your command-based robot code a lot smoother
 * and easier to understand.
 */
public class Robot {

    // ... in MyRobot.java (extends Robot)

    // enum to specify opmode type
    public enum OpModeType {
        TELEOP, AUTO
    }

    // the constructor with a specified opmode type
    public Robot(OpModeType type) {
        if (type == OpModeType.TELEOP) {
            initTele();
        } else {
            initAuto();
        }
    }

    /*
     * Initialize teleop or autonomous, depending on which is used
     */
    public void initTele() {
        // initialize teleop-specific scheduler
    }

    public void initAuto() {
        // initialize auto-specific scheduler
    }


    // ... in your opmode

    // our robot object
    Robot m_robot = new Robot(Robot.OpModeType.TELEOP);
}