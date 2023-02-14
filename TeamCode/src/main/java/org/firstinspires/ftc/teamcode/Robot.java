package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeServos;

public class Robot {

    // Hardware components (motors, sensors, servos, etc)
    public Gripper      gripper;
    public Slides       slides;
    public Arm          arm;
    public Drivetrain   drivetrain;
    public Intake       intake;
    public IntakeServos intakeServos;


    public void init(HardwareMap hardwareMap){

    }

}
