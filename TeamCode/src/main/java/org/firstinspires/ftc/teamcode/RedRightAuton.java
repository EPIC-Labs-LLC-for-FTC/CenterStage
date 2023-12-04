package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotObjects.EPIC.Mecanum_Wheels;

@Autonomous(name = "Red Right Auton",group = "Competition")

public class RedRightAuton extends LinearOpMode {
//    OpenCvCamera webcam;
//    Scanner scanner;
    @Override
    public void runOpMode() throws InterruptedException {
        Mecanum_Wheels wheels = new Mecanum_Wheels(hardwareMap);
        wheels.parent = this;
        wheels.IsAutonomous = true;
        wheels.telemetry = this.telemetry;
        wheels.initialize();
        waitForStart();
//        sleep(2000);
        double distance = 40;
        //wheels.leftErrorAdjustment = 0.75;
        double diff = 0;
        //Left Straf
        wheels.encoderDrive(0.6,   distance, -distance, -distance, distance, 2);
       while(opModeIsActive()){

            }

        }


    }

