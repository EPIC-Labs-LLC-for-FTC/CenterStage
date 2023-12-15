package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotObjects.EPIC.Adventurer.BluePipe;
import org.firstinspires.ftc.teamcode.RobotObjects.EPIC.Adventurer.Mecanum_Wheels;

@Autonomous(name = "Adventure Blue Left Auton",group = "Competition")

public class AdventureBlueLeftAuton extends LinearOpMode {
//    OpenCvCamera webcam;
//    Scanner scanner;

    public DcMotorEx spinTake = null;
    @Override
    public void runOpMode() throws InterruptedException {

        spinTake = hardwareMap.get(DcMotorEx.class, "spin");
        BluePipe pipeline = new BluePipe(telemetry);
        pipeline.map(hardwareMap);
        pipeline.initialize();
        Mecanum_Wheels wheels = new Mecanum_Wheels(hardwareMap);
        wheels.parent = this;
        wheels.IsAutonomous = true;
        wheels.telemetry = this.telemetry;
        wheels.initialize();
        waitForStart();

        double distance = 5.25;
        String location = "Left"; // pipeline.getLocation();
        sleep(500);
        switch (location){
            case "Left":
                //Auton code if the camera detects the object to the left
                wheels.encoderDrive(0.6,   -distance, distance, distance, -distance, 2);
                distance = 24;
                wheels.encoderDrive(0.6,   distance, distance, distance, distance, 2);
                spinTake.setPower(-1);
                sleep(1000);
                spinTake.setPower(0);
                distance = 22;
                wheels.encoderDrive(0.6,   -distance, -distance, -distance, -distance, 2);
                distance = 40;
                wheels.encoderDrive(0.6,   -distance, distance, distance, -distance, 2);
                break;

            case "Middle":
                //Auton code for middle object
                break;

            case "Right":
                //Auton code for right object
                break;
        }
//        sleep(2000);
        //double distance = 40;
        //wheels.leftErrorAdjustment = 0.75;
        //double diff = 0;
        //Left Straf
        //wheels.encoderDrive(0.6,   -distance, distance, -distance, distance, 2);
       while(opModeIsActive()){

            }

        }


    }

