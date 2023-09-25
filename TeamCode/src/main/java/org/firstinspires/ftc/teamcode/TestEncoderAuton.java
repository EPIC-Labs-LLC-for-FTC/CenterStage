package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotObjects.EPIC.Mecanum_Wheels;
import org.openftc.easyopencv.OpenCvCamera;

@Disabled
@Autonomous(name = "TestEncodrAuton")
public class TestEncoderAuton extends LinearOpMode {
    OpenCvCamera webcam;
    public DcMotorEx slide1;
    public DcMotorEx slide2;

    public Servo arm1;
    public Servo arm2;
    public Servo outtakeArm;
    public Servo finger1;
    public Servo finger2;
    public Servo wrist1;
    public Servo wrist2;
    @Override
    public void runOpMode() throws InterruptedException {
        double distance = 0;
        Mecanum_Wheels mecanum = new Mecanum_Wheels(hardwareMap);
        mecanum.leftErrorAdjustment = 1;
        mecanum.parent = this;
        mecanum.IsAutonomous = true;
        mecanum.velocity = 400;
        mecanum.telemetry = this.telemetry;
        mecanum.initialize();
        finger1 = hardwareMap.get(Servo.class, "fingerRight");
        finger2 = hardwareMap.get(Servo.class, "fingerLeft");
        wrist1 = hardwareMap.get(Servo.class, "wristRight");
        wrist2 = hardwareMap.get(Servo.class, "wristLeft");
        outtakeArm= hardwareMap.get(Servo.class, "outtakeArm");
        arm1 = hardwareMap.get(Servo.class,"armRight");
        arm2 = hardwareMap.get(Servo.class, "armLeft");

        wrist2.setDirection(Servo.Direction.REVERSE);
        finger2.setDirection(Servo.Direction.REVERSE);
        arm2.setDirection(Servo.Direction.REVERSE);

        slide1 = hardwareMap.get(DcMotorEx.class, "slide1");
        slide2 = hardwareMap.get(DcMotorEx.class, "slide2");

        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        wrist1.setPosition(0.3);
        wrist2.setPosition(0.3);

        outtakeArm.setPosition(0.11);
        arm1.setPosition(0.44);
        arm2.setPosition(0.44);

        waitForStart();
        sleep(100);
        //forward
        distance = 51;
        mecanum.encoderDrive(0.8,distance,distance,distance,distance,2);
        //strafe left
        distance = 12.2;
        mecanum.encoderDrive(0.8,-distance,distance,distance,-distance,2);
        distance = 2.5;
        mecanum.encoderDrive(0.8,-distance,-distance,-distance,-distance,1);
        raiseSlide(2400);
        sleep(2500);
        outtakeArm.setPosition(0.55);
        sleep(1000);
        raiseSlide(0);
        sleep(2000);
        outtakeArm.setPosition(0.04);
        sleep(700);
        //strafe right
        distance = 13.25;
        mecanum.encoderDrive(0.8,distance,-distance,-distance,distance,2);
        //turn left
        distance = 21.4;
        mecanum.encoderDrive(0.8,-distance,-distance,distance,distance,2);
        //strafe right
        distance = 2;
        mecanum.encoderDrive(0.8,distance,-distance,-distance,distance,1);
        arm1.setPosition(0.24);
        arm2.setPosition(0.24);
        sleep(500);
        wrist1.setPosition(0);
        wrist2.setPosition(0);
        sleep(500);
        //backward
        distance = 16;
        mecanum.encoderDrive(0.8,-distance,-distance,-distance,-distance,2);
        finger1.setPosition(0.63);
        finger2.setPosition(0.48);
        sleep(500);
        outtakeArm.setPosition(0.05);
        arm1.setPosition(0.5);
        arm2.setPosition(0.5);
        wrist1.setPosition(0.66);
        wrist2.setPosition(0.66);
        sleep(700);
        finger1.setPosition(0.16);
        finger2.setPosition(0.16);
        sleep(200);
        //forward
        distance = 17.5;
        mecanum.encoderDrive(0.8,distance,distance,distance,distance,2);
        //strafe right
        distance = 14.5;
        mecanum.encoderDrive(0.8,distance,-distance,-distance,distance,2);
        //turn left
        distance = 1.25;
        mecanum.encoderDrive(0.8,-distance,-distance,-distance,-distance,1);
        raiseSlide(2400);
        sleep(2000);
        outtakeArm.setPosition(0.54);
        sleep(750);
        outtakeArm.setPosition(0.05);
        raiseSlide(0);
        sleep(1000);
        //strafe left
        distance = 13.5;
        mecanum.encoderDrive(0.8,-distance,distance,distance,-distance,2);









    }
    private void raiseSlide(int index){
        slide1.setPower(0.8);
        slide2.setPower(0.8);
        slide1.setTargetPosition(index);
        slide2.setTargetPosition(index);
        slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
}
