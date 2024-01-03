package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp()
public class AdventurerTeleOp extends LinearOpMode {

    //Define motors
    public DcMotorEx leftFront = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx leftBack = null;
    public DcMotorEx  rightBack = null;
   // public DcMotorEx spintake = null;
    public DcMotorEx arm = null;

    public Servo servo;

    double movement;
    double rotation;
    double strafe;

    public void driverControl() {
        movement = gamepad1.left_stick_y;
        rotation = gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;

        double magnitude = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2));
        double direction = Math.atan2(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        boolean precision = gamepad1.right_bumper;

        //INFO Increasing speed to a maximum of 1
        double lf = magnitude * Math.sin(direction + Math.PI / 4) + rotation;
        double lb = magnitude * Math.cos(direction + Math.PI / 4) + rotation;
        double rf = magnitude * Math.cos(direction + Math.PI / 4) - rotation;
        double rb = magnitude * Math.sin(direction + Math.PI / 4) - rotation;

        double hypot = Math.hypot(movement, strafe);
        double ratio;
        if (movement == 0 && strafe == 0)
            ratio = 1;
        else if (precision)
            ratio = hypot / (Math.max(Math.max(Math.max(Math.abs(lf), Math.abs(lb)), Math.abs(rb)), Math.abs(rf))) / 2;
        else
            ratio = hypot / (Math.max(Math.max(Math.max(Math.abs(lf), Math.abs(lb)), Math.abs(rb)), Math.abs(rf)));

        leftFront.setPower(ratio * lf);
        leftBack.setPower(ratio * lb);
        rightFront.setPower(ratio * rf);
        rightBack.setPower(ratio * rb);
    }

    public void armLift() {
        if(gamepad2.a) {
            arm.setTargetPosition(0);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1);
        } else if(gamepad2.x) {
            arm.setTargetPosition(1000);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1);
        } else if(gamepad2.y) {
            arm.setTargetPosition(3500);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1);
        }

    }

    public void droneControl() {

        if (gamepad2.dpad_up) {
            servo.setDirection(Servo.Direction.FORWARD);
            servo.setPosition(1);
        }

    }


    @Override
    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");
        leftBack = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightBack = hardwareMap.get(DcMotorEx.class, "backRight");
        servo=hardwareMap.get(Servo.class,"servo");
        arm = hardwareMap.get(DcMotorEx.class, "arm");



        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                int position;
                position = arm.getCurrentPosition();
                telemetry.addData("Motor Position", position);
                telemetry.update();

                driverControl();
                armLift();
                droneControl();

            }
        }
    }
}
