package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="New TeleOp Test", group="test")
public class NewTeleOpTest extends LinearOpMode {

    public DcMotorEx leftFront = null;

    public DcMotorEx rightFront = null;
    public DcMotorEx leftBack = null;
    public DcMotorEx  rightBack = null;
    public DcMotorEx slide = null;
    public DcMotorEx spinTake = null;
    public Servo wrist = null;
    public Servo arm1 = null;
    public Servo arm2 = null;
    public Servo claw1 = null;
    public Servo claw2 = null;
    double movement;
    double rotation;
    double strafe;

    public void spinnerControl() {

        if (gamepad1.right_bumper) {
            spinTake.setPower(1);
        } else {
            spinTake.setPower(0);
        }

        if (gamepad1.left_bumper){
            spinTake.setPower(-1);
        }else {
            spinTake.setPower(0);
        }

    }

    public void SlideAndArmControl() {

        if (gamepad2.right_bumper){
            slide.setPower(1);
        }else{
            slide.setPower(0);
        }

        if (gamepad2.left_bumper){
            slide.setPower(-1);
        }else{
            slide.setPower(0);
        }

        if (gamepad2.dpad_left){
            arm1.setPosition(0);
            arm2.setPosition(1);
        }

        if (gamepad2.dpad_right){
            arm1.setPosition(1);
            arm2.setPosition(0);
        }

        if (gamepad2.dpad_up){
            wrist.setPosition(0);
        }

        if (gamepad2.dpad_down){
            wrist.setPosition(0.5);
        }

        if (gamepad2.a){
            claw1.setPosition(1);
            claw2.setPosition(0);
        }

        if (gamepad2.b){
            claw1.setPosition(0);
            claw2.setPosition(1);
        }

        telemetry.addData("slidePosition", slide.getCurrentPosition());
    }

    public void driverControl() {
        movement = gamepad1.left_stick_y;
        rotation = gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;

        double magnitude = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2));
        double direction = Math.atan2(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        boolean precision = gamepad1.right_bumper;

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

    @Override
    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        spinTake = hardwareMap.get(DcMotorEx.class, "spinTake");
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        wrist = hardwareMap.get(Servo.class, "wrist");
        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 = hardwareMap.get(Servo.class, "arm2");
        claw1 = hardwareMap.get(Servo.class, "claw1");
        claw2 = hardwareMap.get(Servo.class, "claw2");

        leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinTake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                driverControl();
                SlideAndArmControl();
                spinnerControl();
            }
        }
    }
}
