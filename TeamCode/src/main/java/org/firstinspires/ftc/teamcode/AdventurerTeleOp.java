package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="AdventurerTeleOp")
public class AdventurerTeleOp extends LinearOpMode {

    //Define motors
    public DcMotorEx leftFront = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx leftBack = null;
    public DcMotorEx  rightBack = null;
    public DcMotorEx spinTake = null;
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

    public void spinnerControl() {

        if (gamepad2.left_stick_y > 0) {
            spinTake.setPower(1);
        } else if (gamepad2.left_stick_y < 0) {
            spinTake.setPower(-1);
        }

    }

    @Override
    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");
        leftBack = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightBack = hardwareMap.get(DcMotorEx.class, "backRight");
        spinTake = hardwareMap.get(DcMotorEx.class, "spin");
        servo=hardwareMap.get(Servo.class,"servo");



        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinTake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                driverControl();
                spinnerControl();
            }
        }
    }
}
