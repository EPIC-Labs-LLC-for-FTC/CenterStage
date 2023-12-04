package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Teleop extends LinearOpMode {
    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;
    @Override
    public void runOpMode() throws InterruptedException {
        frontRight=hardwareMap.get(DcMotorEx.class,"frontRight");
        frontLeft=hardwareMap.get(DcMotorEx.class,"frontLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");


        frontRight.setPower(gamepad2.left_stick_y-gamepad1.right_stick_x);
        frontLeft .setPower(gamepad1.left_stick_y+gamepad1.right_stick_x);
        backRight.setPower(gamepad1.left_stick_y-gamepad1.right_stick_x);
        backLeft.setPower(gamepad1.left_stick_y+gamepad1.right_stick_x);
    }
}
