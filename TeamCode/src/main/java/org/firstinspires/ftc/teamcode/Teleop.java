package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Teleop extends LinearOpMode {
    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;
    @Override
    public void runOpMode() throws InterruptedException {
        frontRight=hardwareMap.get(DcMotor.class,"frontRight");
        frontLeft=hardwareMap.get(DcMotor.class,"frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");


        frontRight.setPower(gamepad1.left_stick_y-gamepad1.right_stick_x);
        frontLeft .setPower(gamepad1.left_stick_y+gamepad1.right_stick_x);
        backRight.setPower(gamepad1.left_stick_y-gamepad1.right_stick_x);
        backLeft.setPower(gamepad1.left_stick_y+gamepad1.right_stick_x);
    }
}
