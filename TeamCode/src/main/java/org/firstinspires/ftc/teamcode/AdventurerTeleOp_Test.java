package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotObjects.EPIC.Adventurer.Mecanum_Wheels;

@TeleOp()
public class AdventurerTeleOp_Test extends LinearOpMode {

    Mecanum_Wheels wheels = null;

    @Override
    public void runOpMode() {
        wheels = new Mecanum_Wheels(hardwareMap);
        //wheels.IsAutonomous = false;
        wheels.parent = this;
        wheels.telemetry = this.telemetry;
        wheels.initialize();
        waitForStart();
        double lefty = 0;
        boolean x;
        boolean y;
        boolean a;
        boolean b;
        String motor = "";
        while (opModeIsActive()) {
            lefty = gamepad1.left_stick_y;

            x = gamepad1.x;
            a = gamepad1.a;
            b = gamepad1.b;
            y = gamepad1.y;

            if (x) {
                motor = "frontLeft";
            }
            else if (y) {
                motor = "frontRight";
            }
            else if (a) {
                motor = "backLeft";
            }
            else if (b) {
                motor = "backRight";
            }
            if (motor == "frontLeft") {
                wheels.moveFrontLeft(lefty);
            }
            else if (motor == "frontRight") {
                wheels.moveFrontRight(lefty);
            }
            else if (motor == "backLeft") {
                wheels.moveBackLeft(lefty);
            }
            else if (motor == "backRight") {
                wheels.moveBackRight(lefty);
            }
            telemetry.addData("Motor", motor);
            telemetry.addData("lefty", lefty);
            telemetry.update();

        }


    }


}
