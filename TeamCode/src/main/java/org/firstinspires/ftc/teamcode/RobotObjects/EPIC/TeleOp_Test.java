package org.firstinspires.ftc.teamcode.RobotObjects.EPIC;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;



@TeleOp(name = "TeleOp_Test")
public class TeleOp_Test extends LinearOpMode {
    Mecanum_Wheels wheels;
    double lefty = 0.0;
    double lefty2 = 0.0;
    double leftx = 0.0;
    double righty = 0.0;
    double rightx = 0.0;
    double righty2 = 0.0;
    double speed = 1.0;

    private static void logGamepad(Telemetry telemetry, Gamepad gamepad, String prefix) {
        telemetry.addData(prefix + "Synthetic",
                gamepad.getGamepadId() == Gamepad.ID_UNASSOCIATED);
        for (Field field : gamepad.getClass().getFields()) {
            if (Modifier.isStatic(field.getModifiers())) continue;

            try {
                telemetry.addData(prefix + field.getName(), field.get(gamepad));
            } catch (IllegalAccessException e) {
            }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        Servo door = hardwareMap.get(Servo.class,"door");

        waitForStart();

        while (opModeIsActive()) {

            lefty = -gamepad1.left_stick_y;
            leftx = -gamepad1.left_stick_x;
            righty = gamepad1.right_stick_y;
            rightx = -gamepad1.right_stick_x;

            righty2 = gamepad2.right_stick_y;

            lefty2 = gamepad2.left_stick_y;

            boolean b = gamepad1.b;
            boolean x = gamepad1.x;
            boolean y = gamepad1.y;
            boolean a = gamepad1.a;
            if (gamepad2.b){door.setPosition(0);
            if (gamepad2.a){
                door.setPosition(1);
            }
            if (a) {
                speed = 0.6;
            } else if (b) {
                speed = 0.25;
            } else if (x){
                speed = 0.1;
            } else if (y){
                speed = 1.0;
            }
            if (Math.abs(lefty) > Math.abs(leftx)) {
                rightFront.setPower(lefty * speed);
                leftFront.setPower(-lefty * speed);
                rightRear.setPower(lefty * speed);
                leftRear.setPower(-lefty * speed);
            } else {
                rightFront.setPower(-leftx * speed);
                leftFront.setPower(-leftx * speed);
                rightRear.setPower(leftx * speed);
                leftRear.setPower(leftx * speed);

                if (rightx > 0) {
                    rightFront.setPower(rightx * speed);
                    leftFront.setPower(rightx * speed);
                    rightRear.setPower(rightx * speed);
                    leftRear.setPower(rightx * speed);
                } else {
                    rightFront.setPower(rightx * speed);
                    leftFront.setPower(rightx * speed);
                    rightRear.setPower(rightx * speed);
                    leftRear.setPower(rightx * speed);
                }
            }

            telemetry.addData("speed: ", speed);

            telemetry.update();

            telemetry.update();
        }
    }
}}

