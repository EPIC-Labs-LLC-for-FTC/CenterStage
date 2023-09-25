package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotObjects.EPIC.Outtake2023;

@Disabled
@TeleOp(name = "TestOuttake",group = "test")
public class TestOuttake extends LinearOpMode{

    public Outtake2023 outtake;
    double position = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        outtake = new Outtake2023(hardwareMap);
        outtake.telemetry = telemetry;
        outtake.parent = this;
        position = outtake.outtake.getPosition();
        waitForStart();
        while(opModeIsActive()){
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        if(x){
            position += 0.01;
            outtake.setPosition(position);
            telemetry.addData("Outtake position:", outtake.outtake.getPosition());
            telemetry.update();
        }
        if(b){
            position -= 0.01;
            outtake.setPosition(position);
            telemetry.addData("Outtake position:", outtake.outtake.getPosition());
            telemetry.update();
        }
        telemetry.addData("Outtake position:", outtake.outtake.getPosition());
        telemetry.update();
    }
}}
