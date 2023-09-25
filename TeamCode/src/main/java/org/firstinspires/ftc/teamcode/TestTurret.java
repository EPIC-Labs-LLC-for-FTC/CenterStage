package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotObjects.EPIC.Turret2023;

@Disabled
@TeleOp(name = "TestTurret",group = "test")
public class TestTurret extends LinearOpMode{

    public Turret2023 turret;
    double position;


    @Override
    public void runOpMode() throws InterruptedException {
        turret = new Turret2023(hardwareMap);
        turret.telemetry = telemetry;
        turret.parent = this;
        position = turret.turret.getPosition();
        waitForStart();
        while (opModeIsActive()){
            boolean x = gamepad1.x;
            boolean y = gamepad1.y;
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            if(x){
                position += 0.01;
                turret.setPosition(position);
                telemetry.addData("Outtake position:", turret.turret.getPosition());
                telemetry.update();
                sleep(700);
            }
            if(b){
                position -= 0.01;
                turret.setPosition(position);
                telemetry.addData("Outtake position:",turret.turret.getPosition());
                telemetry.update();
                sleep(700);
            }
            telemetry.addData("Outtake position:", turret.turret.getPosition());
            telemetry.addData("pole position:", turret.getPoleDist());
            telemetry.update();
//            turret.turretIncrease();
//            sleep(2000);
//            turret.turretIncrease();
//            sleep(2000);
//            turret.turretIncrease();
//            sleep(2000);
//            turret.turretDecrease();
//            sleep(2000);
//            turret.turretDecrease();
//            sleep(2000);
//            turret.turretDecrease();
//            turret.home();
//            sleep(2000);
        }
    }}
