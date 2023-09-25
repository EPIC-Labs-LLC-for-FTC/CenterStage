package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotObjects.EPIC.Arm2023;
import org.firstinspires.ftc.teamcode.RobotObjects.EPIC.Claw2023;

@TeleOp(name = "TestArm",group = "test")
public class TestArm extends LinearOpMode{

    public Arm2023 arm;
    public Claw2023 finger;
    //0.72
    //0.74
    //0.79
    //0.82
    //0.84



    @Override
    public void runOpMode() throws InterruptedException {
        finger = new Claw2023(hardwareMap);
        finger.telemetry = telemetry;
        finger.parent = this;
        arm = new Arm2023(hardwareMap);
        arm.telemetry = telemetry;
        arm.parent = this;
        double armPosition = 0.0;
        arm.doGround();
        boolean x1 = gamepad1.x;
        boolean y1 = gamepad1.y;
        boolean a1 = gamepad1.a;
        boolean b1 = gamepad1.b;
        arm.doInitialize();
        sleep(1000);
        arm.doLevel4();
        waitForStart();
        while(opModeIsActive()){

            armPosition = 0;

            x1 = gamepad1.x;
            y1 = gamepad1.y;
            a1 = gamepad1.a;
            b1 = gamepad1.b;
            if(x1){
                armPosition += 0.05;
                arm.armset(armPosition);
                sleep(700);
            }
            if(b1){
                armPosition -= 0.05;
                arm.armset(armPosition);
                sleep(700);
            }
            if(a1){
                finger.grab();
                sleep(700);
            }
            if(y1){
                finger.release();
                sleep(700);
            }

            telemetry.addData("arm: %d", arm.getPosition());
            telemetry.update();

//            arm.ground();
//            sleep(2000);
//            arm.level2();
//            sleep(2000);
//            arm.level3();
//            sleep(2000);
//            arm.level4();
//            sleep(2000);
//            arm.level5();
//            sleep(2000);
//            arm.dropping();
//            sleep(2000);

    }
}}
