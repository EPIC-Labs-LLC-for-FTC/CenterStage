package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotObjects.EPIC.Arm2023;
import org.firstinspires.ftc.teamcode.RobotObjects.EPIC.IntakeSlide2023;
import org.firstinspires.ftc.teamcode.RobotObjects.EPIC.OuttakeSlide2023;

@Disabled
@TeleOp(name = "Test Claw Slide",group = "test")
public class TestSlider extends LinearOpMode{

    public IntakeSlide2023 slide;
    public OuttakeSlide2023 outtakeSlide;
    public Arm2023 arm;
    int outtakePosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = new Arm2023(hardwareMap);
        arm.telemetry = telemetry;
        arm.parent = this;
//        arm.ground();

        slide = new IntakeSlide2023(hardwareMap);
        slide.telemetry = telemetry;
        slide.parent = this;
//        slide.in();
        outtakeSlide = new OuttakeSlide2023(hardwareMap);
        outtakeSlide.telemetry = telemetry;
        outtakeSlide.parent = this;
//        slide.in();
//        arm.dropping();
        waitForStart();
        while(opModeIsActive()){
            boolean x = gamepad1.x;
            boolean y = gamepad1.y;
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
//            arm.level3();
//            slide.in();
//            sleep(1000);
//            slide.out();
//            sleep(1000);
//            slide.in();
//            sleep(1000);
//            slide.out();
//            sleep(1000);
//            slide.in();
//            sleep(1000);
//            slide.out();
//            sleep(1000);

            if(x){
                outtakePosition += 20;
                outtakeSlide.set(outtakePosition);
                telemetry.addData("Outtake slide position: ", outtakeSlide.outtakeSlide.getCurrentPosition());
                telemetry.update();
            }
            if(b){
                outtakePosition -= 20;
                outtakeSlide.set(outtakePosition);
                telemetry.addData("Outtake slide position: ", outtakeSlide.outtakeSlide.getCurrentPosition());
                telemetry.update();
            }
            telemetry.addData("Outtake slide position: ", outtakeSlide.outtakeSlide.getCurrentPosition());
            telemetry.update();

    }
}}
