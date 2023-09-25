package org.firstinspires.ftc.teamcode.RobotObjects.EPIC;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Slide2023 {
    public ServoImplEx intakeSlide1;
    public ServoImplEx intakeSlide2;
    public DcMotorEx outtakeSlide;
    public Telemetry telemetry;
    public LinearOpMode parent;
    private double extended = 1;
    private double home = 0.2;


    public Slide2023(HardwareMap hardwareMap){
        intakeSlide1 = hardwareMap.get(ServoImplEx.class,"intakeSlide1");
        intakeSlide2 = hardwareMap.get(ServoImplEx.class,"intakeSlide2");
        outtakeSlide = hardwareMap.get(DcMotorEx.class,"outtakeSlide");
        outtakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void outtakeSet(int position){
        outtakeSlide.setTargetPosition(position);
        outtakeSlide.setPower(0.8);
        outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void extend(){
        intakeSlide1.setPosition(extended);
        intakeSlide2.setPosition(extended);
    }
    public void home(){
        intakeSlide1.setPosition(home);
        intakeSlide2.setPosition(home);
    }
}
