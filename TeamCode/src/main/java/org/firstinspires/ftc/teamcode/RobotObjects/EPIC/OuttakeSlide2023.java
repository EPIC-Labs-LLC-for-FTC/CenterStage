package org.firstinspires.ftc.teamcode.RobotObjects.EPIC;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OuttakeSlide2023 {
    public static DcMotorEx outtakeSlide;
    public Telemetry telemetry;
    public LinearOpMode parent;
    private int min = 0;
    private int max = 2800;


    private ElapsedTime runtime = new ElapsedTime();


    public OuttakeSlide2023(HardwareMap hardwareMap){
        outtakeSlide = hardwareMap.get(DcMotorEx.class,"outtakeSlide");
        outtakeSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        outtakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void set(int position){
        if(position<=min){

        }
        else if(position>=max){
            outtakeSlide.setTargetPosition(max);
            outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            outtakeSlide.setPower(0.9);
        }
        else{
            outtakeSlide.setTargetPosition(position);
            outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            outtakeSlide.setPower(0.9);
        }


    }
    public void set(int position,double timeoutS){
        outtakeSlide.setTargetPosition(position);
        outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        outtakeSlide.setPower(1);
        while (parent.opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (outtakeSlide.isBusy())) {

        }
        //outtakeSlide.setPower(0);
    }
    public int getPosition(){
        return outtakeSlide.getCurrentPosition();
    }
    public int getTargetPosition(){
        return outtakeSlide.getCurrentPosition();
    }
}
