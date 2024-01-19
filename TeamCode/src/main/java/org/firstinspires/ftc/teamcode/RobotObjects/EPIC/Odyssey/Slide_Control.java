package org.firstinspires.ftc.teamcode.RobotObjects.EPIC.Odyssey;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Slide_Control {
    public LinearOpMode parent;

    private ElapsedTime runtime = new ElapsedTime();

    public Telemetry telemetry;

    public DcMotorEx slide;

    public double power = 0.0;

    public int deliverPos = -2000;

    public int pickUpPos = 0;

    public Slide_Control(HardwareMap hardwareMap) {slide = hardwareMap.get(DcMotorEx.class, "slide");}

    public void initialize() {
//        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        power = 0.6;
    }

    public void deliver() {
        slide.setTargetPosition(deliverPos);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(power);
        parent.sleep(200);

        telemetry.addData("slide Positiion:",slide.getCurrentPosition());
        telemetry.update();
    }


    public void pickUp() {
        slide.setTargetPosition(pickUpPos);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(power);
        parent.sleep(200);

        telemetry.addData("slide Positiion:",slide.getCurrentPosition());
        telemetry.update();
    }

    public double getCurrentPosition(){
        return slide.getCurrentPosition();
    }

    public void increment(int value){

        int pos = slide.getCurrentPosition() + value;
        slide.setTargetPosition(pos);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(power);
        parent.sleep(100);

        telemetry.addData("slide Positiion:",slide.getCurrentPosition());
        telemetry.update();
    }
}
