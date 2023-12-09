package org.firstinspires.ftc.teamcode.RobotObjects.EPIC.Odyssey;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {
    public LinearOpMode parent;

    private ElapsedTime runtime = new ElapsedTime();

    public Telemetry telemetry;

    public DcMotorEx arm;

    public double power = 0.0;

    public int homePos = 69;

    public int deliverPos = 1225;

    public int pickUpPos = 269;

    public Arm(HardwareMap hardwareMap) {
        arm = hardwareMap.get(DcMotorEx.class, "arm");
    }

    public void initialize() {
        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        power = 0.6;
    }

    public void deliver() {
        arm.setTargetPosition(deliverPos);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(power);
        parent.sleep(500);

        telemetry.addData("arm Positiion:",arm.getCurrentPosition());
        telemetry.update();
    }

    public void gotoHome() {
        arm.setTargetPosition(homePos);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(power);
        parent.sleep(500);

        telemetry.addData("arm Positiion:",arm.getCurrentPosition());
        telemetry.update();
    }

    public void pickUp() {
        arm.setTargetPosition(pickUpPos);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(power);
        parent.sleep(500);

        telemetry.addData("arm Positiion:",arm.getCurrentPosition());
        telemetry.update();
    }

    public double getCurrentPosition(){
        return arm.getCurrentPosition();
    }



    public void increment(int value){

        int pos = arm.getCurrentPosition() + value;
        arm.setTargetPosition(pos);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(power);
        parent.sleep(100);

        telemetry.addData("arm Positiion:",arm.getCurrentPosition());
        telemetry.update();
    }
}
