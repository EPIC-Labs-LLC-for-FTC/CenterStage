package org.firstinspires.ftc.teamcode.RobotObjects.EPIC.Odyssey;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm_Claw_Control {
    public LinearOpMode parent;

    private ElapsedTime runtime = new ElapsedTime();

    public Telemetry telemetry;

    public Servo wrist;
    public Servo arm1;
    public Servo arm2;
    public Servo claw1;
    public Servo claw2;

    public Arm_Claw_Control(HardwareMap hardwareMap){
        wrist = hardwareMap.get(Servo.class, "wrist");
        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 = hardwareMap.get(Servo.class, "arm2");
        claw1 = hardwareMap.get(Servo.class, "claw1");
        claw2 = hardwareMap.get(Servo.class, "claw2");
    }

    public void initialize(){

        wrist.setPosition(0.06);
        arm1.setPosition(1);
        arm2.setPosition(0);
        claw1.setPosition(0);
        claw2.setPosition(1);

    }

    public void setDirection(Servo.Direction direction) {

        wrist.setDirection(direction);
        arm1.setDirection(direction);
        arm2.setDirection(direction);
        claw1.setDirection(direction);
        claw2.setDirection(direction);

    }

    public void wristIncrement(double value){

        double pos = wrist.getPosition() + value;
        if(pos>=0 && pos<=1)
            wrist.setPosition(pos);
        parent.sleep(100);

        telemetry.addData("Wrist Positiion:",wrist.getPosition());
        telemetry.update();
    }

    public void arm1Increment(double value){

        double pos = arm1.getPosition() + value;
        if(pos>=0 && pos<=1)
            arm1.setPosition(pos);
        parent.sleep(100);

        telemetry.addData("arm1 Positiion:",arm1.getPosition());
        telemetry.update();
    }

    public void arm2Increment(double value){

        double pos = arm2.getPosition() - value;
        if(pos>=0 && pos<=1)
            arm2.setPosition(pos);
        parent.sleep(100);

        telemetry.addData("arm2 Positiion:",arm2.getPosition());
        telemetry.update();
    }

    public void wristDecrement(double value){

        double pos = wrist.getPosition() - value;
        if(pos>=0 && pos<=1)
            wrist.setPosition(pos);
        parent.sleep(100);

        telemetry.addData("Wrist Positiion:",wrist.getPosition());
        telemetry.update();
    }

    public void arm1Decrement(double value){

        double pos = arm1.getPosition() - value;
        if(pos>=0 && pos<=1)
            arm1.setPosition(pos);
        parent.sleep(100);

        telemetry.addData("arm1 Positiion:",arm1.getPosition());
        telemetry.update();
    }

    public void arm2Decrement(double value){

        double pos = arm2.getPosition() + value;
        if(pos>=0 && pos<=1)
            arm2.setPosition(pos);
        parent.sleep(100);

        telemetry.addData("arm2 Positiion:",arm2.getPosition());
        telemetry.update();
    }

    public void wristHorizontal(){
        wrist.setPosition(0.06);
        parent.sleep(100);

    }

    public void wristVertical() {
        wrist.setPosition(0.28);
        parent.sleep(100);

    }

    public void armMid() {
        arm1.setPosition(0.94);
        arm2.setPosition(0.06);
        parent.sleep(200);
    }

    public void armPickUp() {
        arm1.setPosition(0.99);
        arm2.setPosition(0.01);
        parent.sleep(100);
    }

    public void armDeliver() {
        arm1.setPosition(0);
        arm2.setPosition(1);
        parent.sleep(100);
    }

    public void clawClose() {
        claw1.setPosition(0);
        claw2.setPosition(1);
        parent.sleep(100);
    }

    public void clawOpen() {
        claw1.setPosition(1);
        claw2.setPosition(0);
        parent.sleep(100);
    }
}
