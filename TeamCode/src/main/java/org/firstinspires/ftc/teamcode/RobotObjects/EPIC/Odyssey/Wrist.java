package org.firstinspires.ftc.teamcode.RobotObjects.EPIC.Odyssey;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Wrist {
    public LinearOpMode parent;

    private ElapsedTime runtime = new ElapsedTime();

    public Telemetry telemetry;

    public Servo wrist = null;

    public double homePos = 1;

    public double deliverPos = 0.25;

    public Wrist(HardwareMap hardwareMap){
        wrist = hardwareMap.get(Servo.class, "wrist");
    }

    public void initialize(){

    }

    public void setDirection(Servo.Direction direction) {
        wrist.setDirection(direction);
    }

    public void increment(double value){
//        if(wrist.getDirection()== Servo.Direction.REVERSE){
//            value = -value;
//        }
        double pos = wrist.getPosition() + value;
        if(pos>=0 && pos<=1)
            wrist.setPosition(pos);
        parent.sleep(100);

        telemetry.addData("Wrist Positiion:",wrist.getPosition());
        telemetry.update();
    }

    public void decrement(double value){
//        if(wrist.getDirection()== Servo.Direction.REVERSE){
//            value = -value;
//        }

        double pos = wrist.getPosition() - value;
        if(pos>=0 && pos<=1)
            wrist.setPosition(pos);
        parent.sleep(100);

        telemetry.addData("Wrist Positiion:",wrist.getPosition());
        telemetry.update();
    }

    public void gotoHome(){
        wrist.setPosition(0);
        parent.sleep(250);

    }

    public void pickUp() {
        wrist.setPosition(0.2);
        parent.sleep(250);

    }

    public void deliver() {
        wrist.setPosition(1);
        parent.sleep(250);
    }
}
