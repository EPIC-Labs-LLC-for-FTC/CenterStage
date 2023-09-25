package org.firstinspires.ftc.teamcode.RobotObjects.EPIC;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Timer;
import java.util.TimerTask;

public class Arm2023 {
    //Configuration used: 6wheelConfig
    public ServoImplEx arm;
    //public DcMotorEx liftMotor;
    public Telemetry telemetry;
    public LinearOpMode parent;
    public double dropping = 0.2;
    public double transition = 0.5;
    public double initialize = 0.4;
    //0.75
    public double level5 = 0.72;
    //0.79
    public double level4 = 0.74;
    //0.82
    public double level3 = 0.79;
    //0.87
    public double level2 = 0.82;
    //0.88
    public double ground = 0.84;
    public int servoTimeMS = 500;

    public Arm2023(HardwareMap hardwareMap) {
        arm = hardwareMap.get(ServoImplEx.class,"arm");
        //arm.isPwmEnabled()
    }

    public void doDropping()
    {
        Timer t = new Timer();
        TimerTask tt = new TimerTask() {
            @Override
            public void run() {
        arm.setPosition(dropping);
        telemetry.addData("arm Claw 1:%d", arm.getPosition());
        telemetry.update();
            }
        };
        t.schedule(tt,servoTimeMS);
    }
    public void doLevel5()
    {
        Timer t = new Timer();
        TimerTask tt = new TimerTask() {
            @Override
            public void run() {
        arm.setPosition(level5);
        telemetry.addData("arm Claw 1:%d", arm.getPosition());
        telemetry.update();
            }
        };
        t.schedule(tt,servoTimeMS);
    }
    public void doLevel4()
    {
        Timer t = new Timer();
        TimerTask tt = new TimerTask() {
            @Override
            public void run() {
        arm.setPosition(level4);
        telemetry.addData("arm Claw 1:%d", arm.getPosition());
        telemetry.update();
            }
        };
        t.schedule(tt,servoTimeMS);
    }
    public void doLevel3()
    {
        Timer t = new Timer();
        TimerTask tt = new TimerTask() {
            @Override
            public void run() {
        arm.setPosition(level3);
        telemetry.addData("arm Claw 1:%d", arm.getPosition());
        telemetry.update();
            }
        };
        t.schedule(tt,servoTimeMS);
    }
    public void doLevel2()
    {
        Timer t = new Timer();
        TimerTask tt = new TimerTask() {
            @Override
            public void run() {
        arm.setPosition(level2);
        telemetry.addData("arm Claw 1:%d", arm.getPosition());
        telemetry.update();
            }
        };
        t.schedule(tt,servoTimeMS);
    }
    public void doTransition(){
        Timer t = new Timer();
        TimerTask tt = new TimerTask() {
            @Override
            public void run() {
        arm.setPosition(transition);
        telemetry.addData("arm 1:%d", arm.getPosition());
        telemetry.update();
            }
        };
        t.schedule(tt,servoTimeMS);

    }
    public void doInitialize()
    {
        Timer t = new Timer();
        TimerTask tt = new TimerTask() {
            @Override
            public void run() {
        arm.setPosition(initialize);
        telemetry.addData("arm  1:%d", arm.getPosition());
        telemetry.update();
            }
        };
        t.schedule(tt,servoTimeMS);
    }
    public void doGround()
    {
        Timer t = new Timer();
        TimerTask tt = new TimerTask() {
            @Override
            public void run() {
        arm.setPosition(ground);
        telemetry.addData("arm 1:%d", arm.getPosition());
        telemetry.update();
            }
        };
        t.schedule(tt,servoTimeMS);
    }
    public double getPosition(){
        return arm.getPosition();
    }
    public void armset(double position ){
        arm.setPosition(arm.getPosition()+position);
    }
}
