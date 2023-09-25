package org.firstinspires.ftc.teamcode.RobotObjects.EPIC;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Outtake2023 {
    //Configuration used: 6wheelConfig
    public ServoImplEx outtake;
    //public DcMotorEx liftMotor;
    public Telemetry telemetry;
    public LinearOpMode parent;
    public double dropping = 0.81;
    public double recieving = 0;
    public boolean isDropped = false;
    public boolean isRecieved = true;

    public Outtake2023(HardwareMap hardwareMap) {
        outtake = hardwareMap.get(ServoImplEx.class,"outtake");

    }

    public void drop()
    {
        outtake.setPosition(.875);
        telemetry.addData("Finger Claw 1:%d", outtake.getPosition());
        telemetry.update();
        isDropped = true;
        isRecieved = false;
    }

    public void recieve() {
        outtake.setPosition(0);
        telemetry.addData("Finger Claw 1:%d", outtake.getPosition());
        telemetry.update();
        isDropped = false;
        isRecieved = true;
    }
    public void setPosition(double position){
        outtake.setPosition(position);
    }
    public double getPosition(){
        return outtake.getPosition();
    }

}
