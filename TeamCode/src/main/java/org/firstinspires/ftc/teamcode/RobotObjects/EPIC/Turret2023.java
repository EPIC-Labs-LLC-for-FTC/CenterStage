package org.firstinspires.ftc.teamcode.RobotObjects.EPIC;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Turret2023 {
    //Configuration used: 6wheelConfig
    public ServoImplEx turret;
    //public DcMotorEx liftMotor;
    public Telemetry telemetry;
    public LinearOpMode parent;
    public double front = 0.4;
    public double left  = 0;
    public double right  = 1;
    public double turretPosition = 0;
    public boolean incrementer = false;
    public double autonLeft = 0.22;
    public Rev2mDistanceSensor poleDist;


    public Turret2023(HardwareMap hardwareMap) {
        turret = hardwareMap.get(ServoImplEx.class,"turret");
        poleDist = hardwareMap.get(Rev2mDistanceSensor.class,"rightDist");
        turret.setDirection(Servo.Direction.FORWARD);

    }
    public void setPosition(double position){
        turret.setPosition(position);
    }
    public void home()
    {
        turret.setPosition(front);
        telemetry.addData("Finger Claw 1:%d", turret.getPosition());

        telemetry.update();
    }
    public void leftAuton()
    {
        turret.setPosition(autonLeft);
        telemetry.addData("Finger Claw 1:%d", turret.getPosition());

        telemetry.update();
    }
    public void right() {
        turret.setPosition(right);
        telemetry.addData("Finger Claw 1:%d", turret.getPosition());
        telemetry.update();
        //parent.sleep(5000);
    }
    public void left() {
        turret.setPosition(left);
        telemetry.addData("Finger Claw 1:%d", turret.getPosition());
        telemetry.update();
        //parent.sleep(5000);
    }
    public void turretIncrease(){
        if(incrementer) {
            turretPosition = turret.getPosition() + 0.1;
            turret.setPosition(turretPosition);
            parent.sleep(1000);
            incrementer=false;
        }
    }
    public void turretDecrease(){
        if(incrementer) {
            turretPosition = turret.getPosition() - 0.1;
            turret.setPosition(turretPosition);
            parent.sleep(1000);
            incrementer=false;
        }
    }
    public void alignToPoleLeft(){
        double pdist = poleDist.getDistance(DistanceUnit.INCH);
        double odist = pdist;
        while(pdist>odist)
        //if(pdist<odist);
        {
            if(turret.getPosition()-0.075<0){
                break;
            }
            turret.setPosition(turret.getPosition()-0.075);
            turret.getPosition();
            pdist = poleDist.getDistance(DistanceUnit.INCH);
            if(pdist<odist)
                break;
            pdist = odist;
        }
        telemetry.addData("turret pos:", turret.getPosition());
        telemetry.addData("pole dist:", poleDist.getDistance(DistanceUnit.INCH));
        telemetry.update();

    }

    public double getPoleDist(){
        return poleDist.getDistance(DistanceUnit.INCH);
    }

    public double getTurretPosition(){
        return turret.getPosition();
    }
    public void alignPole(){
        alignToPoleRight();
        //alignToPoleLeft();
    }
    public void alignToPoleRight(){
        double pdist = poleDist.getDistance(DistanceUnit.INCH);
        double odist = pdist;
        while(pdist<odist)
        //if(pdist<odist);
        {
            if(turret.getPosition()+0.075>1){
                break;
            }
            turret.setPosition(turret.getPosition()-0.1);
            parent.sleep(500);
            //turret.getPosition();
            pdist = poleDist.getDistance(DistanceUnit.INCH);
            if(pdist>odist)
                break;
            pdist = odist;
            telemetry.addData("turret pos:", turret.getPosition());
            telemetry.addData("pole dist:", poleDist.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }

    }
}
