package org.firstinspires.ftc.teamcode.RobotObjects.EPIC;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensor2023 {
    public Telemetry telemetry;
    public LinearOpMode parent;
    private int averager = 4;
    private Rev2mDistanceSensor rightDist;
    private Rev2mDistanceSensor leftDist;
    private Rev2mDistanceSensor rBackDist;
    private Rev2mDistanceSensor lBackDist;

    public DistanceSensor2023(HardwareMap hardwareMap) {
        rightDist = hardwareMap.get(Rev2mDistanceSensor.class,"rightDist");
        leftDist = hardwareMap.get(Rev2mDistanceSensor.class,"leftDist");
        rBackDist = hardwareMap.get(Rev2mDistanceSensor.class,"rBackDist");
        lBackDist = hardwareMap.get(Rev2mDistanceSensor.class,"lBackDist");
    }
    public double getLeftDistance(){
        double dist = 0;
        for(int i=0;i<averager;i++){
            dist+=leftDist.getDistance(DistanceUnit.INCH);
        }
        return dist/averager;
    }
    public double getRightDistance(){
        double dist = 0;
        for(int i=0;i<averager;i++){
            dist+=rightDist.getDistance(DistanceUnit.INCH);
        }
        return dist/averager;
    }
    public double getRightBackDistance(){
        double dist = 0;
        for(int i=0;i<averager;i++){
            dist+=rBackDist.getDistance(DistanceUnit.INCH);
        }
        return dist/averager;
    }
    public double getLeftBackDistance(){
        double dist = 0;
        for(int i=0;i<averager;i++){
            dist+=lBackDist.getDistance(DistanceUnit.INCH);
        }
        return dist/averager;
    }
}
