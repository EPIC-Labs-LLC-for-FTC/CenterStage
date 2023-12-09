package org.firstinspires.ftc.teamcode.RobotObjects.EPIC.Odyssey;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Door {
    public LinearOpMode parent;

    private ElapsedTime runtime = new ElapsedTime();

    public Telemetry telemetry;

    public double openValue = 1.0;

    public double closeValue = 0.4;

    public Servo door;
    public Door(HardwareMap hardwareMap){
        door = hardwareMap.get(Servo.class,"door");
    }

    public void initialize(){

    }

    public void open(){
        door.setPosition(openValue);
    }

    public void close() {
        door.setPosition(closeValue);
    }
}
