package org.firstinspires.ftc.teamcode.RobotObjects.EPIC.Odyssey;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Launch_Control {
    public LinearOpMode parent;

    private ElapsedTime runtime = new ElapsedTime();

    public Telemetry telemetry;
    public Servo air;

    public Launch_Control(HardwareMap hardwareMap) {
        air = hardwareMap.get(Servo.class, "air");
    }


    public void launch() {
        air.setPosition(0.5);
        parent.sleep(100);
    }
}
