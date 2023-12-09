package org.firstinspires.ftc.teamcode.RobotObjects.EPIC.Odyssey;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Spinner {

    public LinearOpMode parent;

    private ElapsedTime runtime = new ElapsedTime();

    public Telemetry telemetry;

    public DcMotorEx spinTake;

    public double power = 0.0;

    public Spinner(HardwareMap hardwareMap)
    {
        spinTake = hardwareMap.get(DcMotorEx.class, "spinTake");
    }

    public void initialize(){
        power = 0.6;
    }

    public void forward(){
        spinTake.setPower(power);
    }

    public void backward() {
        spinTake.setPower(-power);
    }

    public void stop() {
        spinTake.setPower(0);
    }
}
