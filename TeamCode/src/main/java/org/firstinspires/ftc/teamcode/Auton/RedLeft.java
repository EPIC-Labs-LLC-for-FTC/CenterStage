package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotObjects.EPIC.SanjeevMECANUM;

@Autonomous(name = "RedLeft")
public class RedLeft extends LinearOpMode {

    SanjeevMECANUM drive = new SanjeevMECANUM();

    @Override
    public void runOpMode() throws InterruptedException {
        drive.mecanum(hardwareMap);
        drive.initialize();

        waitForStart();
        drive.encoderMovementY(4, 0.5);
        sleep(10000);
        drive.encoderMovementY(40,0.4);
        sleep(10000);
    }
}
