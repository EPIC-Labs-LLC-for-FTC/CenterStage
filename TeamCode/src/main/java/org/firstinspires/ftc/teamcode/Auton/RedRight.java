package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotObjects.EPIC.SanjeevMECANUM;

@Autonomous(name = "RedRight")
public class RedRight extends LinearOpMode {

    SanjeevMECANUM drive = new SanjeevMECANUM();

    @Override
    public void runOpMode() throws InterruptedException {
        drive.mecanum(hardwareMap);
        drive.initialize();

        waitForStart();
        drive.encoderMovementX(-48, 0.5);
        sleep(10000);
        drive.encoderMovementY(60,0.4);
        sleep(10000);
    }
}
