package org.firstinspires.ftc.teamcode.RobotObjects.EPIC;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class SampleAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RedPipe pipeline = new RedPipe(telemetry);
        pipeline.map(hardwareMap);
        pipeline.initialize();

        waitForStart();

        switch (pipeline.getLocation()){
            case LEFT:
                //Auton code if the camera detects the object to the left
                break;

            case MIDDLE:
                //Auton code for middle object
                break;

            case RIGHT:
                //Auton code for right object
                break;
        }
    }

}
