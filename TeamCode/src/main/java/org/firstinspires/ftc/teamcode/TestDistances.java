package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotObjects.EPIC.DistanceSensor2023;

@Disabled
@TeleOp(name = "Test Distances",group = "test")
public class TestDistances extends LinearOpMode{

    public DistanceSensor2023 distances;

    @Override
    public void runOpMode() throws InterruptedException {
        distances = new DistanceSensor2023(hardwareMap);
        distances.telemetry = telemetry;
        distances.parent = this;
        double leftDist = distances.getLeftDistance();
        double rightDist = distances.getRightDistance();
        double leftBackDistance = distances.getLeftBackDistance();
        double rightBackDistance = distances.getRightBackDistance();
        telemetry.addData("leftDist:",leftDist);
        telemetry.addData("rightDist:",rightDist);
        telemetry.addData("leftBackDistance:",leftBackDistance);
        telemetry.addData("rightBackDistance:",rightBackDistance);
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){

            leftDist = distances.getLeftDistance();
            rightDist = distances.getRightDistance();
            leftBackDistance = distances.getLeftBackDistance();
            rightBackDistance = distances.getRightBackDistance();
            telemetry.addData("leftDist:",leftDist);
            telemetry.addData("rightDist:",rightDist);
            telemetry.addData("leftBackDistance:",leftBackDistance);
            telemetry.addData("rightBackDistance:",rightBackDistance);
            telemetry.update();
            sleep(2000);

    }
}}
