package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.RobotObjects.EPIC.Odyssey.Arm_Claw_Control;
import org.firstinspires.ftc.teamcode.RobotObjects.EPIC.Odyssey.Slide_Control;

@Autonomous(name="Odyssey_Blue_Left", group="Odyssey")
public class Odyssey_Blue_Left extends LinearOpMode {

    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;
    public IMU imu = null;

    double headingError  = 0;
    double targetHeading = 0;
    double driveSpeed = 0;
    double turnSpeed = 0;

    double leftFrontSpeed = 0;
    double rightFrontSpeed = 0;
    double leftBackSpeed = 0;
    double rightBackSpeed = 0;

    int leftFrontTarget = 0;
    int rightFrontTarget = 0;
    int rightBackTarget = 0;
    int leftBackTarget = 0;

    double COUNTS_PER_MOTOR_REV = 537.7 ;
    double DRIVE_GEAR_REDUCTION = 1.0 ;
    double WHEEL_DIAMETER_INCHES = 3.779528 ;
    double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    double DRIVE_SPEED = 0.5;
    double TURN_SPEED = 0.5;
    double HEADING_THRESHOLD = 1;
    double P_TURN_GAIN = 0.1;
    double P_DRIVE_GAIN = 0.1;

    org.firstinspires.ftc.teamcode.RobotObjects.EPIC.Odyssey.Slide_Control Slide_Control = null;
    org.firstinspires.ftc.teamcode.RobotObjects.EPIC.Odyssey.Arm_Claw_Control Arm_Claw_Control = null;

    @Override
    public void runOpMode() throws InterruptedException {

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        imu = hardwareMap.get(IMU.class, "imu");

        Arm_Claw_Control = new Arm_Claw_Control(hardwareMap);
        Slide_Control = new Slide_Control(hardwareMap);

        Slide_Control.parent = this;
        Slide_Control.telemetry = this.telemetry;
        Slide_Control.initialize();
        Slide_Control.power = 1;

        Arm_Claw_Control.parent = this;
        Arm_Claw_Control.telemetry = this.telemetry;
        Arm_Claw_Control.initialize();

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        Slide_Control.parent = this;
        Slide_Control.telemetry = this.telemetry;
        Slide_Control.initialize();
        Slide_Control.power = 1;

        Arm_Claw_Control.parent = this;
        Arm_Claw_Control.telemetry = this.telemetry;
        Arm_Claw_Control.initialize();

        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();
        }

        leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        imu.resetYaw();

        waitForStart();

        driveStraight(1, 12, 0);
        spline(1,9,-90,5);
        Arm_Claw_Control.armMid();
        sleep(200);
        Slide_Control.deliver();
        sleep(200);
        Arm_Claw_Control.armDeliver();
        driveStraight(1, -41.5, -90);
        Arm_Claw_Control.clawOpen();
        sleep(1500);
        strafe(1,20,-90);
        sleep(200);
        driveStraight(1,4,-90);
        sleep(200);
        Arm_Claw_Control.armPickUp();
        sleep(500);
        Slide_Control.pickUp();

        telemetry.addData("wrist", Arm_Claw_Control.wrist.getPosition());
        telemetry.addData("arm1", Arm_Claw_Control.arm1.getPosition());
        telemetry.addData("arm2", Arm_Claw_Control.arm2.getPosition());
        telemetry.addData("slide", Slide_Control.slide.getCurrentPosition());
        telemetry.update();

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(100);
    }
    public void driveStraight(double maxDriveSpeed, double distance, double heading) {
        if (opModeIsActive()) {
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            leftFrontTarget = leftFront.getCurrentPosition() + moveCounts;
            rightFrontTarget = rightFront.getCurrentPosition() + moveCounts;
            leftBackTarget = leftBack.getCurrentPosition() + moveCounts;
            rightBackTarget = rightBack.getCurrentPosition() + moveCounts;

            leftFront.setTargetPosition(leftFrontTarget);
            rightFront.setTargetPosition(rightFrontTarget);
            leftBack.setTargetPosition(leftBackTarget);
            rightBack.setTargetPosition(rightBackTarget);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            while (opModeIsActive() && (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {

                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                if (distance < 0)
                    turnSpeed *= -1;

                moveRobot(driveSpeed, turnSpeed);

                sendTelemetry(true);
            }

            moveRobot(0, 0);
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void strafe(double maxStrafeSpeed, double distance, double heading) {
        if (opModeIsActive()) {
            int moveCounts = (int) (distance * COUNTS_PER_INCH);

            leftFrontTarget = leftFront.getCurrentPosition() + moveCounts;
            rightFrontTarget = rightFront.getCurrentPosition() - moveCounts;
            leftBackTarget = leftBack.getCurrentPosition() - moveCounts;
            rightBackTarget = rightBack.getCurrentPosition() + moveCounts;

            leftFront.setTargetPosition(leftFrontTarget);
            rightFront.setTargetPosition(rightFrontTarget);
            leftBack.setTargetPosition(leftBackTarget);
            rightBack.setTargetPosition(rightBackTarget);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            maxStrafeSpeed = Math.abs(maxStrafeSpeed);
            moveRobot(0, maxStrafeSpeed);

            while (opModeIsActive() && (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                moveRobot(0, maxStrafeSpeed + turnSpeed);

                sendTelemetry(true);
            }

            moveRobot(0, 0);
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    public void spline(double maxStrafeSpeed, double distance, double heading, double curvature) {
        int moveCounts = (int) (distance * COUNTS_PER_INCH);

        leftFrontTarget = leftFront.getCurrentPosition() - moveCounts;
        rightFrontTarget = rightFront.getCurrentPosition() + moveCounts;
        leftBackTarget = leftBack.getCurrentPosition() + moveCounts;
        rightBackTarget = rightBack.getCurrentPosition() - moveCounts;

        double curveAdjustment = curvature * moveCounts;
        leftFrontTarget += curveAdjustment;
        rightFrontTarget += curveAdjustment;
        leftBackTarget += curveAdjustment;
        rightBackTarget += curveAdjustment;

        leftFront.setTargetPosition(leftFrontTarget);
        rightFront.setTargetPosition(rightFrontTarget);
        leftBack.setTargetPosition(leftBackTarget);
        rightBack.setTargetPosition(rightBackTarget);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        maxStrafeSpeed = Math.abs(maxStrafeSpeed);

        moveRobot(maxStrafeSpeed, 0);

        while (opModeIsActive() && (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {
            turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

            moveRobot(maxStrafeSpeed, turnSpeed);

            sendTelemetry(true);
        }

        moveRobot(0, 0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafespline(double maxStrafeSpeed, double distance, double heading, double curvature) {
        int moveCounts = (int) (distance * COUNTS_PER_INCH);

        leftFrontTarget = leftFront.getCurrentPosition() - moveCounts;
        rightFrontTarget = rightFront.getCurrentPosition() + moveCounts;
        leftBackTarget = leftBack.getCurrentPosition() + moveCounts;
        rightBackTarget = rightBack.getCurrentPosition() - moveCounts;

        double curveAdjustment = curvature * moveCounts;
        leftFrontTarget += curveAdjustment;
        rightFrontTarget += curveAdjustment;
        leftBackTarget += curveAdjustment;
        rightBackTarget += curveAdjustment;

        leftFront.setTargetPosition(leftFrontTarget);
        rightFront.setTargetPosition(rightFrontTarget);
        leftBack.setTargetPosition(leftBackTarget);
        rightBack.setTargetPosition(rightBackTarget);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        maxStrafeSpeed = Math.abs(maxStrafeSpeed);

        moveRobot(maxStrafeSpeed, 0);

        while (opModeIsActive() && (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {
            turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

            moveRobot(maxStrafeSpeed, turnSpeed);

            sendTelemetry(true);
        }

        moveRobot(0, 0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void diagonalStrafe(double maxStrafeSpeed, double distance, double heading, boolean forward) {
        int moveCounts = (int) (distance * COUNTS_PER_INCH);

        if (forward) {
            leftFrontTarget = leftFront.getCurrentPosition() - moveCounts;
            rightBackTarget = rightBack.getCurrentPosition() + moveCounts;
        } else {
            leftBackTarget = leftBack.getCurrentPosition() - moveCounts;
            rightFrontTarget = rightFront.getCurrentPosition() + moveCounts;
        }

        leftFront.setTargetPosition(leftFrontTarget);
        rightFront.setTargetPosition(rightFrontTarget);
        leftBack.setTargetPosition(leftBackTarget);
        rightBack.setTargetPosition(rightBackTarget);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        maxStrafeSpeed = Math.abs(maxStrafeSpeed);

        moveRobot(maxStrafeSpeed, 0);

        while (opModeIsActive() && (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {
            turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

            moveRobot(maxStrafeSpeed, turnSpeed);

            sendTelemetry(true);
        }

        moveRobot(0, 0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void turnToHeading(double maxTurnSpeed, double heading) {

        getSteeringCorrection(heading, P_DRIVE_GAIN);

        while (opModeIsActive() && (Math.toRadians(headingError) > HEADING_THRESHOLD)) {

            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            moveRobot(0, turnSpeed);

            sendTelemetry(false);
        }

        moveRobot(0, 0);
    }

    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            moveRobot(0, turnSpeed);

            sendTelemetry(false);
        }

        moveRobot(0, 0);
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;

        headingError = targetHeading - getHeading();

        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;
        turnSpeed  = turn;

        leftFrontSpeed  = drive - turn;
        rightFrontSpeed = drive + turn;
        rightBackSpeed = drive + turn;
        leftBackSpeed = drive - turn;

        double max = Math.max(Math.max(Math.abs(leftFrontSpeed), Math.abs(rightFrontSpeed)), Math.max(Math.abs(leftBackSpeed), Math.abs(rightBackSpeed)));
        if (max > 1.0)
        {
            leftFrontSpeed /= max;
            rightFrontSpeed /= max;
            leftBackSpeed /= max;
            rightBackSpeed /=max;
        }

        leftFront.setPower(leftFrontSpeed);
        rightFront.setPower(rightFrontSpeed);
        leftBack.setPower(leftBackSpeed);
        rightBack.setPower(rightBackSpeed);
    }

    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R",  "%7d:%7d", leftFrontTarget,  rightFrontTarget, leftBackTarget, rightBackTarget);
            telemetry.addData("Actual Pos L:R",  "%7d:%7d", leftFront.getCurrentPosition(), rightFront.getCurrentPosition(), leftBack.getCurrentPosition(), rightBack.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", leftFrontSpeed, rightFrontSpeed, leftBackSpeed, rightBackSpeed);
        telemetry.update();
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}
