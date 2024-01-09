package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotObjects.EPIC.Odyssey.Arm_Claw_Control;
import org.firstinspires.ftc.teamcode.RobotObjects.EPIC.Odyssey.Launch_Control;
import org.firstinspires.ftc.teamcode.RobotObjects.EPIC.Odyssey.Slide_Control;
import org.firstinspires.ftc.teamcode.RobotObjects.EPIC.Odyssey.Spinner;

@TeleOp(name="Odyssey_v2")
public class Odyssey_v2 extends LinearOpMode {

    public DcMotorEx leftFront = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx leftBack = null;
    public DcMotorEx rightBack = null;
    public DcMotorEx spinTake = null;
//    public DcMotorEx hang1 = null;
//    public DcMotorEx hang2 = null;


    Slide_Control Slide_Control = null;
    Arm_Claw_Control Arm_Claw_Control = null;
    Launch_Control Launch_Control = null;

    double movement;
    double rotation;
    double strafe;

    public void fullControl(){

        if (gamepad1.right_bumper) {
            spinTake.setPower(1);
        } else {
            spinTake.setPower(0);
        }
        if (gamepad1.left_bumper){
            spinTake.setPower(-1);
        }else {
            spinTake.setPower(0);
        }

        if (gamepad2.left_bumper){
            Slide_Control.increment(+100);
        }
        if (gamepad2.right_bumper){
            Slide_Control.increment(-100);
        }

        if (gamepad2.b){
            Arm_Claw_Control.arm1Increment(0.05);
            Arm_Claw_Control.arm2Increment(0.05);
        }
        if (gamepad2.a){
            Arm_Claw_Control.arm1Decrement(0.05);
            Arm_Claw_Control.arm2Decrement(0.05);
        }

//        if (gamepad1.a){
//            Arm_Claw_Control.wristIncrement(0.01);
//        }
//        if (gamepad1.b){
//            Arm_Claw_Control.wristDecrement(0.01);
//        }
        

//        if (gamepad2.dpad_left){
//            Arm_Claw_Control.armMid();
//        }
        if (gamepad2.dpad_down){
            Arm_Claw_Control.armPickUp();
        }
        if (gamepad2.dpad_left){
            Arm_Claw_Control.armDeliver();
        }

        if (gamepad2.right_trigger > 0.2){
            Arm_Claw_Control.clawClose();
        }
        if (gamepad2.left_trigger > 0.2){
            Arm_Claw_Control.clawOpen();
        }

        if (gamepad2.x){
            Arm_Claw_Control.wristHorizontal();
        }
        if (gamepad2.y){
            Arm_Claw_Control.wristVertical1();
        }

        if (gamepad1.a){
            Launch_Control.launch();
        }

//        if(gamepad1.a){
//            hang1.setPower(1);
//            hang2.setPower(-1);
//        }else{
//            hang1.setPower(0);
//            hang2.setPower(0);
//        }
//        if(gamepad1.b){
//            hang1.setPower(-1);
//            hang2.setPower(1);
//        }else{
//            hang1.setPower(0);
//            hang2.setPower(0);
//        }


        telemetry.addData("wrist", Arm_Claw_Control.wrist.getPosition());
        telemetry.addData("arm1", Arm_Claw_Control.arm1.getPosition());
        telemetry.addData("arm2", Arm_Claw_Control.arm2.getPosition());
        telemetry.addData("slide", Slide_Control.slide.getCurrentPosition());
//        telemetry.addData("hang1", hang1.getCurrentPosition());
//        telemetry.addData("hang2", hang2.getCurrentPosition());
        telemetry.update();

    }

    public void driverControl() {
        movement = gamepad1.left_stick_y;
        rotation = gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;

        double magnitude = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2));
        double direction = Math.atan2(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        boolean precision = gamepad1.right_bumper;

        double lf = magnitude * Math.sin(direction + Math.PI / 4) + rotation;
        double lb = magnitude * Math.cos(direction + Math.PI / 4) + rotation;
        double rf = magnitude * Math.cos(direction + Math.PI / 4) - rotation;
        double rb = magnitude * Math.sin(direction + Math.PI / 4) - rotation;

        double hypot = Math.hypot(movement, strafe);
        double ratio;
        if (movement == 0 && strafe == 0)
            ratio = 1;
        else if (precision)
            ratio = hypot / (Math.max(Math.max(Math.max(Math.abs(lf), Math.abs(lb)), Math.abs(rb)), Math.abs(rf))) / 2;
        else
            ratio = hypot / (Math.max(Math.max(Math.max(Math.abs(lf), Math.abs(lb)), Math.abs(rb)), Math.abs(rf)));

        leftFront.setPower(ratio * lf);
        leftBack.setPower(ratio * lb);
        rightFront.setPower(ratio * rf);
        rightBack.setPower(ratio * rb);
    }

    @Override
    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        spinTake = hardwareMap.get(DcMotorEx.class, "spinTake");
//        hang1 = hardwareMap.get(DcMotorEx.class, "hang1");
//        hang2 = hardwareMap.get(DcMotorEx.class, "hang2");

        Arm_Claw_Control = new Arm_Claw_Control(hardwareMap);
        Slide_Control = new Slide_Control(hardwareMap);
        Launch_Control = new Launch_Control(hardwareMap);

        leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        hang1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        hang2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinTake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        hang1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        hang2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        hang2.setDirection(DcMotorSimple.Direction.REVERSE);

        Slide_Control.parent = this;
        Slide_Control.telemetry = this.telemetry;
        Slide_Control.initialize();
        Slide_Control.power = 1;

        Arm_Claw_Control.parent = this;
        Arm_Claw_Control.telemetry = this.telemetry;
        Arm_Claw_Control.initialize();

        Launch_Control.parent = this;
        Launch_Control.telemetry = this.telemetry;

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                driverControl();
                fullControl();
            }
        }
    }
}
