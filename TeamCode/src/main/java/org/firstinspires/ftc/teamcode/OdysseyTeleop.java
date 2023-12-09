package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotObjects.EPIC.Odyssey.Arm;
import org.firstinspires.ftc.teamcode.RobotObjects.EPIC.Odyssey.Door;
import org.firstinspires.ftc.teamcode.RobotObjects.EPIC.Odyssey.Mecanum_Wheels;
import org.firstinspires.ftc.teamcode.RobotObjects.EPIC.Odyssey.Spinner;
import org.firstinspires.ftc.teamcode.RobotObjects.EPIC.Odyssey.Wrist;

@TeleOp(name="Odyssey Tele Op")
public class OdysseyTeleop extends LinearOpMode {

    //Component Object definition
    Mecanum_Wheels wheels = null;//new Mecanum_Wheels(hardwareMap);
    Spinner spinner =  null;//new Spinner(hardwareMap);
    Arm arm =  null;//new Arm(hardwareMap);
    Wrist wrist =  null;//new Wrist(hardwareMap);
    Door door =  null;//new Door(hardwareMap);

   //initialize gamepad1 variables
    double lefty1 = 0.0;
    //double lefty2 = 0.0;
    double leftx1 = 0.0;
    double righty1 = 0.0;
    double rightx1 = 0.0;

    boolean x1 = false;
    boolean y1 = false;
    boolean a1 = false;
    boolean b1 = false;

    boolean rb1 = false;
    boolean lb1 = false;

    boolean dl1 = false;
    boolean du1 = false;
    boolean dr1 = false;
    boolean dd1 = false;

    float rt1 = 0;
    float lt1 = 0;

    //initialize gamepad2 variables
    double lefty2 = 0.0;
    //double lefty2 = 0.0;
    double leftx2 = 0.0;
    double righty2 = 0.0;
    double rightx2 = 0.0;

    boolean x2 = false;
    boolean y2 = false;
    boolean a2 = false;
    boolean b2 = false;

    boolean rb2 = false;
    boolean lb2 = false;

    boolean dl2 = false;
    boolean du2 = false;
    boolean dr2 = false;
    boolean dd2 = false;

    float rt2 = 0;
    float lt2 = 0;



    @Override
    public void runOpMode() {

        wheels = new Mecanum_Wheels(hardwareMap);
        spinner = new Spinner(hardwareMap);
        arm = new Arm(hardwareMap);
        wrist = new Wrist(hardwareMap);
        door = new Door(hardwareMap);

        //Initialize Mecannum wheels
        wheels.IsAutonomous = false;
        wheels.parent = this;
        wheels.telemetry = this.telemetry;
        wheels.initialize();

        //Initialize Spinner
        spinner.parent = this;
        spinner.telemetry = this.telemetry;
        spinner.initialize();
        spinner.power = 0.6; //value should be updated after initialize

        //Initialize arm
        arm.parent = this;
        arm.telemetry = this.telemetry;
        arm.initialize();
        arm.power = 0.6; //value should be updated after initialize

        //Initilaize wrist
        wrist.parent = this;
        wrist.telemetry = this.telemetry;
        wrist.initialize();
        //wrist.setDirection(Servo.Direction.REVERSE);

        //Initilaize door
        door.parent = this;
        door.telemetry = this.telemetry;
        door.initialize();




        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                readGamePad1();
                readGamePad2();

                if(rb1){
                    spinner.forward();
                }
                else if(lb1) {
                    spinner.backward();
                }
                spinner.stop();
                if(dl2){
                    arm.gotoHome();
                    wrist.gotoHome();
                }
                else if(du2){
                    arm.deliver();
                    wrist.deliver();
                }
                else if(dd2){
                    arm.pickUp();
                }

                if(du1){
                    wrist.decrement(0.05);
                }
                else if(dd1){
                    wrist.increment(0.025);
                }

                if(a1){
                    arm.increment(100);
                }
                else if(b1) {
                    arm.increment(-100);
                }

                if(x2){
                    door.close();
                }
                else if(y2) {
                    door.open();
                }




//                driverControl();
//                armControl();
//                spinnerControl();
//                doorContorl();


                wheels.move(lefty1,righty1,leftx1,rightx1);

                telemetry.addData("lefty", "%.2f", lefty1);
                //telemetry.addData("lefty2", "%.2f", lefty2);
                telemetry.addData("leftx", "%.2f", leftx1);
                telemetry.addData("righty", "%.2f", righty1);
                //telemetry.addData("lefty2", "%.2f", lefty2);
                telemetry.addData("rightx", "%.2f", rightx1);
                telemetry.addData("wrist", "%.2f", wrist.wrist.getPosition());

                telemetry.addData("arm", "%.2f", arm.getCurrentPosition());
                telemetry.update();
            }
        }
    }

    private void readGamePad1(){

        lefty1 = gamepad1.left_stick_y;
        leftx1 = gamepad1.left_stick_x;
        righty1 = gamepad1.right_stick_y;
        rightx1 = gamepad1.right_stick_x;

        x1 = gamepad1.x;
        y1 = gamepad1.y;
        a1 = gamepad1.a;
        b1 = gamepad1.b;

        rb1 = gamepad1.right_bumper;
        lb1 = gamepad1.left_bumper;

        dl1 = gamepad1.dpad_left;
        du1 = gamepad1.dpad_up;
        dr1 = gamepad1.dpad_right;
        dd1 = gamepad1.dpad_down;

        rt1 = gamepad1.right_trigger;
        lt1 = gamepad1.left_trigger;
    }

    private void readGamePad2(){
        lefty2 = gamepad2.left_stick_y;
        leftx2 = gamepad2.left_stick_x;
        righty2 = gamepad2.right_stick_y;
        rightx2 = gamepad2.right_stick_x;

        x2 = gamepad2.x;
        y2 = gamepad2.y;
        a2 = gamepad2.a;
        b2 = gamepad2.b;

        rb2 = gamepad2.right_bumper;
        lb2 = gamepad2.left_bumper;

        dl2 = gamepad2.dpad_left;
        du2 = gamepad2.dpad_up;
        dr2 = gamepad2.dpad_right;
        dd2 = gamepad2.dpad_down;

        rt2 = gamepad2.right_trigger;
        lt2 = gamepad2.left_trigger;
    }
}
