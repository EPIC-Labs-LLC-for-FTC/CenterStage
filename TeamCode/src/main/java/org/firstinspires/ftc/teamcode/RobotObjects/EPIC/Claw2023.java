package org.firstinspires.ftc.teamcode.RobotObjects.EPIC;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Claw2023 {
    //Configuration used: 6wheelConfig
    public ServoImplEx finger;
    //public DcMotorEx liftMotor;
    public Telemetry telemetry;
    public LinearOpMode parent;
    public double fingerMin = 0.6;
    public double fingerMax = 0.4;
    public boolean isGrabbed = false;
    public boolean isReleased = false;
    public int servoTimeMS = 750;

    public Claw2023(HardwareMap hardwareMap) {
        finger = hardwareMap.get(ServoImplEx.class,"finger");
        //arm = hardwareMap.get(DcMotorEx.class,"arm");
        //arm.setDirection(DcMotor.Direction.REVERSE);

        //liftMotor = hardwareMap.get(DcMotorEx.class,"Lift");

    }

    public void initiateClaw() {

    }

    public void grab()
    {
//        Timer t = new Timer();
//        TimerTask tt = new TimerTask() {
//            @Override
//            public void run() {
                finger.setPosition(fingerMin);
                //parent.sleep(1000);
                telemetry.addData("Finger Claw 1:%d", finger.getPosition());
                telemetry.update();
                isReleased = false;
                isGrabbed = true;

//            }
//        };
//        t.schedule(tt,servoTimeMS);

    }

    public void release() {
        //Timer t = new Timer();
        //TimerTask tt = new TimerTask() {
          //  @Override
            //public void run() {
                finger.setPosition(fingerMax);
                //parent.sleep(1000);
                telemetry.addData("Finger Claw 1:%d", finger.getPosition());
                telemetry.update();
                isReleased = true;
                isGrabbed = false;

           // }
        //};
        //t.schedule(tt,servoTimeMS);
    }
}
