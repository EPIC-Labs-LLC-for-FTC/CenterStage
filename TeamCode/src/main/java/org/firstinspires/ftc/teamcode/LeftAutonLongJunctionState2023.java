package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotObjects.EPIC.Arm2023;
import org.firstinspires.ftc.teamcode.RobotObjects.EPIC.Claw2023;
import org.firstinspires.ftc.teamcode.RobotObjects.EPIC.DistanceSensor2023;
import org.firstinspires.ftc.teamcode.RobotObjects.EPIC.IntakeSlide2023;
import org.firstinspires.ftc.teamcode.RobotObjects.EPIC.Mecanum_Wheels;
import org.firstinspires.ftc.teamcode.RobotObjects.EPIC.Outtake2023;
import org.firstinspires.ftc.teamcode.RobotObjects.EPIC.OuttakeSlide2023;
import org.firstinspires.ftc.teamcode.RobotObjects.EPIC.Turret2023;

@Disabled
@Autonomous(name = "Long Junction Left Auton State 2023",group = "Competition")
public class LeftAutonLongJunctionState2023 extends LinearOpMode {
//    OpenCvCamera webcam;
//    Scanner scanner;
    public Claw2023 finger;
    public Arm2023 arm;
    public Outtake2023 outtake;
    public OuttakeSlide2023 outtakeSlide;
    public Turret2023 turret;
    public IntakeSlide2023 slide;
    public DistanceSensor2023 ds;
//    public DistanceSensor backDistance;
//    public DistanceSensor rightDistance;
    @Override
    public void runOpMode() throws InterruptedException {
//        scanner = new Scanner(hardwareMap);
//        scanner.telemetry = this.telemetry;
//        scanner.parent = this;
//        scanner.initialize();
        Mecanum_Wheels wheels = new Mecanum_Wheels(hardwareMap);
        wheels.parent = this;
        wheels.IsAutonomous = true;
        wheels.telemetry = this.telemetry;
        //wheels.backLeftAdjustment = 0.95;
        //wheels.backRightAdjustment = 0.95;
        wheels.initialize();
int servoSleep = 500;
int motorSleepMs = 2000;
double motorSleepS = 0.5;
//int outtakeTicks = 500; //mid junction
            int outtakeTicks = 1480;
//        backDistance = hardwareMap.get(DistanceSensor.class, "backDistance");
//        rightDistance = hardwareMap.get(DistanceSensor.class, "rightDistance");
        slide = new IntakeSlide2023(hardwareMap);
        slide.parent = this;
        slide.telemetry = telemetry;
        slide.in();
        finger = new Claw2023(hardwareMap);
        finger.parent = this;
        finger.telemetry = telemetry;
        arm = new Arm2023(hardwareMap);
        arm.parent = this;
        arm.telemetry = telemetry;
        arm.doInitialize();
        outtake = new Outtake2023(hardwareMap);
        outtake.parent = this;
        outtake.telemetry = telemetry;
        outtake.recieve();
        outtakeSlide = new OuttakeSlide2023(hardwareMap);
        outtakeSlide.parent = this;
        outtakeSlide.telemetry = this.telemetry;
        turret = new Turret2023(hardwareMap);
        turret.parent = this;
        turret.telemetry = telemetry;
        turret.setPosition(0.80);
        ds = new DistanceSensor2023(hardwareMap);
        ds.parent = this;
        ds.telemetry = telemetry;
        int parkingSpot = 3;
//        parkingSpot = scanner.getParkingSpot();
//        scanner.releaseCamera();
        double leftDist = ds.getLeftDistance();
        double rightDist = ds.getRightDistance();
        double leftBackDistance = ds.getLeftBackDistance();
        double rightBackDistance = ds.getRightBackDistance();
        double poleDist = turret.getPoleDist();
        telemetry.addData("leftDist:",leftDist);
        telemetry.addData("rightDist:",rightDist);
        telemetry.addData("leftBackDistance:",leftBackDistance);
        telemetry.addData("rightBackDistance:",rightBackDistance);
        telemetry.update();
        waitForStart();
//        sleep(2000);
        double distance = 48;
        wheels.leftErrorAdjustment = 0.75;
        double diff = 0;
        //Left Straf
        wheels.encoderDrive(0.6,   -distance, distance, distance, -distance, 2);

//        leftBackDistance = ds.getLeftBackDistance();
//        rightBackDistance = ds.getRightBackDistance();
//        diff = (leftBackDistance - rightBackDistance);
        diff = 4;
//        //correction rotation
//        while((leftBackDistance>32 || rightBackDistance>32) ) {
//            diff = 2*diff;
            wheels.encoderDrive(0.6, -diff, -diff, diff, diff, 1);
//            leftBackDistance = ds.getLeftBackDistance();
//            rightBackDistance = ds.getRightBackDistance();
//            diff = (leftBackDistance - rightBackDistance);
//            if(leftBackDistance<32 && rightBackDistance<32 && diff>1)
//                break;
//        }
//        //sleep(10000);
//        //distance = 6.0;
        leftBackDistance = ds.getLeftBackDistance();
        rightBackDistance = ds.getRightBackDistance();
//
        leftBackDistance = 32.5-leftBackDistance;
        rightBackDistance = 32.5-rightBackDistance;
//        //forward
        wheels.encoderDrive(0.5,   leftBackDistance, leftBackDistance, rightBackDistance, rightBackDistance, 1);
//        //turret.alignPole();
        turret.setPosition(0.73);
        //sleep(servoSleep);

        arm.doTransition();
        sleep(servoSleep);
        outtakeSlide.set(outtakeTicks,motorSleepS);
        //sleep(1500);
        outtake.drop();
        sleep(servoSleep);
        outtake.recieve();
        outtakeSlide.set(0,motorSleepS);
        slide.outAuton();
        sleep(servoSleep);
        arm.doLevel5();
        //sleep(500);
        finger.grab();
        sleep(servoSleep);
        arm.doTransition();
        sleep(servoSleep);
        slide.in();
        sleep(servoSleep);
        arm.doDropping();
        sleep(servoSleep);
        finger.release();
        sleep(servoSleep);
        arm.doTransition();
        outtakeSlide.set(outtakeTicks,motorSleepS);
        //sleep(1000);
        outtake.drop();
        sleep(servoSleep);
        outtake.recieve();
        outtakeSlide.set(0);
        slide.outAuton();
        sleep(servoSleep);
        arm.doLevel4();

        sleep(servoSleep);
        finger.grab();
        sleep(servoSleep);
        arm.doTransition();
        sleep(servoSleep);
        slide.in();
        sleep(servoSleep);
        arm.doDropping();
        sleep(servoSleep);
        finger.release();
        sleep(servoSleep);
        arm.doTransition();
        outtakeSlide.set(outtakeTicks,motorSleepS);
        //sleep(1000);
        outtake.drop();
        sleep(servoSleep);
        outtake.recieve();
        outtakeSlide.set(0,motorSleepS);
        slide.outAuton();
        sleep(servoSleep);
        arm.doLevel3();

        sleep(servoSleep);
        finger.grab();
        sleep(servoSleep);
        arm.doTransition();
        sleep(servoSleep);
        slide.in();
        sleep(servoSleep);
        arm.doDropping();
        sleep(servoSleep);
        finger.release();
        sleep(servoSleep);
        arm.doTransition();
        outtakeSlide.set(outtakeTicks,motorSleepS);
        //sleep(1000);
        outtake.drop();
        sleep(servoSleep);
        outtake.recieve();
        outtakeSlide.set(0,motorSleepS);
        slide.in();
        sleep(servoSleep);
        arm.doDropping();
        //slide.outAuton();
        sleep(servoSleep);
        //arm.doLevel2();
            if(parkingSpot==3)
                    distance = -23;
            wheels.encoderDrive(0.8,distance,distance*0.75,distance,distance*0.75,2);
        while(opModeIsActive()){
            poleDist = turret.getPoleDist();
            leftDist = ds.getLeftDistance();
            rightDist = ds.getRightDistance();
            leftBackDistance = ds.getLeftBackDistance();
            rightBackDistance = ds.getRightBackDistance();
            telemetry.addData("poleDist:",poleDist);
            telemetry.addData("leftDist:",leftDist);
            telemetry.addData("rightDist:",rightDist);
            telemetry.addData("leftBackDistance:",leftBackDistance);
            telemetry.addData("rightBackDistance:",rightBackDistance);
            telemetry.update();
            sleep(2000);

        }
//            wheels.encoderDrive(0.8,   -30, 30, 30, -30, 3);
//
//            turret.left();
//            sleep(500);
//            //preload
//            slide.outtakeSet(20);
//            outtake.drop();
//            sleep(1500);
//            outtake.recieve();
//            slide.outtakeSet(0);
//            sleep(1000);
//            //first cycle
//            arm.level5();
//            slide.intakeSet(35);
//            sleep(1000);
//            finger.grab();
//            sleep(600);
//            arm.dropping();
//            slide.intakeSet(0);
//            sleep(1000);
//            slide.outtakeSet(20);
//            outtake.drop();
//            sleep(1500);
//            outtake.recieve();
//            slide.outtakeSet(0);
//            sleep(1000);
//            //second cycle
//            arm.level4();
//            slide.intakeSet(35);
//            sleep(1000);
//            finger.grab();
//            sleep(600);
//            arm.dropping();
//            slide.intakeSet(0);
//            sleep(1000);
//            slide.outtakeSet(20);
//            outtake.drop();
//            sleep(1500);
//            outtake.recieve();
//            slide.outtakeSet(0);
//            sleep(1000);
//            //third cycle
//            arm.level3();
//            slide.intakeSet(35);
//            sleep(1000);
//            finger.grab();
//            sleep(600);
//            arm.dropping();
//            slide.intakeSet(0);
//            sleep(1000);
//            slide.outtakeSet(20);
//            outtake.drop();
//            sleep(1500);
//            outtake.recieve();
//            slide.outtakeSet(0);
//            sleep(1000);
//            //fourth cycle
//            arm.level2();
//            slide.intakeSet(35);
//            sleep(1000);
//            finger.grab();
//            sleep(600);
//            arm.dropping();
//            slide.intakeSet(0);
//            sleep(1000);
//            slide.outtakeSet(20);
//            outtake.drop();
//            sleep(1500);
//            outtake.recieve();
//            slide.outtakeSet(0);
//            sleep(1000);
//            //last cycle
//            arm.ground();
//            slide.intakeSet(35);
//            sleep(1000);
//            finger.grab();
//            sleep(600);
//            arm.dropping();
//            slide.intakeSet(0);
//            sleep(1000);


        }

       //arm.level5();
        //outtakeSlide.setPower(0.8);
        //outtakeSlide.setTargetPosition(-1300);
        //outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //finger.grab();
    }

