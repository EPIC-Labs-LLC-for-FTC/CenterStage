import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOpTest1")
public class TeleOpTest1 extends LinearOpMode {

    //Define motors
    public DcMotorEx leftFront = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx leftBack = null;
    public DcMotorEx  rightBack = null;
    public DcMotorEx arm = null;
    public DcMotorEx spinTake = null;
    public Servo door = null;
    public Servo wrist = null;
    public int targetPosition;

    double movement;
    double rotation;
    double strafe;

    public void doorContorl() {

        boolean y = gamepad2.y;
        boolean b = gamepad2.b;
        double doorClose = 1;
        double doorOpen = 0.4;

        if (y) {
            door.setPosition(doorClose);
        } else if (b) {
            door.setPosition(doorOpen);
        }

    }

    public void spinnerControl() {

        boolean right_bumper = gamepad1.right_bumper;
        boolean left_bumper = gamepad1.left_bumper;

        if (right_bumper) {
            spinTake.setPower(1);
        } else {
            spinTake.setPower(0);
        }
        if (left_bumper){
            spinTake.setPower(-1);
        }else {
            spinTake.setPower(0);
        }


    }

    public void armControl() {


        if (gamepad2.dpad_left) {
            wrist.setDirection(Servo.Direction.REVERSE);
            wrist.setPosition(0);
        }

        if (gamepad2.dpad_down){
            wrist.setDirection(Servo.Direction.REVERSE);
            wrist.setPosition(1);
        }

        if( gamepad2.right_trigger > 0.5){
            arm.setTargetPosition(-1650);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(-1);
        }

        if( gamepad2.left_trigger > 0.5){
            arm.setTargetPosition(90);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1);
        }

        if (gamepad2.dpad_right) {
            arm.setTargetPosition(-50);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(-1);
        }

        if( gamepad2.dpad_up){
            wrist.setDirection(Servo.Direction.REVERSE);
            wrist.setPosition(0.2);
        }
        telemetry.addData("Target Position", targetPosition);
        telemetry.update();
    }
    public void driverControl() {
        movement = gamepad1.left_stick_y;
        rotation = gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;

        double magnitude = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2));
        double direction = Math.atan2(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        boolean precision = gamepad1.right_bumper;

        //INFO Increasing speed to a maximum of 1
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
        door = hardwareMap.get(Servo.class,"door");
        wrist = hardwareMap.get(Servo.class, "wrist");
        arm = hardwareMap.get(DcMotorEx.class, "arm");

        leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinTake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        targetPosition = arm.getCurrentPosition();

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                driverControl();
                armControl();
                spinnerControl();
                doorContorl();
            }
        }
    }
}
