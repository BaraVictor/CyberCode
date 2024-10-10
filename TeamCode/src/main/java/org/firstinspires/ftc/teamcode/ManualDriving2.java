


package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.sun.tools.javac.tree.DCTree;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Manualdriving2", group="A")
public class ManualDriving2 extends LinearOpMode {

    ElapsedTime timer = new ElapsedTime();
    boolean isButtonHeld = false;
    boolean isButtonHeld1 = false;
    double schimbator = 0.75;

    int Nivel = 4;

    private DcMotor LeftArmMotor;
    private DcMotor RightArmMotor;
    DcMotor LeftMotor;
    DcMotor RightMotor;

    DcMotor encoderLeft;
    DcMotor encoderRight;
    private Servo demoServo;

    private Servo demoServoRight;
    private Servo demoServoLeft;
    private Servo planeServo;
    private static final double MAX_POSITION_SERVO_WRITE = 0.4;
    private static final double MIN_POSITION_SERVO_WRITE = 0;
    private static final double MAX_POSITION_SERVO_LEFT = 0;
    private static final double MIN_POSITION_SERVO_LEFT = 0.4;

    private static final double MAX_POSITION_SERVO = 0.4;
    private static final double MIN_POSITION_SERVO = 0.0;

    double open1 = 0.65;
    double closed1 = 0.91 ;
    double open2 = 0.77;
    double closed2 = 0.507;

    double whideOpen1 = 0.51;
    double whideOpen2 = 0.95;

    private PIDController controller;
    boolean usingPid = false;
    public static double p = 0.05, i = 0.05, d = 0.0;
    public static double f = 0.05;
    public static int target = 0;
    private final double ticks_in_degree = 288 / 180.0;
    double cpoz;
    private Servo demoServo2;
    private static final double MAX_POSITION_SERVO_WRIST = 1;
    private static final double MIN_POSITION_SERVO_WRIST = -0.5;


    private ElapsedTime runtime = new ElapsedTime();

    // Adjust these values based on your robot and gearin
    boolean c3 = false;

    @Override
    public void runOpMode() {
        LeftMotor = (DcMotor) this.hardwareMap.dcMotor.get("LeftMotor");
        RightMotor = (DcMotor) this.hardwareMap.dcMotor.get("RightMotor");

        demoServo = hardwareMap.servo.get("servo");
        demoServoRight = hardwareMap.servo.get("servo2");
        demoServoLeft = hardwareMap.servo.get("servo3");
        planeServo = hardwareMap.servo.get("servo4");

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        LeftArmMotor = hardwareMap.get(DcMotorEx.class, "LeftArmMotor");
        RightArmMotor = hardwareMap.get(DcMotorEx.class, "RightArmMotor");
        RightArmMotor.setDirection(DcMotor.Direction.REVERSE);
        LeftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reverse one of the motors if needed
        RightArmMotor.setDirection(DcMotor.Direction.REVERSE);

        demoServoRight.setPosition(closed1);
        demoServoLeft.setPosition(closed2);
        demoServo.setPosition(1.0);
        planeServo.setPosition(0);
        encoderLeft = LeftMotor;
        encoderRight = RightMotor;
        LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {

            cpoz =(int)(((double)encoderRight.getCurrentPosition()/8192.0)*28.0);
            telemetry.addData("ticks: ", LeftArmMotor.getCurrentPosition());
            telemetry.addData("encoderLeft",LeftMotor.getCurrentPosition());
            telemetry.addData("encoderRight",RightMotor.getCurrentPosition());
            telemetry.addData("encoderRight2",cpoz);
            double leftStickX = (double) this.gamepad1.left_stick_y;
            double leftStickY = (double) this.gamepad1.right_stick_x;
            double RightMotorPower = leftStickY + leftStickX;
            double LeftMotorPower = leftStickY - leftStickX;

            this.LeftMotor.setPower(-LeftMotorPower);
            this.RightMotor.setPower(-RightMotorPower);

            controller.setPID(p, i, d);
            int armPos = (LeftArmMotor.getCurrentPosition()+RightArmMotor.getCurrentPosition())/2;
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
            double power = pid + ff;
//            if (gamepad1.left_stick_y > 0) {
//                target += 2; // Adjust power based on your needs
//            }
//            // Gamepad button Y for moving the arm down
//            else if (gamepad1.left_stick_y < 0) {
//                target -= 2;
//            }
            // Stop the a2rm if no buttons are pressed
            // 11- autoblue
//            if (!gamepad1.dpad_left) {
//                if(isButtonHeld) {
//                    timer.reset();
//                    isButtonHeld = true;
//                }
//            }
//            if (!gamepad1.dpad_right) {
//                if(isButtonHeld1) {
//                    timer.reset();
//                    isButtonHeld1 = true;
//                }
//            }
//            if (isButtonHeld = true && Nivel>1) {
//                if(timer.time()<1000){
//                    Nivel--;
//                    isButtonHeld=false;
//                }
//                timer.reset();
//            }
//            if (isButtonHeld1 = true && Nivel<6) {
//                if(timer.time() <1000){
//                    Nivel++;
//                    isButtonHeld1=false;
//                }
//                timer.reset();
//            }
//            if(gamepad1.left_stick_button){
//                Nivel=6;
//            }
            if (gamepad1.dpad_down) {
                demoServoRight.setPosition(whideOpen1);
                demoServoLeft.setPosition(whideOpen2);
            }
            LeftArmMotor.setPower(power);
            RightArmMotor.setPower(power);
            if (gamepad1.a) {
                target = -8;        //inainte -3
                demoServo.setPosition(0.52);
            }
            if (gamepad1.b) {
                target = -25;       //inainte -20
                demoServo.setPosition(1.0);
            }
            //          first pixel position
            if (gamepad1.x) {
                target = -685;      //inainte -740
                demoServo.setPosition(1.0);
            }
            if (gamepad1.y) {
                    target = -610;      //inainte -575
                    demoServo.setPosition(0.33);
            }
            //drone
            if (gamepad1.left_stick_button) { //  gamepad1.getButton(Gamepad.Keys.LEC)
                planeServo.setPosition(1);
            }
            if (gamepad1.right_stick_button) {
                planeServo.setPosition(0);
            }

            if (gamepad1.right_trigger > 0) {
                demoServoRight.setPosition(closed1);
            }
            if (gamepad1.right_bumper) {
                demoServoRight.setPosition(open1);
            }
            if (gamepad1.left_trigger > 0) {
                demoServoLeft.setPosition(closed2);
            }
            if (gamepad1.left_bumper) {
                demoServoLeft.setPosition(open2);
            }

//            if (gamepad1.right_stick_y < 0) {
//                if (demoServo.getPosition() < 1) {
//                    demoServo.setPosition(demoServo.getPosition() + 0.005);
//                }
//            }
//            else if (gamepad1.right_stick_y > 0) {
//                if (demoServo.getPosition() > 0) {
//                    demoServo.setPosition(demoServo.getPosition() - 0.005);
//                }
//            }

            telemetry.addData("Arm Position", LeftArmMotor.getCurrentPosition());
            telemetry.addData("RT Pressed", gamepad1.right_trigger > 0.1);
            telemetry.addData("LT Pressed", gamepad1.left_trigger > 0.1);
            telemetry.addData("Servo Position", demoServo.getPosition());
            telemetry.addData("Nivel: ", Nivel);

            telemetry.addData("Right Servo Position", demoServoRight.getPosition());
            telemetry.addData("Left Servo Position", demoServoLeft.getPosition());

            telemetry.update();
        }
    }
}