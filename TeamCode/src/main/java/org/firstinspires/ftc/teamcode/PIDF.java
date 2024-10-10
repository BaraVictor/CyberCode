package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.tree.DCTree;
@Config
@TeleOp(name = "PIDF", group = "B")
public class PIDF extends OpMode {
    private PIDController controller;
    boolean usingPid=false;
    public static double p = 0.05, i = 0.05, d = 0;
    public static double f = 0.05;
    public static int target = 0;
    private final double ticks_in_degree = 288 / 180.0;
    private DcMotorEx motor1;
    private DcMotorEx motor2;
    private Servo demoServo;
    private final int MAX_POSITION = 0;
    private final int MIN_POSITION = -1000;
    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motor1 = hardwareMap.get(DcMotorEx.class,"arm_motor1");
        motor2 = hardwareMap.get(DcMotorEx.class,"arm_motor2");
        demoServo = hardwareMap.servo.get("servo");
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2 .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int armPos = (motor1.getCurrentPosition()+motor2.getCurrentPosition())/2;
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
        double power = pid + ff;
//            if (gamepad1.left_stick_y > 0 && motor1.getCurrentPosition() < MAX_POSITION) {
//                target+=2; // Adjust power based on your needs
//            }
//            // Gamepad button Y for moving the arm down
//            else if (gamepad1.left_stick_y < 0 && motor1.getCurrentPosition() > MIN_POSITION) {
//                target-=2;
//            }
//            // Stop the arm if no buttons are pressed

        motor1.setPower(power);
        motor2.setPower(power);
        if(gamepad1.y){
            target = -575;
        }
        if (gamepad1.a) {
            target = -8;        //inainte -3
            demoServo.setPosition(0.65);
        }
        if (gamepad1.b) {
            target = -25;       //inainte -20
            demoServo.setPosition(1.0);
        }

        telemetry.addData("pos ",armPos);
        telemetry.addData("target ",target);
        telemetry.update();
    }
}
