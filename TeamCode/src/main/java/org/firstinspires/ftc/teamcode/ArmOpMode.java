package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ManualDriving", group = "TeleOp")
public class ArmOpMode extends LinearOpMode {

    private DcMotor motorArm1;
    private DcMotor motorArm2;
    DcMotor motor2;
    DcMotor motor1;

    private Servo demoServo;
    private static final double MAX_POSITION_SERVO = 0.4;
    private static final double MIN_POSITION_SERVO = 0.0;

    private Servo demoServo2;
    private static final double MAX_POSITION_SERVO_WRIST = 1;
    private static final double MIN_POSITION_SERVO_WRIST = -0.5;


    private ElapsedTime runtime = new ElapsedTime();

    // Adjust these values based on your robot and gearing
    private final int MAX_POSITION = 0;
    private final int MIN_POSITION = -1000;
    @Override
    public void runOpMode() {

        demoServo = hardwareMap.servo.get("servo1");
        demoServo.setPosition(MAX_POSITION_SERVO);
        demoServo2 = hardwareMap.servo.get("servo2");
        demoServo2.setPosition(MAX_POSITION_SERVO_WRIST);

        motorArm1 = hardwareMap.get(DcMotor.class, "motor_arm1");
        motorArm2 = hardwareMap.get(DcMotor.class, "motor_arm2");
        motor1 = (DcMotor)this.hardwareMap.dcMotor.get("motor1");
        motor2 = (DcMotor)this.hardwareMap.dcMotor.get("motor2");

        // Reverse one of the motors if needed
        motorArm2.setDirection(DcMotor.Direction.REVERSE);

        // Set motor mode to RUN_USING_ENCODER to enable encoder position tracking
        motorArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        runtime.reset();



        while (opModeIsActive()) {
          telemetry.addData("ticks: ", motorArm1.getCurrentPosition());
            double leftStickX = (double)this.gamepad2.left_stick_y;
            double leftStickY = (double)this.gamepad2.left_stick_x;
            double motor2Power = leftStickY + leftStickX;
            double motor1Power = leftStickY - leftStickX;
            motor1.setPower(-motor1Power/2);
            motor2.setPower(-motor2Power/2);

            if (gamepad1.right_trigger > 0.1) {
                if (demoServo.getPosition() < MAX_POSITION_SERVO) {
                    demoServo.setPosition(demoServo.getPosition() + 0.02);
                }
            }
            // Check if LT (Left Trigger) is pressed to set the servo to min position
            else if (gamepad1.left_trigger > 0.1) {
                if (demoServo.getPosition() > MIN_POSITION_SERVO) {
                    demoServo.setPosition(demoServo.getPosition() - 0.02 );
                }
            }

            if (gamepad1.right_stick_y > 0) {
                if (demoServo2.getPosition() < MAX_POSITION_SERVO_WRIST) {
                    demoServo2.setPosition(demoServo2.getPosition() + 0.02);
                }
            }
            // Check if LT (Left Trigger) is pressed to set the servo to min position
            else if (gamepad1.right_stick_y < 0) {
                if (demoServo2.getPosition() > MIN_POSITION_SERVO_WRIST) {
                    demoServo2.setPosition(demoServo2.getPosition() - 0.02);
                }
            }

            if(gamepad1.x)
            {
                // seteaza bratul in pozitia de a arunca pixelul
                if(motorArm1.getCurrentPosition()>-750) {
                    motorArm1.setPower(-1.0);
                    motorArm2.setPower(-1.0);
                }
                if(motorArm1.getCurrentPosition()<-750) {
                    motorArm1.setPower(1.0);
                    motorArm2.setPower(1.0);
                }
            }
            if(gamepad1.b)
            {
                // seteaza bratul in pozitia de a ridica pixelul
                demoServo.setPosition(0.23);
                demoServo2.setPosition(0.23);
                if(motorArm1.getCurrentPosition()<-5) {
                    motorArm1.setPower(-1.0);
                    motorArm2.setPower(-1.0);
                }
                if(motorArm1.getCurrentPosition()<-5) {
                    motorArm1.setPower(1.0);
                    motorArm2.setPower(1.0);
                }
            }
            int targetPosition = -500;

            while (gamepad1.a && motorArm1.getCurrentPosition() > targetPosition) {
                moveArmDown();

                motorArm1.setTargetPosition(targetPosition);
                motorArm2.setTargetPosition(targetPosition);

                // Add a short delay to avoid busy-waiting and allow the loop to execute periodically
                sleep(50);
            }
            while (gamepad1.a && motorArm1.getCurrentPosition() < targetPosition) {
                moveArmUp();

                motorArm1.setTargetPosition(targetPosition);
                motorArm2.setTargetPosition(targetPosition);

                // Add a short delay to avoid busy-waiting and allow the loop to execute periodically
                sleep(50);
            }/*
            if(gamepad1.a)
            {
                // seteaza bratul in pozitia de a ridica pixelul
                if(motorArm1.getCurrentPosition()>-500) {
                    motorArm1.setPower(-1.0);
                    motorArm2.setPower(-1.0);
                }
                if(motorArm1.getCurrentPosition()<-500) {
                    motorArm1.setPower(1.0);
                    motorArm2.setPower(1.0);
                }
                motorArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
*/
            // Gamepad button A for moving the arm up
            if (gamepad1.left_stick_y > 0 && motorArm1.getCurrentPosition() < MAX_POSITION) {
                moveArmUp();
            }
            // Gamepad button Y for moving the arm down
            else if (gamepad1.left_stick_y < 0 && motorArm1.getCurrentPosition() > MIN_POSITION) {
                moveArmDown();
            }
            // Stop the arm if no buttons are pressed
            else {
                stopArm();
            }
            //poz MAX = 1000;poz up = 500; poz minim = 0;
            if(gamepad1.left_stick_y == 0 && motorArm1.getCurrentPosition()<-1000)
            {
                motorArm1.setPower(0.5);
                motorArm2.setPower(0.5);
            }
            else if(gamepad1.left_stick_y == 0 && motorArm1.getCurrentPosition()<-950)
            {
                motorArm1.setPower(0.45);
                motorArm2.setPower(0.45);
            }
            else if(gamepad1.left_stick_y == 0 && motorArm1.getCurrentPosition()<-900)
            {
                motorArm1.setPower(0.4);
                motorArm2.setPower(0.4);
            }
            else if(gamepad1.left_stick_y == 0 && motorArm1.getCurrentPosition()<-850)
            {
                motorArm1.setPower(0.35);
                motorArm2.setPower(0.35);
            }
            else if(gamepad1.left_stick_y == 0 && motorArm1.getCurrentPosition()<-800)
            {
                motorArm1.setPower(0.3);
                motorArm2.setPower(0.3);
            }
            else if(gamepad1.left_stick_y == 0 && motorArm1.getCurrentPosition()<-750)
            {
                motorArm1.setPower(0.25);
                motorArm2.setPower(0.25);
            }
            else if(gamepad1.left_stick_y == 0 && motorArm1.getCurrentPosition()<-700)
            {
                motorArm1.setPower(0.2);
                motorArm2.setPower(0.2);
            }
            else if(gamepad1.left_stick_y == 0 && motorArm1.getCurrentPosition()<-650)
            {
                motorArm1.setPower(0.15);
                motorArm2.setPower(0.15);
            }
            else if(gamepad1.left_stick_y == 0 && motorArm1.getCurrentPosition()<-600)
            {
                motorArm1.setPower(0.1);
                motorArm2.setPower(0.1);
            }
            else if(gamepad1.left_stick_y == 0 && motorArm1.getCurrentPosition()<-550)
            {
                motorArm1.setPower(0.05);
                motorArm2.setPower(0.05);
            }
            else if(gamepad1.left_stick_y == 0 && motorArm1.getCurrentPosition()==-400)
            {
                motorArm1.setPower(0);
                motorArm2.setPower(0);
            }

            /*if(gamepad1.left_stick_y == 0 && motorArm1.getCurrentPosition()<-450)
            {
                motorArm1.setPower(-0.05);
                motorArm2.setPower(-0.05);
            }*/
            else if(gamepad1.left_stick_y == 0 && motorArm1.getCurrentPosition()<-400)
            {
                motorArm1.setPower(-0.1);
                motorArm2.setPower(-0.1);
            }
            else if(gamepad1.left_stick_y == 0 && motorArm1.getCurrentPosition()<-350)
            {
                motorArm1.setPower(-0.15);
                motorArm2.setPower(-0.15);
            }
            else if(gamepad1.left_stick_y == 0 && motorArm1.getCurrentPosition()<-300)
            {
                motorArm1.setPower(-0.2);
                motorArm2.setPower(-0.2);
            }
            else if(gamepad1.left_stick_y == 0 && motorArm1.getCurrentPosition()<-250)
            {
                motorArm1.setPower(-0.25);
                motorArm2.setPower(-0.25);
            }
            else if(gamepad1.left_stick_y == 0 && motorArm1.getCurrentPosition()<-200)
            {
                motorArm1.setPower(-0.3);
                motorArm2.setPower(-0.3);
            }
            else if(gamepad1.left_stick_y == 0 && motorArm1.getCurrentPosition()<-150)
            {
                motorArm1.setPower(-0.35);
                motorArm2.setPower(-0.35);
            }
            else if(gamepad1.left_stick_y == 0 && motorArm1.getCurrentPosition()<-100)
            {
                motorArm1.setPower(-0.4);
                motorArm2.setPower(0.4);
            }
            else if(gamepad1.left_stick_y == 0 && motorArm1.getCurrentPosition()<-50)
            {
                motorArm1.setPower(-0.45);
                motorArm2.setPower(-0.45);
            }
            else if(gamepad1.left_stick_y == 0 && motorArm1.getCurrentPosition()<0)
            {
                motorArm1.setPower(-0.5);
                motorArm2.setPower(-0.5);
            }

            telemetry.addData("Arm Position", motorArm1.getCurrentPosition());
            telemetry.addData("RT Pressed", gamepad1.right_trigger > 0.1);
            telemetry.addData("LT Pressed", gamepad1.left_trigger > 0.1);
            telemetry.addData("Servo Position", demoServo.getPosition());
            telemetry.addData("Servo Wrist Position", demoServo2.getPosition());
            telemetry.update();
        }
    }

    private void moveArmUp() {
        motorArm1.setPower(1); // Adjust power based on your needs
        motorArm2.setPower(1); // Adjust power based on your needs
    }

    private void moveArmDown() {
        motorArm1.setPower(-1); // Adjust power based on your needs
        motorArm2.setPower(-1); // Adjust power based on your needs
    }

    private void stopArm() {
        motorArm1.setPower(0);
        motorArm2.setPower(0);
    }
}
