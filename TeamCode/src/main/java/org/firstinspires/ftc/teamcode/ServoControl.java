package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "SimpleServoControl", group = "TeleOp")
public class ServoControl extends OpMode {

    private static final int MIN_DEGREE = 30;
    private static final int MAX_DEGREE = 180;
    private static final int DEGREE_INCREMENT = 30;

    // Create a lookup table for servo positions
    private static final double[] SERVO_POSITIONS = {
            degreesToPosition(30),
            degreesToPosition(60),
            degreesToPosition(90),
            degreesToPosition(120),
            degreesToPosition(150),
            degreesToPosition(180)
    };

    // Convert degrees to servo position (assuming a linear relationship)
    private static double degreesToPosition(int degrees) {
        return (degrees - MIN_DEGREE) / (double) (MAX_DEGREE - MIN_DEGREE);
    }
    private Servo servo;

    @Override
    public void init() {
        servo = hardwareMap.servo.get("servo1"); // "servo" is the name configured in your robot configuration file
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            double targetPosition = 0.60; // 0.12=30 grade
            servo.setPosition(targetPosition);
            telemetry.addData("Servo Position", servo.getPosition());
            telemetry.update();
        }
        if (gamepad1.b) {
            double targetPosition = -0.60;
            servo.setPosition(targetPosition);
            telemetry.addData("Servo Position", servo.getPosition());
            telemetry.update();
        }
    }
}
