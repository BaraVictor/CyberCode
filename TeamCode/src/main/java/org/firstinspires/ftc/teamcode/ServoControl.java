package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "ma-ta", group = "TeleOp")
public class ServoControl extends OpMode {
    private Servo demoServo;
    private static final double MAX_POSITION = 0.4;
    private static final double MIN_POSITION = 0.0;

    @Override
    public void init() {
        demoServo = hardwareMap.servo.get("servo1");
        demoServo.setPosition(MAX_POSITION);
    }

    @Override
    public void loop() {
        // Check if RT (Right Trigger) is pressed to set the servo to max position
        if (gamepad1.right_trigger > 0.1) {
            if (demoServo.getPosition() < MAX_POSITION) {
                demoServo.setPosition(demoServo.getPosition() + 0.01);
            }
        }
        // Check if LT (Left Trigger) is pressed to set the servo to min position
        else if (gamepad1.left_trigger > 0.1) {
            if (demoServo.getPosition() > MIN_POSITION) {
                demoServo.setPosition(demoServo.getPosition() - 0.01);
            }
        }

        // Add any additional logic here...

        telemetry.addData("RT Pressed", gamepad1.right_trigger > 0.1);
        telemetry.addData("LT Pressed", gamepad1.left_trigger > 0.1);
        telemetry.addData("Servo Position", demoServo.getPosition());
        telemetry.update();
    }
}
