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
    private Servo demoServo2;
    private static final double MAX_POSITION_SERVO_WRIST = 1.3;
    private static final double MIN_POSITION_SERVO_WRIST = 0.0;

    @Override
    public void init() {
        demoServo2 = hardwareMap.servo.get("servo2");
        demoServo2.setPosition(MAX_POSITION_SERVO_WRIST);
    }

    @Override
    public void loop() {
        // Check if RT (Right Trigger) is pressed to set the servo to max position
        demoServo2.setPosition(-1);
        // Check if LT (Left Trigger) is pressed to set the servo to min position
        if (gamepad1.left_trigger>0.1) {
            demoServo2.setPosition(1);
        }

        // Add any additional logic here...

        telemetry.addData("RT Pressed", gamepad1.right_trigger > 0.1);
        telemetry.addData("LT Pressed", gamepad1.left_trigger > 0.1);
        telemetry.addData("Servo Position", demoServo2.getPosition());
        telemetry.update();
    }
}
