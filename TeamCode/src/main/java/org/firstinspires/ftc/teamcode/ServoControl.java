//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(
        name = "SimpleServoControl",
        group = "TeleOp"
)
public class ServoControl extends OpMode {
    private static final int MIN_DEGREE = 30;
    private static final int MAX_DEGREE = 180;
    private static final int DEGREE_INCREMENT = 30;
    private static final double[] SERVO_POSITIONS = new double[]{degreesToPosition(30), degreesToPosition(60), degreesToPosition(90), degreesToPosition(120), degreesToPosition(150), degreesToPosition(180)};
    private Servo servo;

    public ServoControl() {
    }

    private static double degreesToPosition(int degrees) {
        return (double)(degrees - 30) / 150.0;
    }

    public void init() {
        this.servo = (Servo)this.hardwareMap.servo.get("servo1");
    }

    public void loop() {
        double targetPosition;
        if (this.gamepad1.a) {
            targetPosition = 0.6;
            this.servo.setPosition(targetPosition);
            this.telemetry.addData("Servo Position", this.servo.getPosition());
            this.telemetry.update();
        }

        if (this.gamepad1.b) {
            targetPosition = -0.6;
            this.servo.setPosition(targetPosition);
            this.telemetry.addData("Servo Position", this.servo.getPosition());
            this.telemetry.update();
        }

    }
}
