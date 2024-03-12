//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(
        name = "Cod_Miscare",
        group = "A"
)
public class Cod_Miscare extends OpMode {
    DcMotor motor2;
    DcMotor motor1;
    double schimbator = 1.0;

    public Cod_Miscare() {
    }

    public void init() {
        this.motor1 = (DcMotor)this.hardwareMap.dcMotor.get("motor1");
        this.motor2 = (DcMotor)this.hardwareMap.dcMotor.get("motor2");
    }

    public void loop() {
        double leftStickX = (double) this.gamepad2.left_stick_y;
        double leftStickY = (double) this.gamepad2.right_stick_x;
        double motor2Power = leftStickY + leftStickX;
        double motor1Power = leftStickY - leftStickX;
        if (gamepad2.right_trigger > 0.1) {
            schimbator = 1;
        }
        if (gamepad2.left_trigger > 0.1) {
            schimbator = 0.75;
        }
        if (gamepad2.right_bumper && gamepad2.left_stick_y < 0) {
            schimbator = 0.25;
            long startTime = System.currentTimeMillis();
            if (System.currentTimeMillis() - startTime < 200) {
                this.motor1.setPower(-motor1Power * schimbator);
                this.motor2.setPower(-motor2Power * schimbator);
            }
            schimbator = 0.5;
            if (System.currentTimeMillis() - startTime < 400) {
                this.motor1.setPower(-motor1Power * schimbator);
                this.motor2.setPower(-motor2Power * schimbator);
            }
            schimbator = 0.75;
            if (System.currentTimeMillis() - startTime < 600) {
                this.motor1.setPower(-motor1Power * schimbator);
                this.motor2.setPower(-motor2Power * schimbator);
            }
            schimbator = 1;
            if (System.currentTimeMillis() - startTime < 800) {
                this.motor1.setPower(-motor1Power * schimbator);
                this.motor2.setPower(-motor2Power * schimbator);
            }
            schimbator = 1.0;
        }
        if (gamepad2.left_bumper) {
            schimbator = 0.5;
        }
        this.motor1.setPower(-motor1Power*schimbator);
        this.motor2.setPower(-motor2Power*schimbator);
    }
}