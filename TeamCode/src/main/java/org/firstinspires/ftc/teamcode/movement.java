//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(
        name = "movement",
        group = "TeleOp"
)
public class movement extends OpMode {
    DcMotor motor2;
    DcMotor motor1;

    public movement() {
    }

    public void init() {
        this.motor1 = (DcMotor)this.hardwareMap.dcMotor.get("motor1");
        this.motor2 = (DcMotor)this.hardwareMap.dcMotor.get("motor2");
    }

    public void loop() {
        double leftStickX = (double)this.gamepad1.left_stick_y;
        double leftStickY = (double)this.gamepad1.left_stick_x;
        double motor2Power = leftStickY + leftStickX;
        double motor1Power = leftStickY - leftStickX;
        this.motor1.setPower(-motor1Power/2);
        this.motor2.setPower(-motor2Power/2);
    }
}
