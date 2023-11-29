package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "ArmControlOpMode", group = "TeleOp")
public class ArmOpMode extends LinearOpMode {

    private DcMotor motorArm1;
    private DcMotor motorArm2;

    private ElapsedTime runtime = new ElapsedTime();

    // Adjust these values based on your robot and gearing
    private final int MAX_POSITION = 500; // Maximum encoder count for 90 degrees
    private final int MIN_POSITION = 0;   // Minimum encoder count for 0 degrees

    @Override
    public void runOpMode() {
        motorArm1 = hardwareMap.get(DcMotor.class, "motor_arm1");
        motorArm2 = hardwareMap.get(DcMotor.class, "motor_arm2");

        // Reverse one of the motors if needed
        motorArm2.setDirection(DcMotor.Direction.REVERSE);

        // Set motor mode to RUN_USING_ENCODER to enable encoder position tracking
        motorArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // Gamepad button A for moving the arm up
            if (gamepad1.a && motorArm1.getCurrentPosition() < MAX_POSITION) {
                moveArmUp();
            }
            // Gamepad button Y for moving the arm down
            else if (gamepad1.y && motorArm1.getCurrentPosition() > MIN_POSITION) {
                moveArmDown();
            }
            // Stop the arm if no buttons are pressed
            else {
                stopArm();
            }

            telemetry.addData("Arm Position", motorArm1.getCurrentPosition());
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
