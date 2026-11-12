
package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Flower Test")
public class flower extends LinearOpMode {

    private Servo servo1;
    private Servo servo2;

    @Override
    public void runOpMode() {

        servo1 = hardwareMap.get(Servo.class, "f2");
        servo2 = hardwareMap.get(Servo.class, "f1");

        telemetry.addLine("Flower pronta");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) { //baixo
                servo1.setPosition(0.2);
                servo2.setPosition(0.8);
            }

            if (gamepad1.b) { //shoot
                servo1.setPosition(0.4);
                servo2.setPosition(0.4);
            }

            if (gamepad1.y) { //alto
                servo1.setPosition(0.8);
                servo2.setPosition(0.2);
            }

            telemetry.addLine("Foi");
            telemetry.addData("Servo f2", servo1.getPosition());
            telemetry.addData("Servo f1", servo2.getPosition());
            telemetry.update();
        }
    }
}
