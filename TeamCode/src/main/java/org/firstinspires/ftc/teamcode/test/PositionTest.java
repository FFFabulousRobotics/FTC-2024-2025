package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotAuto;
import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.hardware.RobotChassis;


@TeleOp
public class PositionTest extends LinearOpMode {

    RobotAuto robotAuto;
    RobotChassis robotChassis;

    @Override
    public void runOpMode() {

        robotAuto = new RobotAuto(this);
        robotChassis = new RobotChassis(this);

        waitForStart();
        while (opModeIsActive()) {
            //robotChassis.driveRobot(gamepad2.left_stick_y, gamepad2.left_stick_x, gamepad2.right_stick_x);
            if (gamepad2.dpad_up) {
                robotChassis.driveRobot(-1, 0, 0);
            } else {
                robotChassis.driveRobot(gamepad2.left_stick_y, gamepad2.left_stick_x, gamepad2.right_stick_x);
            }
            telemetry.addData("x", robotAuto.getPosition().x);
            telemetry.addData("y", robotAuto.getPosition().y);
            telemetry.addData("h", robotAuto.getPosition().h);
            telemetry.update();
            sleep(10);
        }
    }
}
