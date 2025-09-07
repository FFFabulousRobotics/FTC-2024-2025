package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotAuto;
import org.firstinspires.ftc.teamcode.hardware.RobotChassis;

@TeleOp
public class AutoTestForHeading extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotChassis robotChassis = new RobotChassis(this);
        RobotAuto robotAuto = new RobotAuto(this);

        waitForStart();
        if (isStopRequested()) return;
        double currentX,currentY,dx,dy,angle,unitX,unitY,deltaDistance;
        double kp;
        SparkFunOTOS.Pose2D pose = robotAuto.getPosition();
        currentX = pose.x; currentY = pose.y;
        dx = 0-currentX;
        dy = 0-currentY;

        while (opModeIsActive()) {
            SparkFunOTOS.Pose2D pos = robotAuto.getPosition();
            // Log the position to the telemetry
            pose = robotAuto.getPosition();
            currentX = pose.x; currentY = pose.y;
            dx = 0-currentX;
            dy = 0-currentY;
            telemetry.addData("x",dx);
            telemetry.addData("y",dy);

            // change it into a unit length
            angle = Math.atan2(dy,dx);
            unitY = Math.sin(angle);
            unitX = Math.cos(angle);
            telemetry.addData("unitX",unitX);
            telemetry.addData("unitY",unitY);
            telemetry.addData("directAngle",Math.toDegrees(angle)-90);
            telemetry.addData("X coordinate", pose.x);
            telemetry.addData("Y coordinate", pose.y);
            telemetry.addData("Heading angle", pose.h);
            telemetry.update();
            if(gamepad1.a){
                robotAuto.gotoPos(0,0);
                telemetry.update();
            }
            if(gamepad1.b){
                robotAuto.gotoPosWithHeading(0,0, 0);
                telemetry.update();
            }
            if(gamepad1.x){
                robotAuto.turnToHeading(0.3,Math.toDegrees(angle)-90);
            }
            robotChassis.driveRobot(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            sleep(50);
        }
    }
}
