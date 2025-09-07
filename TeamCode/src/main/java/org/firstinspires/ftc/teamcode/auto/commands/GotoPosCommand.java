package org.firstinspires.ftc.teamcode.auto.commands;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.auto.Command;
import org.firstinspires.ftc.teamcode.hardware.PIDController;
import org.firstinspires.ftc.teamcode.hardware.RobotAuto;

public class GotoPosCommand implements Command {

    double desiredX,desiredY;
    RobotAuto robotAuto;
    double currentX,currentY,dx,dy,angle,unitX,unitY,deltaDistance;
    double kp;
    double pGain = 0.06;
    double threshold = 10;//这里本来是0.5
    SparkFunOTOS.Pose2D pose;
    PIDController pidControllerForDistance = new PIDController();

    public GotoPosCommand(RobotAuto robotAuto, double desiredX, double desiredY){
        this.robotAuto = robotAuto;
        this.desiredX = desiredX;
        this.desiredY = desiredY;
    }

    public GotoPosCommand(RobotAuto robotAuto, double desiredX, double desiredY, boolean isFast){
        if(isFast){
            this.pGain = 1;
            this.threshold = 6;
        }
        this.robotAuto = robotAuto;
        this.desiredX = desiredX;
        this.desiredY = desiredY;
    }

    @Override
    public void iterate() {
        // get the position error
        pose = robotAuto.getPosition();
        currentX = pose.x; currentY = pose.y;
        dx = desiredX-currentX;
        dy = desiredY-currentY;

        // change it into a unit length
        angle = Math.atan2(dy,dx);
        unitY = Math.sin(angle);
        unitX = Math.cos(angle);
        deltaDistance = robotAuto.calcDistance(dx,dy);

        // p control
        double rate = pidControllerForDistance.updatePID(deltaDistance);
        if(rate >= 1) rate = 1;
        robotAuto.absoluteDriveRobot(-unitY * rate,unitX * rate,0);
    }

    @Override
    public boolean hasNext() {
        return robotAuto.calcDistance(dx,dy) > threshold;
    }

    @Override
    public void init() {
        SparkFunOTOS.Pose2D pose = robotAuto.getPosition();
        currentX = pose.x; currentY = pose.y;
        dx = desiredX-currentX;
        dy = desiredY-currentY;

        pidControllerForDistance.setPIDArguments(this.pGain,0,0);
        pidControllerForDistance.reset();
    }

    @Override
    public void finish() {
        robotAuto.stopMotor();
    }
}
