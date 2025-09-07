package org.firstinspires.ftc.teamcode.auto.commands;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.auto.Command;
import org.firstinspires.ftc.teamcode.hardware.RobotAuto;
import org.firstinspires.ftc.teamcode.hardware.RobotChassis;

public class ForwardCommand implements Command {
    RobotAuto robotAuto;
    RobotChassis robotChassis;
    double distance;
    double maxDriveSpeed = 1;

    double heading;
    double startTime;
    boolean isTimed = false;
    double time = 0;

    static final double COUNTS_PER_MOTOR_REV = 560;
    static final double DRIVE_GEAR_REDUCTION = (double) 3 / 5;
    // wheel diameter in inches
    static final double WHEEL_DIAMETER_INCHES = 2.55;

    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    public ForwardCommand(RobotAuto robotAuto, double distance){
        this.robotAuto = robotAuto;
        this.robotChassis = robotAuto.robotChassis;
        this.distance = -distance;
    }

    public ForwardCommand(RobotAuto robotAuto, boolean isTimed, double second){
        this.robotAuto = robotAuto;
        this.robotChassis = robotAuto.robotChassis;
        this.distance = second*10;
        this.isTimed = isTimed;
        this.time = second;
    }

    public ForwardCommand(RobotAuto robotAuto, double distance, double speed){
        this.robotAuto = robotAuto;
        this.robotChassis = robotAuto.robotChassis;
        this.distance = -distance;
        this.maxDriveSpeed = speed;
    }

    @Override
    public void iterate() {
        robotAuto.update();
        double turnSpeed = robotAuto.getSteeringCorrection(heading, 0.03);

        // if driving in reverse, the motor correction also needs to be reversed
        if (distance < 0)
            turnSpeed *= -1.0;

        // Apply the turning correction to the current driving speed.
        robotChassis.driveRobot(maxDriveSpeed, 0, -turnSpeed);
    }

    @Override
    public boolean hasNext() {
        if(isTimed){
            return (this.robotAuto.opMode.getRuntime() < startTime + time);
        }else{
            return robotChassis.isAllBusy();
        }
    }

    @Override
    public void init() {
        heading = robotAuto.getHeading();
        robotChassis.resetEncoder();
        int moveCounts = (int) (distance * COUNTS_PER_INCH);
        robotAuto.setStraightTargetPosition(moveCounts);

        robotChassis.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        maxDriveSpeed = Math.abs(maxDriveSpeed);
        robotChassis.driveRobot(maxDriveSpeed, 0, 0);
        this.startTime = this.robotAuto.opMode.getRuntime();
    }

    @Override
    public void finish() {
        robotChassis.stopMotor();
        robotChassis.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
