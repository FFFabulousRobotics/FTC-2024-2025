package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.auto.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.auto.commands.ForwardCommand;
import org.firstinspires.ftc.teamcode.auto.commands.GotoPosCommand;
import org.firstinspires.ftc.teamcode.auto.commands.GotoPosWithHeadingCommand;
import org.firstinspires.ftc.teamcode.auto.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.auto.commands.SetLiftPositionCommand;
import org.firstinspires.ftc.teamcode.auto.commands.SleepCommand;
import org.firstinspires.ftc.teamcode.hardware.RobotAuto;
import org.firstinspires.ftc.teamcode.hardware.RobotTop;

@Autonomous
public class HangAuto extends LinearOpMode {
    final int STRETCH_BACK_POSITION = 70;
    final int STRETCH_OUT_POSITION = 1500;
    final double SPIN_DEFAULT_POSITION_L = 0.54;
    final double SPIN_DEFAULT_POSITION_R = 0.51;
    final double SPIN_HOVERING_POSITION_L = 0.25;
    final double SPIN_HOVERING_POSITION_R = 0.78;
    final double SPIN_DOWN_POSITION = 0;
    final double TURN_BACK_POSITION = 0.5;
    final double TURN_LOCK_POSITION = 0.5;
    final double TURN_HOVERING_POSITION = 0.75;
    final double TURN_DOWN_POSITION = 0.85;
    final double GRAB_OPEN_POSITION = 0.4;
    final double GRAB_CLOSE_POSITION = 0.92;
    final double TOP_BACK = 0.03;
    final double TOP_OUT = 0.66;
    final double BACK_CLOSE = 0.7;
    final double BACK_OPEN = 0.1;

    final private double[] posHanging = {10.88, -22, 0};//挂的地方
    final private double[] posHanging2 = {15, -22, 0};//第二次挂的地方(有误差)
    final private double[] posHanging3 = {13, -23, 0};//第三次挂的地方(有误差)
    final private double[] posHanging4 = {7, -30, 0};//第四次挂的地方(有误差)
    final private double[] posMiddleStop = {-16, -24, 0};//中间停靠点
    final private double[] posReadyForPush0 = {-18, -50, 0};//准备 准备推的位置
    final private double[] posReadyForPush = {-31, -54, 0};//准备推的位置
    final private double[] posReadyForPush2 = {-40, -54, 0};//准备推的位置2
    final private double[] posPushed = {-32, -11.5, 0};//推到这个位置
    final private double[] posBack = {-31, -20, 0};//回一点位置
    final private double[] posBack1 = {-31, -20, 195};//回一点位置 旋转
    final private double[] posGet = {-28,-3.5,195};//拿方块地方
    final private double[] posGet1 = {-30,-3.5,180};//拿方块地方
    final private double[] parkPosition = {-40.0, -5, 270};//停靠

    RobotTop robotTop;
    RobotAuto robotAuto;

    @Override
    public void runOpMode(){
        robotTop = new RobotTop(this);
        robotAuto = new RobotAuto(this);
        waitForStart();
        robotTop.setArmLeftSpinPosition(SPIN_DEFAULT_POSITION_L);
        robotTop.setArmRightSpinPosition(SPIN_DEFAULT_POSITION_R);
        robotTop.setLiftServoPosition(BACK_CLOSE);
        hangSample1();
        pushSample1();
        hangSample2();
        pushSample2();
        //getSample();
        hangSample3();
        getSample2();
        hangSample4();
        park();
    }
    protected void hangSample1(){
        ParallelCommandGroup cmd1 = new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new GotoPosWithHeadingCommand(robotAuto,posHanging[0],posHanging[1],posHanging[2]),
                        new ForwardCommand(robotAuto,true,0.5)
                ),
                new SequentialCommandGroup(
                        new InstantCommand(() -> robotTop.setTurnPosition(TURN_LOCK_POSITION)),
                        new SleepCommand(300),
                        new SetLiftPositionCommand(robotAuto,800)
                )
        );
        cmd1.runCommand();
        SequentialCommandGroup cmd2 = new SequentialCommandGroup(
                new InstantCommand(() -> robotTop.setLiftPower(-0.4)),
                new SleepCommand(200),
                new InstantCommand(() -> robotTop.setLiftPower(0)),
                new InstantCommand(robotAuto::release),
                new InstantCommand(() -> robotTop.setLiftPower(0.2)),
                new SleepCommand(200),
                new InstantCommand(() -> robotTop.setLiftPower(0))
        );
        cmd2.runCommand();
    }

    protected void pushSample1(){
        SequentialCommandGroup cmd3 = new SequentialCommandGroup(
                new GotoPosCommand(robotAuto,posMiddleStop[0],posMiddleStop[1],true),
                new GotoPosWithHeadingCommand(robotAuto,posReadyForPush0[0],posReadyForPush0[1],posReadyForPush0[2],true),
                //new SleepCommand(500),
                new GotoPosWithHeadingCommand(robotAuto,posReadyForPush[0],posReadyForPush[1],posReadyForPush[2],true),
                new GotoPosWithHeadingCommand(robotAuto,posPushed[0],posPushed[1],posPushed[2],true),
                new GotoPosWithHeadingCommand(robotAuto,posBack[0],posBack[1],posBack[2]),
                new GotoPosWithHeadingCommand(robotAuto,posBack1[0],posBack1[1],posBack1[2]),
                new GotoPosWithHeadingCommand(robotAuto,posGet[0],posGet[1],posGet[2]),
                //new ForwardCommand(robotAuto,true,0.5),
                //new SleepCommand(300),
                new InstantCommand(robotAuto::grab),
                //new SleepCommand(300),
                new ForwardCommand(robotAuto,1)

        );
        cmd3.runCommand();
    }
    protected void hangSample2(){
        ParallelCommandGroup cmd1 = new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new GotoPosWithHeadingCommand(robotAuto,posHanging2[0],posHanging2[1],posHanging2[2]),
                        new ForwardCommand(robotAuto,true,0.5)
                ),
                new SequentialCommandGroup(
                        new InstantCommand(() -> robotTop.setTurnPosition(TURN_LOCK_POSITION)),
                        new SleepCommand(300),
                        new SetLiftPositionCommand(robotAuto,800)
                )
        );
        cmd1.runCommand();
        SequentialCommandGroup cmd2 = new SequentialCommandGroup(
                new InstantCommand(() -> robotTop.setLiftPower(-0.5)),
                new SleepCommand(300),
                new InstantCommand(() -> robotTop.setLiftPower(0)),
                new InstantCommand(robotAuto::release),
                new InstantCommand(() -> robotTop.setLiftPower(0.2)),
                new SleepCommand(200),
                new InstantCommand(() -> robotTop.setLiftPower(0))
        );
        cmd2.runCommand();
    }

    protected void pushSample2(){
        SequentialCommandGroup cmd3 = new SequentialCommandGroup(
                new GotoPosCommand(robotAuto,posMiddleStop[0],posMiddleStop[1],true),
                //new GotoPosWithHeadingCommand(robotAuto,posReadyForPush0[0],posReadyForPush0[1],posReadyForPush0[2],true),
                new GotoPosWithHeadingCommand(robotAuto,posReadyForPush2[0],posReadyForPush2[1],posReadyForPush2[2],true),
                new GotoPosWithHeadingCommand(robotAuto,posPushed[0],posPushed[1],posPushed[2],true),
                new GotoPosWithHeadingCommand(robotAuto,posBack[0],posBack[1],posBack[2]),
                new GotoPosWithHeadingCommand(robotAuto,posBack1[0],posBack1[1],posBack1[2]),
                new GotoPosWithHeadingCommand(robotAuto,posGet[0],posGet[1],posGet[2]),
                //new ForwardCommand(robotAuto,true,0.5),
                //new SleepCommand(200),
                new InstantCommand(robotAuto::grab),
                //new SleepCommand(300),
                new ForwardCommand(robotAuto,1)

        );
        cmd3.runCommand();
    }

//    protected void getSample(){
//        SequentialCommandGroup cmd5 = new SequentialCommandGroup(
//                new InstantCommand(() -> robotTop.setLiftPower(0)),
//                new GotoPosWithHeadingCommand(robotAuto,posGet[0],posGet[1],posGet[2]),
//                //new ForwardCommand(robotAuto,-13),
//                new SleepCommand(300),
//                new InstantCommand(robotAuto::grab)
//                //new ForwardCommand(robotAuto,1)
//                //new SleepCommand(500)
//        );
//        cmd5.runCommand();
//    }
protected void hangSample3(){
    ParallelCommandGroup cmd1 = new ParallelCommandGroup(
            new SequentialCommandGroup(
                    new GotoPosWithHeadingCommand(robotAuto,posHanging3[0],posHanging3[1],posHanging3[2]),
                    new ForwardCommand(robotAuto,true,0.5)
            ),
            new SequentialCommandGroup(
                    new InstantCommand(() -> robotTop.setTurnPosition(TURN_LOCK_POSITION)),
                    new SleepCommand(300),
                    new SetLiftPositionCommand(robotAuto,800)
            )
    );
    cmd1.runCommand();
    SequentialCommandGroup cmd2 = new SequentialCommandGroup(
            new InstantCommand(() -> robotTop.setLiftPower(-0.2)),
            new SleepCommand(200),
            //new InstantCommand(() -> robotTop.setLiftPower(0)),
            new InstantCommand(robotAuto::release),
            //new InstantCommand(() -> robotTop.setLiftPower(0.2)),
            new SleepCommand(200),
            new InstantCommand(() -> robotTop.setLiftPower(-0.5)),
            new SleepCommand(100)
    );
    cmd2.runCommand();
}

    protected void getSample2(){
        SequentialCommandGroup cmd5 = new SequentialCommandGroup(
                new InstantCommand(() -> robotTop.setLiftPower(0)),
                new GotoPosWithHeadingCommand(robotAuto,posGet1[0],posGet1[1],posGet1[2]),
                //new ForwardCommand(robotAuto,-13),
                //new SleepCommand(50),
                //new ForwardCommand(robotAuto,true,0.2),
                //new ForwardCommand(robotAuto,1),
                new InstantCommand(robotAuto::grab),
                //new SleepCommand(500),
                new ForwardCommand(robotAuto,1)
                //new SleepCommand(500)
        );
        cmd5.runCommand();
    }
    protected void hangSample4(){
        ParallelCommandGroup cmd1 = new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new GotoPosWithHeadingCommand(robotAuto,posHanging4[0],posHanging4[1],posHanging4[2])
                        //new ForwardCommand(robotAuto,true,0.7)
                ),
                new SequentialCommandGroup(
                        new InstantCommand(() -> robotTop.setTurnPosition(TURN_LOCK_POSITION)),
                        //new SleepCommand(300),
                        new SetLiftPositionCommand(robotAuto,800)
                )
        );
        cmd1.runCommand();
        SequentialCommandGroup cmd2 = new SequentialCommandGroup(
                new InstantCommand(() -> robotTop.setLiftPower(-0.5)),
                new SleepCommand(200),
                new InstantCommand(() -> robotTop.setLiftPower(0)),
                new InstantCommand(robotAuto::release),
                new InstantCommand(() -> robotTop.setLiftPower(0.2)),
                new SleepCommand(200),
                new InstantCommand(() -> robotTop.setLiftPower(0))
        );
        cmd2.runCommand();
    }



    protected void park(){
        ParallelCommandGroup cmd7 = new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new GotoPosWithHeadingCommand(robotAuto,parkPosition[0],parkPosition[1],parkPosition[2])
                ),
                new SequentialCommandGroup(
                        new InstantCommand(() -> robotTop.setTurnPosition(TURN_BACK_POSITION))
                )
        );
        cmd7.runCommand();
    }
}

