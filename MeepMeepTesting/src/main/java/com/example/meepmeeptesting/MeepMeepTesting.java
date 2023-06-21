//package com.example.meepmeeptesting;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.noahbres.meepmeep.MeepMeep;
//import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
//import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
//
//public class MeepMeepTesting {
//    public static void main(String[] args) {
//        MeepMeep meepMeep = new MeepMeep(800);
//
//        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(52.48291908330528, 40, Math.toRadians(180), Math.toRadians(180), 14.2)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(-33, -60, Math.toRadians(-180)))
////                                .splineToSplineHeading(new Pose2d(-12.5, -60, Math.toRadians(-180)), Math.toRadians(180)) set x to 0
////                                .splineToConstantHeading(new Vector2d(-12.5, -11.5), Math.toRadians(100))
////                                .splineToConstantHeading(new Vector2d(-24.5, -11.5), Math.toRadians(70)) weird substation path
//                                  .splineToConstantHeading(new Vector2d(-35.5, -35.5), Math.toRadians(170))
////                                .splineTo(new Vector2d(-23.5, -12), Math.toRadians(180))
//                                .splineToConstantHeading(new Vector2d(-23.5, -12), Math.toRadians(25))
//
//                                .build()
//                );
//
//        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
//                .setDarkMode(true)
//                .setBackgroundAlpha(0.95f)
//                .addEntity(myBot)
//                .start();
//    }
//}
package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d initialPosition = new Pose2d(-36, -60, Math.toRadians(-90));
        Pose2d secondPosition = new Pose2d(-36, -24, Math.toRadians(-90));
        Pose2d finalPosition = new Pose2d(-24, -10, Math.toRadians(180));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(initialPosition)
                                .setReversed(true)
                                .splineToSplineHeading(secondPosition, calculateTangent(initialPosition,secondPosition))
                                .splineToSplineHeading(finalPosition, Math.toRadians(0))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    public static double calculateTangent(Pose2d initialPosition, Pose2d finalPosition) {
        double xd = initialPosition.getX() - finalPosition.getX();
        double yd = initialPosition.getY() - finalPosition.getY();
        return Math.atan2(yd,xd) - Math.PI;
    }
    public static double calculateTangent(Vector2d initialPosition, Vector2d finalPosition) {
        double xd = initialPosition.getX() - finalPosition.getX();
        double yd = initialPosition.getY() - finalPosition.getY();
        return Math.atan2(yd,xd) - Math.PI;
    }
    public static double calculateTangent(Pose2d initialPosition, Vector2d finalPosition) {
        double xd = initialPosition.getX() - finalPosition.getX();
        double yd = initialPosition.getY() - finalPosition.getY();
        return Math.atan2(yd,xd) - Math.PI;
    }
    public static double calculateTangent(Vector2d initialPosition, Pose2d finalPosition) {
        double xd = initialPosition.getX() - finalPosition.getX();
        double yd = initialPosition.getY() - finalPosition.getY();
        return Math.atan2(yd,xd) - Math.PI;
    }
}