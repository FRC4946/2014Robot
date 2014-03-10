/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc4946.vision;

import edu.wpi.first.wpilibj.camera.AxisCameraException;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.image.BinaryImage;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.CriteriaCollection;
import edu.wpi.first.wpilibj.image.NIVision;
import edu.wpi.first.wpilibj.image.NIVisionException;
import edu.wpi.first.wpilibj.image.ParticleAnalysisReport;
import edu.wpi.first.wpilibj.image.RGBImage;

/**
 *
 * @author Matthew
 */
public class Vision {

    VisionFunctions m_functions = new VisionFunctions();

    public static int[] verticalTargetList = new int[VisionConstants.MAX_PARTICLES];
    public static int[] horizontalTargetList = new int[VisionConstants.MAX_PARTICLES];
    public static int verticalTargetCount = 0;
    public static int horizontalTargetCount = 0;

    /**
     * Filter out everything but large green particles.
     * 
     * @param camera The camera to get the image from
     * @param criteria The image processing criteria
     * @return The binary, filtered image
     */
    public BinaryImage getFilteredImage(AxisCamera camera, CriteriaCollection criteria) {
        BinaryImage filteredImage = null;

        try {
            //ColorImage image = camera.getImage();     // comment if using stored images
            ColorImage image = new RGBImage("/testImage.jpg");		// get the sample image from the cRIO flash

            BinaryImage thresholdImage = image.thresholdHSV(105, 137, 230, 255, 133, 183);   // keep only green objects
            filteredImage = thresholdImage.particleFilter(criteria);           // filter out small particles

            thresholdImage.free();
            image.free();

        } catch (NIVisionException ex) {
            ex.printStackTrace();
 //       } catch (AxisCameraException ex) {        // this is needed if the camera.getImage() is called
 //           ex.printStackTrace();
        }
        return filteredImage;

}

    /**
     * Check if there are any particles in the image
     * 
     * @param image The image to find particles in
     * @return If there are any particles in the given image
     */
    public boolean areParticles(BinaryImage image) {
        int numberOfParticles = 0;

        try {
            numberOfParticles = image.getNumberParticles();
            image.free();
        } catch (NIVisionException ex) {
            ex.printStackTrace();
        }
        return numberOfParticles > 0;
    }

    /**
     * Look at all the particles in the image, and determine if they are targets
     * or not. Then, determine if they are vertical or horizontal targets
     * 
     * @param image The image to find targets in
     */
    public void detectTargets(BinaryImage image) {
        try {
            for (int partID = 0; partID < VisionConstants.MAX_PARTICLES && partID < image.getNumberParticles(); partID++) {
                ParticleAnalysisReport report = image.getParticleAnalysisReport(partID);
                Scores curScores = new Scores();

                //Score each particle on rectangularity and aspect ratio
                curScores.rectangularity = m_functions.scoreRectangularity(report);
                curScores.aspectRatioVertical = m_functions.scoreAspectRatio(image, report, partID, true);
                curScores.aspectRatioHorizontal = m_functions.scoreAspectRatio(image, report, partID, false);

                //Check if the particle is a horizontal target, if not, check if it's a vertical target
                if (m_functions.scoreCompare(curScores, false)) {
                    horizontalTargetList[horizontalTargetCount++] = partID; //Add particle to target array and increment count
                } else if (m_functions.scoreCompare(curScores, true)) {
                    verticalTargetList[verticalTargetCount++] = partID;  //Add particle to target array and increment count
                }
            }
            image.free();
        } catch (NIVisionException ex) {
            ex.printStackTrace();
        }
    }

    /**
     * Report the scores of the given pair of targets. Scores are used to
     * determine the best target.
     * 
     * @param image The image to find targets in
     * @param vertTarget The ID of the vertical target
     * @param horizTarget The ID of the horizontal target
     * @return The report of the specified target
     */
    public TargetReport getTargetScores(BinaryImage image, int vertTarget, int horizTarget) {
        TargetReport curTarget = new TargetReport();
        double horizWidth, horizHeight, vertWidth;

        try {
            ParticleAnalysisReport verticalReport = image.getParticleAnalysisReport(vertTarget);
            ParticleAnalysisReport horizontalReport = image.getParticleAnalysisReport(horizTarget);
            //Measure equivalent rectangle sides for use in score calculation
            horizWidth = NIVision.MeasureParticle(image.image, horizTarget, false, NIVision.MeasurementType.IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE);
            vertWidth = NIVision.MeasureParticle(image.image, vertTarget, false, NIVision.MeasurementType.IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE);
            horizHeight = NIVision.MeasureParticle(image.image, horizTarget, false, NIVision.MeasurementType.IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE);

            //Determine if the horizontal target is in the expected location to the left of the vertical target
            curTarget.leftScore = m_functions.ratioToScore(1.2 * (verticalReport.boundingRectLeft - horizontalReport.center_mass_x) / horizWidth);
            //Determine if the horizontal target is in the expected location to the right of the  vertical target
            curTarget.rightScore = m_functions.ratioToScore(1.2 * (horizontalReport.center_mass_x - verticalReport.boundingRectLeft - verticalReport.boundingRectWidth) / horizWidth);
            //Determine if the width of the tape on the two targets appears to be the same
            curTarget.tapeWidthScore = m_functions.ratioToScore(vertWidth / horizHeight);
            //Determine if the vertical location of the horizontal target appears to be correct
            curTarget.verticalScore = m_functions.ratioToScore(1 - (verticalReport.boundingRectTop - horizontalReport.center_mass_y) / (4 * horizHeight));
            curTarget.totalScore = curTarget.leftScore > curTarget.rightScore ? curTarget.leftScore : curTarget.rightScore;
            curTarget.totalScore += curTarget.tapeWidthScore + curTarget.verticalScore;

            
            image.free();
        } catch (NIVisionException ex) {
            ex.printStackTrace();
        }

        return curTarget;
    }

    /**
     * Look at all the targets, and decide which is the best one
     * 
     * @param image The image to find targets in
     * @return The report of the best target
     */
    public TargetReport getBestTarget(BinaryImage image) {

        TargetReport bestTarget = new TargetReport();
        //Reset the bestTarget scores, and set verticalIndex to first target in case there are no horizontal targets
        bestTarget.totalScore = bestTarget.leftScore = bestTarget.rightScore = bestTarget.tapeWidthScore = bestTarget.verticalScore = 0;
        bestTarget.verticalIndex = verticalTargetList[0];
        for (int i = 0; i < verticalTargetCount; i++) {
            for (int j = 0; j < horizontalTargetCount; j++) {

                TargetReport curTarget = getTargetScores(image, verticalTargetList[i], horizontalTargetList[j]);

                //If the target is the best detected so far store the information about it
                if (curTarget.totalScore > bestTarget.totalScore) {
                    bestTarget.horizontalIndex = horizontalTargetList[j];
                    bestTarget.verticalIndex = verticalTargetList[i];
                    bestTarget.totalScore = curTarget.totalScore;
                    bestTarget.leftScore = curTarget.leftScore;
                    bestTarget.rightScore = curTarget.rightScore;
                    bestTarget.tapeWidthScore = curTarget.tapeWidthScore;
                    bestTarget.verticalScore = curTarget.verticalScore;
                }
            }
            //Determine if the best target is a Hot target
            bestTarget.Hot = m_functions.hotOrNot(bestTarget);
        }
        try {
            image.free();
        } catch (NIVisionException ex) {
            ex.printStackTrace();
        }
        
        return bestTarget;

    }
    
    public double getDistance(BinaryImage image, TargetReport target) {
        ParticleAnalysisReport distanceReport;
        double distance = 0;
        try {
            distanceReport = image.getParticleAnalysisReport(target.verticalIndex);
            distance = m_functions.computeDistance(image, distanceReport, target.verticalIndex);

        } catch (NIVisionException ex) {
            ex.printStackTrace();
        }

        return distance;
    }
}
