package org.firstinspires.ftc.teamcode.PSRobotLibs.lib.utils.vision;

import android.graphics.Bitmap;
import android.os.Environment;
import android.support.annotation.Nullable;
import android.util.Rational;

import com.vuforia.Image;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.net.MalformedURLException;
import java.util.Vector;


/**
 * Created on 9/13/2017.
 */

public class PSVisionUtils {
    /**
     * Saves an inputted bitmap to a directory
     *
     * @param bitmap  Input bitmap
     * @param name    Name of the saved file
     * @param fileDir The name of the file directory for the image to be saved to
     */
    public static void saveImageToFile(Bitmap bitmap, String name, String fileDir) {

        String root = Environment.getExternalStorageDirectory().getAbsolutePath();
        File myDir = new File(root + fileDir);
        myDir.mkdirs();

        String fname = name + ".jpg";
        File file = new File(myDir, fname);
        if (file.exists()) file.delete();
        try {
            FileOutputStream out = new FileOutputStream(file);
            bitmap.compress(Bitmap.CompressFormat.JPEG, 90, out);
            out.flush();
            out.close();

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Converts a bitmap and a type to a openCv mat
     *
     * @param bit    the input bitmap
     * @param cvType the mat type
     * @return output mat
     */
    public static Mat bitmapToMat(Bitmap bit, int cvType) {
        Mat newMat = new Mat(bit.getHeight(), bit.getWidth(), cvType);
        Utils.bitmapToMat(bit, newMat);
        return newMat;
    }

    /**
     * Converts a openCv mat to a bitmap
     *
     * @param mat input mat
     * @return output bitmap
     */
    public static Bitmap matToBitmap(Mat mat) {
        Bitmap newBit = Bitmap.createBitmap(mat.cols(), mat.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(mat, newBit);
        return newBit;
    }

    @Nullable
    public static Image getImageFromFrame(VuforiaLocalizer.CloseableFrame frame, int format) {

        long numImgs = frame.getNumImages();
        for (int i = 0; i < numImgs; i++) {
            if (frame.getImage(i).getFormat() == format) {
                return frame.getImage(i);
            }//if
        }//for

        return null;
    }

    public static void refreshKernel(Mat kernel, int height, int width, int TARGETHEIGHT, int TARGETWIDTH) {

        if (kernel == null || width != TARGETWIDTH || height != TARGETHEIGHT) {
            if (kernel != null) {
                kernel.release();
            }
            kernel = Mat.ones(TARGETHEIGHT, TARGETWIDTH, CvType.CV_8U);
        }

    }

    /**
     * Method to find the centroid of a binary Mask
     *
     * @param mask the input binary mask
     * @return a openCv point with the x and y values of the centroid
     */
    public static Point maskCentroid(Mat mask) {
        Moments mmnts = Imgproc.moments(mask, true);
        return new Point(mmnts.get_m10() / mmnts.get_m00(), mmnts.get_m01() / mmnts.get_m00());
    }



}
