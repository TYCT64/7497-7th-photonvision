/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

 package frc.robot;

 import edu.wpi.first.apriltag.AprilTagFieldLayout;
 import edu.wpi.first.apriltag.AprilTagFields;
 import edu.wpi.first.math.Matrix;
 import edu.wpi.first.math.VecBuilder;
 import edu.wpi.first.math.controller.SimpleMotorFeedforward;
 import edu.wpi.first.math.geometry.Rotation3d;
 import edu.wpi.first.math.geometry.Transform3d;
 import edu.wpi.first.math.geometry.Translation2d;
 import edu.wpi.first.math.geometry.Translation3d;
 import edu.wpi.first.math.numbers.N1;
 import edu.wpi.first.math.numbers.N3;
 import edu.wpi.first.math.util.Units;
 
 public class PhotonvisonConstants {
     public static class Vision {
         public static final String kCameraName = "OV9281";
         public static final String RightName = "ov9281Right";
         // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
         public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, Math.toRadians(0), 0));

        public static final Transform3d kRobotToRightCam =
                new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, Math.toRadians(0), 0));
 
         // The layout of the AprilTags on the field
         public static final AprilTagFieldLayout kTagLayout =
                 AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        // public static final AprilTagFieldLayout kTagLayout ;
        // static {
        //     try {
        //         kTagLayout = AprilTagFieldLayout.loadFromResource("reefscape-andymark.json");
        //     } catch (IOException e) {
        //         throw new RuntimeException("Failed to load AprilTag field layout: " + e.getMessage());
        //     }
        // } //手動匯入.json 我不知道他default是不是對的 反正我這個.json是對的 但我現在不知道他try有沒有成功
 
         // The standard deviations of our vision estimated poses, which affect correction rate
         // (Fake values. Experiment and determine estimation noise on an actual robot.)
         public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
         public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);// x軸 y軸 rotation
         /*多 Tag 在 2 公尺誤差 ±10 公分，設 kMultiTagStdDevs = [0.1, 0.1, Math.toRadians(2)]。
        單 Tag 在 3 公尺誤差 ±50 公分，設 kSingleTagStdDevs = [0.5, 0.5, Math.toRadians(10)] 以此類推 尚未測試 */
     }
 

 }