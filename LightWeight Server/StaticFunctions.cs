using Microsoft.Xna.Framework;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace LightWeight_Server
{
    public static class StaticFunctions
    {

        static String[] cardinalKeys = new String[] { "X", "Y", "Z", "A", "B", "C" };
        static String[] axisKeys = new String[] { "A1", "A2", "A3", "A4", "A5", "A6" };
        static String[] rotationKeys = new String[] { "X1", "X2", "X3", "Y1", "Y2", "Y3", "Z1", "Z2", "Z3" };


        public static Matrix CreateFromQuaternionPosition(Quaternion Q, Vector3 Position)
        {
            Matrix matout = Matrix.CreateFromQuaternion(Q);
            matout.Translation = Position;
            return matout;
        }
        /// <summary>
        /// Returns the double array which represents [X, Y, Z, A, B, C] using Carinal dictionary keys.
        /// </summary>
        /// <param name="dictionary"></param> The Dictionary with cardinal keys
        /// <returns></returns>
        public static double[] getCardinalDoubleArray(ConcurrentDictionary<string, double> dictionary)
        {
            double[] outArray = new double[6];
            for (int i = 0; i < 6; i++)
            {
                outArray[i] = Getvalue(dictionary, getCardinalKey(i));
            }
            return outArray;
        }

        /// <summary>
        /// Returns the double array which represents [A1, A2, A3, A4, A5, A6] using Axis dictionary keys.
        /// </summary>
        /// <param name="dictionary"></param> The Dictionary with axis keys
        /// <returns></returns>
        public static double[] getAxisDoubleArray(ConcurrentDictionary<string, double> dictionary)
        {
            double[] outArray = new double[6];
            for (int i = 0; i < 6; i++)
            {
                outArray[i] = Getvalue(dictionary, getAxisKey(i));
            }
            return outArray;
        }

        /// <summary>
        /// Creates a quaternion from Kuka coordinates, ABC
        /// </summary>
        /// <param name="pose"></param> A double[6] array {X, Y, Z, A, B, C} 
        /// <returns></returns>
        public static Quaternion MakeQuaternionFromKuka(double[] pose)
        {
            Matrix Rz = Matrix.CreateRotationZ((float)pose[3]);
            Matrix Ry = Matrix.CreateRotationY((float)pose[4]);
            Matrix Rx = Matrix.CreateRotationX((float)pose[5]);
            Matrix Rotation = Matrix.Multiply(Matrix.Multiply(Rz, Ry), Rx);
            return Quaternion.CreateFromRotationMatrix(Rotation);
        }

        public static Matrix MakeMatrixFromKuka(double[] pose)
        {
            Matrix Rz = Matrix.CreateRotationZ((float)pose[3]);
            Matrix Ry = Matrix.CreateRotationY((float)pose[4]);
            Matrix Rx = Matrix.CreateRotationX((float)pose[5]);
            Matrix poseout = Matrix.Multiply(Matrix.Multiply(Rz, Ry), Rx);
            poseout.Translation = new Vector3((float)pose[0], (float)pose[1], (float)pose[2]);
            return poseout;
        }

        /// <summary>
        /// Gets the value with a while loop of a concurrentDictionary<string, double>.
        /// </summary>
        /// <param name="dictionary"></param> The dictionary holding the values
        /// <param name="key"></param> The key used to get the value
        /// <returns></returns>
        public static double Getvalue(ConcurrentDictionary<string, double> dictionary, String key)
        {
            double outValue = 0;
            bool hasGotValule = false;
            while (!hasGotValule)
            {
                hasGotValule = dictionary.TryGetValue(key, out outValue);
            }
            return outValue;
        }

        public static String getCardinalKey(int Index)
        {
            return cardinalKeys[Index];
        }

        public static String getAxisKey(int Index)
        {
            return axisKeys[Index];
        }

        public static String getrotationKeys(int Index)
        {
            return rotationKeys[Index];
        }
        
        public static void getAxisAngle(Quaternion quaternion, ref Vector3 outAxis, ref float outAngle)
        {
            if (quaternion.W > 1)
            {
                quaternion.Normalize();
            }
            outAngle = 2 * (float)Math.Acos(quaternion.W);
            float s = (float)Math.Sqrt(1 - quaternion.W * quaternion.W);
            if (s < 0.001)
            {
                outAxis.X = quaternion.X;
                outAxis.Y = quaternion.Y;
                outAxis.Z = quaternion.Z;
            }
            else
            {
                outAxis.X = quaternion.X / s;
                outAxis.Y = quaternion.Y / s;
                outAxis.Z = quaternion.Z / s;
            }
        }
        
        public static Quaternion MakeQuaternionFromKuka(float A, float B, float C)
        {
            return Quaternion.CreateFromRotationMatrix((Matrix.CreateRotationZ(A) * Matrix.CreateRotationY(B)) * Matrix.CreateRotationX(C));
        }

        /// <summary>
        /// Returns a double[3] as there is no position data. This only works when the rotation is less than 90 degrees 
        /// and will return iff angle in axis/angle representation is less than 90 degrees.
        /// </summary>
        /// <param name="rotation"></param> quaternion representing the rotation
        /// <returns></returns>
        public static float[] getKukaAngles(Quaternion rotation)
        {
            Vector3 axis = new Vector3();
            float angle = 0;
            getAxisAngle(rotation, ref axis, ref angle);
            if (angle < Math.PI / 2)
            {
                Matrix rotationMat = Matrix.CreateFromQuaternion(rotation);
                float[] angles = new float[3];
                angles[2] = (float)Math.Atan2(rotationMat.M32, rotationMat.M33);
                angles[1] = (float)Math.Atan2(-rotationMat.M31, Math.Sqrt(rotationMat.M32 * rotationMat.M32 + rotationMat.M33 * rotationMat.M33));
                angles[0] = (float)Math.Atan2(rotationMat.M21, rotationMat.M11);
                return angles;                
            }
            return new float[] { 0, 0, 0 };
        }

        /// <summary>
        /// Updates the float[6] array with kuka ABC angles in [X,Y,Z,A,B,C]. This only works when the rotation is less than 90 degrees 
        /// and will return iff angle in axis/angle representation is less than 90 degrees.
        /// </summary>
        /// <param name="rotation"></param> quaternion representing the rotation
        /// <returns></returns>
        public static void getKukaAngles(Quaternion rotation, ref float[] KukaAngleOut)
        {
            Vector3 axis = new Vector3();
            float angle = 0;
            getAxisAngle(rotation, ref axis, ref angle);
            if (angle < Math.PI / 2)
            {
                Matrix rotationMat = Matrix.CreateFromQuaternion(rotation);
                KukaAngleOut[5] = (float)Math.Atan2(rotationMat.M32, rotationMat.M33);
                KukaAngleOut[4] = (float)Math.Atan2(-rotationMat.M31, Math.Sqrt(rotationMat.M32 * rotationMat.M32 + rotationMat.M33 * rotationMat.M33));
                KukaAngleOut[3] = (float)Math.Atan2(rotationMat.M21, rotationMat.M11);
            }
            else
            {
                KukaAngleOut[5] = 0;
                KukaAngleOut[4] = 0;
                KukaAngleOut[3] = 0;
            }
        }

        /// <summary>
        /// Returns a double[6] array [X,Y,Z,A,B,C]. This only works when the rotation is less than 90 degrees 
        /// and will return iff angle in axis/angle representation is less than 90 degrees.
        /// </summary>
        /// <param name="pose"></param>
        /// <returns></returns>
        public static float[] getKukaAngles(Matrix pose)
        {
            Vector3 axis = new Vector3();
            float angle = 0;
            Vector3 scale = new Vector3();
            Quaternion rotation = new Quaternion();
            Vector3 translation = new Vector3();
            pose.Decompose(out scale, out rotation, out translation);
            getAxisAngle(rotation, ref axis, ref angle);
            if (angle < Math.PI / 2)
            {
                float[] kukaOut = new float[6];
                kukaOut[5] = (float)Math.Atan2(pose.M32, pose.M33);
                kukaOut[4] = (float)Math.Atan2(-pose.M31, Math.Sqrt(pose.M32 * pose.M32 + pose.M33 * pose.M33));
                kukaOut[3] = (float)Math.Atan2(pose.M21, pose.M11);
                kukaOut[2] = translation.Z;
                kukaOut[1] = translation.Y;
                kukaOut[0] = translation.Z;
                return kukaOut;
            }
            return new float[] { 0, 0, 0, 0, 0, 0 };
        }


        /// <summary>
        /// Returns a double[6] array [X,Y,Z,A,B,C]. This only works when the rotation is less than 90 degrees 
        /// and will return iff angle in axis/angle representation is less than 90 degrees.
        /// </summary>
        /// <param name="pose"></param>
        /// <returns></returns>
        public static void getKukaAngles(Matrix pose, ref double[] kukaOut)
        {
            Vector3 axis = new Vector3();
            float angle = 0;
            Vector3 scale = new Vector3();
            Quaternion rotation = new Quaternion();
            Vector3 translation = new Vector3();
            pose.Decompose(out scale, out rotation, out translation);
            getAxisAngle(rotation, ref axis, ref angle);
            if (angle < Math.PI / 2)
            {
                kukaOut[5] = Math.Atan2(pose.M32, pose.M33);
                kukaOut[4] = Math.Atan2(-pose.M31, Math.Sqrt(pose.M32 * pose.M32 + pose.M33 * pose.M33));
                kukaOut[3] = Math.Atan2(pose.M21, pose.M11);
                kukaOut[2] = (double)translation.Z;
                kukaOut[1] = (double)translation.Y;
                kukaOut[0] = (double)translation.X;
            }
            else { kukaOut = new double[] { 0, 0, 0, 0, 0, 0 }; }
        }

        /// <summary>
        /// Returns a double[6] array [X,Y,Z,A,B,C]. This only works when the rotation is less than 90 degrees 
        /// and will return iff angle in axis/angle representation is less than 90 degrees.
        /// </summary>
        /// <param name="pose"></param>
        /// <returns></returns>
        public static void getKukaAngles(Matrix pose, ref float[] KukaAngleOut)
        {
            Vector3 axis = new Vector3();
            float angle = 0;
            Vector3 scale = new Vector3();
            Quaternion rotation = new Quaternion();
            Vector3 translation = new Vector3();
            pose.Decompose(out scale, out rotation, out translation);
            getAxisAngle(rotation, ref axis, ref angle);
            if (angle < Math.PI / 2)
            {
                KukaAngleOut[5] = (float)Math.Atan2(pose.M32, pose.M33);
                KukaAngleOut[4] = (float)Math.Atan2(-pose.M31, Math.Sqrt(pose.M32 * pose.M32 + pose.M33 * pose.M33));
                KukaAngleOut[3] = (float)Math.Atan2(pose.M21, pose.M11);
                KukaAngleOut[2] = translation.Z;
                KukaAngleOut[1] = translation.Y;
                KukaAngleOut[0] = translation.Z;
            }
            else
            {
                KukaAngleOut[5] = 0;
                KukaAngleOut[4] = 0;
                KukaAngleOut[3] = 0;
                KukaAngleOut[2] = translation.Z;
                KukaAngleOut[1] = translation.Y;
                KukaAngleOut[0] = translation.Z;
            }
        }


    }



}
