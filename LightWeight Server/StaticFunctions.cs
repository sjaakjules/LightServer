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

        /// <summary>
        /// Returns the double array using Carinal dictionary keys.
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
        /// Returns the double array using Axis dictionary keys.
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
        /// <param name="poses"></param> A double[6] array {X, Y, Z, A, B, C} 
        /// <returns></returns>
        public static Quaternion MakeQuaternion(double[] poses)
        {
            Matrix Rz = Matrix.CreateRotationZ((float)poses[3]);
            Matrix Ry = Matrix.CreateRotationY((float)poses[4]);
            Matrix Rx = Matrix.CreateRotationX((float)poses[5]);
            Matrix Rotation = Matrix.Multiply(Matrix.Multiply(Rz, Ry), Rx);
            return Quaternion.CreateFromRotationMatrix(Rotation);
        }

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
            return cardinalKeys[Index];
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

        public static Quaternion getQuaternionFromKuka(float A, float B, float C)
        {
            return Quaternion.CreateFromRotationMatrix((Matrix.CreateRotationZ(A) * Matrix.CreateRotationY(B)) * Matrix.CreateRotationX(C));
        }

        public static float[] getKukaAnglesFromQuaternion(Quaternion rotation)
        {
            Matrix rotationMat = Matrix.CreateFromQuaternion(rotation);
            float[] angles = new float[3];
            angles[2] = (float)Math.Atan2(rotationMat.M32, rotationMat.M33);
            angles[1] = (float)Math.Atan2(-rotationMat.M31, Math.Sqrt(rotationMat.M32 * rotationMat.M32 + rotationMat.M33 * rotationMat.M33));
            angles[0] = (float)Math.Atan2(rotationMat.M21, rotationMat.M11);
            return angles;
        }



    }



}
