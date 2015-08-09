using Microsoft.Xna.Framework;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace LightWeight_Server
{
    public static class SF
    {

        static String[] cardinalKeys = new String[] { "X", "Y", "Z", "A", "B", "C" };
        static String[] axisKeys = new String[] { "A1", "A2", "A3", "A4", "A5", "A6" };
        static String[] rotationKeys = new String[] { "X1", "X2", "X3", "Y1", "Y2", "Y3", "Z1", "Z2", "Z3" };

        static Matrix M(Matrix mat1, Matrix mat2)
        {
            return Matrix.Transpose(Matrix.Transpose(mat1) * Matrix.Transpose(mat2));
        }

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



        /*
         * 
         * 
         * 
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
         * 
         * 
         */

        /// <summary>
        /// Creates a quaternion from Kuka coordinates, ABC in degrees
        /// </summary>
        /// <param name="pose"></param> A double[6] array {X, Y, Z, A, B, C} 
        /// <returns></returns>
        public static Quaternion MakeQuaternionFromKuka(double[] pose)
        {
            Quaternion Rz = Quaternion.CreateFromRotationMatrix(Matrix.CreateRotationZ((float)(pose[3] * Math.PI / 180)));
            Quaternion Ry = Quaternion.CreateFromRotationMatrix(Matrix.CreateRotationY((float)(pose[4] * Math.PI / 180)));
            Quaternion Rx = Quaternion.CreateFromRotationMatrix(Matrix.CreateRotationX((float)(pose[5] * Math.PI / 180)));
            return (Rz * Ry * Rx);
        }

        public static Matrix MakeMatrixFromKuka(double[] pose)
        {
            Matrix Rz = Matrix.CreateRotationZ((float)(pose[3] * Math.PI / 180));
            Matrix Ry = Matrix.CreateRotationY((float)(pose[4] * Math.PI / 180));
            Matrix Rx = Matrix.CreateRotationX((float)(pose[5] * Math.PI / 180));
            Matrix poseout = M(M(Rz, Ry), Rx);
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
            quaternion.Normalize();
            if (quaternion.W < 0)
            {
                quaternion = Quaternion.Negate(quaternion);
            }
            float angle = 2 * (float)Math.Acos(quaternion.W);
            float s = (float)Math.Sin(angle / 2);
            if (Math.Abs(s) < 1e-6)
            {
                outAxis.X = quaternion.X;
                outAxis.Y = quaternion.Y;
                outAxis.Z = quaternion.Z;
                outAxis.Normalize();
                if (quaternion.X != 0)
                {
                    if (quaternion.X < 0)
                    {
                        outAngle = (float)(2 * Math.Atan2(-(double)quaternion.X, (double)quaternion.W * -outAxis.X));
                    }
                    else
                    {
                        outAngle = (float)(2 * Math.Atan2((double)quaternion.X, (double)quaternion.W * outAxis.X));
                    }
                }
                else if (quaternion.Y != 0)
                {
                    if (quaternion.Y < 0)
                    {
                        outAngle = (float)(2 * Math.Atan2(-(double)quaternion.Y, (double)quaternion.W * -outAxis.Y));
                    }
                    else
                    {
                        outAngle = (float)(2 * Math.Atan2((double)quaternion.Y, (double)quaternion.W * outAxis.Y));
                    }
                }
                else if (quaternion.Z != 0)
                {
                    if (quaternion.Z < 0)
                    {
                        outAngle = (float)(2 * Math.Atan2(-(double)quaternion.Z, (double)quaternion.W * -outAxis.Z));
                    }
                    else
                    {
                        outAngle = (float)(2 * Math.Atan2((double)quaternion.Z, (double)quaternion.W * outAxis.Z));
                    }
                }
                else
                {
                    outAngle = 0;
                }

            }
            else
            {
                Vector3 axis = new Vector3(quaternion.X / s, quaternion.Y / s, quaternion.Z / s);
                axis.Normalize();
                s = quaternion.X == 0 ? s : quaternion.X / axis.X;
                s = quaternion.Y == 0 ? s : quaternion.Y / axis.Y;
                s = quaternion.Z == 0 ? s : quaternion.Z / axis.Z;
                outAxis = new Vector3(quaternion.X / s, quaternion.Y / s, quaternion.Z / s);
                outAxis.Normalize();
                if (quaternion.X != 0)
                {
                    if (quaternion.X < 0)
                    {
                        outAngle = (float)(2 * Math.Atan2(-(double)quaternion.X, (double)quaternion.W * -outAxis.X));
                    }
                    else
                    {
                        outAngle = (float)(2 * Math.Atan2((double)quaternion.X, (double)quaternion.W * outAxis.X));
                    }
                }
                else if (quaternion.Y != 0)
                {
                    if (quaternion.Y < 0)
                    {
                        outAngle = (float)(2 * Math.Atan2(-(double)quaternion.Y, (double)quaternion.W * -outAxis.Y));
                    }
                    else
                    {
                        outAngle = (float)(2 * Math.Atan2((double)quaternion.Y, (double)quaternion.W * outAxis.Y));
                    }
                }
                else if (quaternion.Z != 0)
                {
                    if (quaternion.Z < 0)
                    {
                        outAngle = (float)(2 * Math.Atan2(-(double)quaternion.Z, (double)quaternion.W * -outAxis.Z));
                    }
                    else
                    {
                        outAngle = (float)(2 * Math.Atan2((double)quaternion.Z, (double)quaternion.W * outAxis.Z));
                    }
                }
                else
                {
                    outAngle = 0;
                }
            }
        }


        /*
         * 
         * 
         * 
        public static Quaternion MakeQuaternionFromKuka(float A, float B, float C)
        {
            Matrix Rz = Matrix.CreateRotationZ(MathHelper.ToRadians(A));
            Matrix Ry = Matrix.CreateRotationY(MathHelper.ToRadians(B));
            Matrix Rx = Matrix.CreateRotationX(MathHelper.ToRadians(C));
            return Quaternion.CreateFromRotationMatrix(M(M(Rz, Ry), Rx));
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
            if (Math.Abs(angle) < Math.PI / 2)
            {
                Matrix rotationMat = Matrix.CreateFromQuaternion(rotation);
                float[] angles = new float[3];
                angles[2] = (float)(180.0f * Math.Atan2(rotationMat.M32, rotationMat.M33) / Math.PI);
                angles[1] = (float)(180.0f * Math.Atan2(-rotationMat.M31, Math.Sqrt(rotationMat.M32 * rotationMat.M32 + rotationMat.M33 * rotationMat.M33)) / Math.PI);
                angles[0] = (float)(180.0f * Math.Atan2(rotationMat.M21, rotationMat.M11) / Math.PI);
                return angles;                
            }
            return new float[] { 0, 0, 0 };
        }
         * 
         */

        /// <summary>
        /// Updates the float[6] array with kuka ABC angles in [X,Y,Z,A,B,C]. This only works when the rotation is less than 90 degrees 
        /// and will return iff angle in axis/angle representation is less than 90 degrees.
        /// </summary>
        /// <param name="rotation"></param> quaternion representing the rotation
        /// <returns></returns>
        public static void getKukaAngles(Quaternion rotation, ref float[] KukaAngleOut)
        {
            float A = 0;
            float B = 0;
            float C = 0;

            Matrix rotationMat = Matrix.CreateFromQuaternion(rotation);
            rotationMat = Matrix.Transpose(rotationMat);

            B = (float)Math.Atan2(-rotationMat.M31, Math.Sqrt(rotationMat.M32 * rotationMat.M32 + rotationMat.M33 * rotationMat.M33));

            if (Math.Abs(Math.Abs(B) - Math.PI / 2) < 1e-6)
            {
                // Gimbal lock situation! A and C form a line of infinate solutions.
                C = 0;// (float)Math.PI / 5f;
                A = (float)Math.Atan2(Math.Sign(B) * rotationMat.M23, Math.Sign(B) * rotationMat.M13) + Math.Sign(B) * C;
            }
            else
            {
                A = (float)Math.Atan2(rotationMat.M21, rotationMat.M11);
                C = (float)Math.Atan2(rotationMat.M32, rotationMat.M33);
            }

            KukaAngleOut[3] = MathHelper.ToDegrees(A);
            KukaAngleOut[4] = MathHelper.ToDegrees(B);
            KukaAngleOut[5] = MathHelper.ToDegrees(C);
        }
        /*
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
                kukaOut[5] = ((float)Math.Atan2(pose.M32, pose.M33));
                kukaOut[4] = ((float)Math.Atan2(-pose.M31, Math.Sqrt(pose.M32 * pose.M32 + pose.M33 * pose.M33)));
                kukaOut[3] = ((float)Math.Atan2(pose.M21, pose.M11));
                kukaOut[2] = translation.Z;
                kukaOut[1] = translation.Y;
                kukaOut[0] = translation.Z;
                return kukaOut;
            }
            return new float[] { 0, 0, 0, 0, 0, 0 };
        }

         * 
         */

        /// <summary>
        /// Returns a double[6] array [X,Y,Z,A,B,C]. This only works when the rotation is less than 90 degrees 
        /// and will return iff angle in axis/angle representation is less than 90 degrees.
        /// </summary>
        /// <param name="pose"></param>
        /// <returns></returns>
        public static void getKukaAngles(Matrix pose, ref double[] kukaOut)
        {
            float A = 0;
            float B = 0;
            float C = 0;
            Matrix rotationMat = Matrix.Transpose(pose);

            B = (float)Math.Atan2(-rotationMat.M31, Math.Sqrt(rotationMat.M32 * rotationMat.M32 + rotationMat.M33 * rotationMat.M33));

            if (Math.Abs(Math.Abs(B) - Math.PI / 2) < 1e-6)
            {
                // Gimbal lock situation! A and C form a line of infinate solutions.
                C = 0;// (float)Math.PI / 5f;
                A = (float)Math.Atan2(Math.Sign(B) * rotationMat.M23, Math.Sign(B) * rotationMat.M13) + Math.Sign(B) * C;
            }
            else
            {
                A = (float)Math.Atan2(rotationMat.M21, rotationMat.M11);
                C = (float)Math.Atan2(rotationMat.M32, rotationMat.M33);
            }

            kukaOut[0] = (double)pose.Translation.X;
            kukaOut[1] = (double)pose.Translation.Y;
            kukaOut[2] = (double)pose.Translation.Z;
            kukaOut[3] = MathHelper.ToDegrees(A);
            kukaOut[4] = MathHelper.ToDegrees(B);
            kukaOut[5] = MathHelper.ToDegrees(C);
            
        }


        /*

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
                KukaAngleOut[5] = MathHelper.ToDegrees((float)Math.Atan2(pose.M32, pose.M33));
                KukaAngleOut[4] = MathHelper.ToDegrees((float)Math.Atan2(-pose.M31, Math.Sqrt(pose.M32 * pose.M32 + pose.M33 * pose.M33)));
                KukaAngleOut[3] = MathHelper.ToDegrees((float)Math.Atan2(pose.M21, pose.M11));
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


         * 
         */

    }

    // CONVENTIONS:
    // Identity matrix. Forwards =   0, 0,-1
    //                  Down     =   0,-1, 0
    //                  Left     =  -1, 0, 0
    ///
    // Matrix are row basis, where litrature is column basis!
    // This requires transpose before and after any multiplication
}
