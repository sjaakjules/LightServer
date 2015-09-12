using Microsoft.Xna.Framework;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace LightWeight_Server
{
    public struct TimeCoordinate
    {
        public readonly long Ipoc;
        public readonly int LocalIpoc;
        public double x, y, z, a, b, c;
        public float angle;
        Quaternion _Orientation;
        public Vector3 axis;


        public TimeCoordinate(double x, double y, double z, double a, double b, double c, long ipoc)
        {
            this.x = x;
            this.y = y;
            this.z = z;
            this.a = a;
            this.b = b;
            this.c = c;
            this.Ipoc = ipoc;
            this.LocalIpoc = Environment.TickCount;
            _Orientation = SF.MakeQuaternionFromKuka(a, b, c);
            SF.getAxisAngle(_Orientation, out axis, out angle);
        }

        public TimeCoordinate(double x, double y, double z, Quaternion Orientation, long ipoc)
        {
            this.x = x;
            this.y = y;
            this.z = z;
            this.a = 0;
            this.b = 0;
            this.c = 0;
            _Orientation = new Quaternion(Orientation.X, Orientation.Y, Orientation.Z, Orientation.W);
            SF.getAxisAngle(Orientation, out axis, out angle);
            this.Ipoc = ipoc;
            this.LocalIpoc = Environment.TickCount;
        }

        public TimeCoordinate(double x, double y, double z, Vector3 axis, float angle, long ipoc)
        {
            this.x = x;
            this.y = y;
            this.z = z;
            this.a = 0;
            this.b = 0;
            this.c = 0;
            this.axis = axis;
            this.angle = angle;
            this.Ipoc = ipoc;
            this.LocalIpoc = Environment.TickCount;

            _Orientation = Quaternion.CreateFromAxisAngle(axis,angle);

        }

        public Vector3 Translation
        {
            get { return new Vector3((float)x, (float)y, (float)z); }
            set { x = value.X; y = value.Y; z = value.Z; }
        }
        public Quaternion Orientation
        {
            get { return _Orientation; }
            set
            {
                _Orientation = value;
                SF.getAxisAngle(value, out axis, out angle);
            }
        }

        public static TimeCoordinate operator +(TimeCoordinate Pose1, TimeCoordinate Pose2)
        {
            return new TimeCoordinate(Pose1.x + Pose2.x, Pose1.y + Pose2.y, Pose1.z + Pose2.z, Pose1.axis + Pose2.axis, Pose1.angle + Pose2.angle,Pose2.Ipoc);
        }

        public static TimeCoordinate operator /(TimeCoordinate Pose1, float Value)
        {
            return new TimeCoordinate(Pose1.x / Value, Pose1.y / Value, Pose1.z / Value, Vector3.Divide(Pose1.axis, Value), Pose1.angle / Value, Pose1.Ipoc);
        }
        
        public TimeCoordinate getRateOfChange(TimeCoordinate pose1)
        {
            Vector3 changeAxis;
            float changeAngle;
            if (this.Ipoc < pose1.Ipoc)
            {
                double delTime = (pose1.Ipoc - this.Ipoc) / TimeSpan.TicksPerMillisecond;
                Quaternion changeOrientation = Quaternion.Inverse(this._Orientation) * pose1._Orientation;
                SF.getAxisAngle(changeOrientation, out changeAxis, out changeAngle);
                return new TimeCoordinate(  (pose1.x - this.x) / delTime,
                                            (pose1.y - this.y) / delTime,
                                            (pose1.z - this.z) / delTime,
                                            changeAxis,
                                            changeAngle / (float)delTime,
                                            pose1.Ipoc);
            }
            else
            {
                double delTime = (this.Ipoc - pose1.Ipoc) / TimeSpan.TicksPerMillisecond;
                Quaternion changeOrientation = Quaternion.Inverse(pose1._Orientation) * this._Orientation;
                SF.getAxisAngle(changeOrientation, out changeAxis, out changeAngle);
                return new TimeCoordinate(  (pose1.x - this.x) / delTime,
                                            (this.y - pose1.y) / delTime,
                                            (this.z - pose1.z) / delTime,
                                            changeAxis,
                                            changeAngle / (float)delTime,
                                            this.Ipoc);
            }
        }

        public float OrientationDisplacement(TimeCoordinate pose1)
        {
            Quaternion tempOrientation = this._Orientation * Quaternion.Inverse(pose1._Orientation);
            Vector3 axis;
            float angle;
            SF.getAxisAngle(tempOrientation, out axis, out angle);
            return angle;
        }
    }


    public class FixedSizedQueue<T> : ConcurrentQueue<T>
    {
        private readonly object syncObject = new object();

        public int Size { get; private set; }

        public FixedSizedQueue(int size)
        {
            Size = size;
        }

        public new void Enqueue(T obj)
        {
            base.Enqueue(obj);
            lock (syncObject)
            {
                while (base.Count > Size)
                {
                    T outObj;
                    base.TryDequeue(out outObj);
                }
            }
        }

        public T LastElement
        {

            get
            {
                lock (syncObject)
                {
                    T[] list = base.ToArray();
                    return list[list.Length - 1];
                }
            }
        }

    }

    public static class SF
    {

        public static TimeCoordinate AverageRateOfChange(TimeCoordinate[] list)
        {
            TimeCoordinate averageRate = new TimeCoordinate(0, 0, 0, Vector3.Zero, 0, list[list.Length - 1].Ipoc);

            for (int i = 1; i < list.Length; i++)
            {
                averageRate += list[i - 1].getRateOfChange(list[i]);
            }
            return averageRate/(float)(list.Length-1);
        }


        static String[] cardinalKeys = new String[] { "X", "Y", "Z", "A", "B", "C" };
        static String[] axisKeys = new String[] { "A1", "A2", "A3", "A4", "A5", "A6" };
        static String[] rotationKeys = new String[] { "X1", "X2", "X3", "Y1", "Y2", "Y3", "Z1", "Z2", "Z3" };

        static Matrix M(Matrix mat1, Matrix mat2)
        {
            return Matrix.Transpose(Matrix.Transpose(mat1) * Matrix.Transpose(mat2));
        }

        static Matrix R(Matrix Rz, Matrix Ry, Matrix Rx)
        {
            return M(M(Rz,Ry),Rx);
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

        public static Quaternion MakeQuaternionFromKuka(double A, double B, double C)
        {
            Quaternion Rz = Quaternion.CreateFromRotationMatrix(Matrix.CreateRotationZ((float)(A * Math.PI / 180)));
            Quaternion Ry = Quaternion.CreateFromRotationMatrix(Matrix.CreateRotationY((float)(B * Math.PI / 180)));
            Quaternion Rx = Quaternion.CreateFromRotationMatrix(Matrix.CreateRotationX((float)(C * Math.PI / 180)));
            return (Rz * Ry * Rx);
        }

        public static Matrix MakeMatrixFromKuka(double[] pose)
        {
            Matrix Rz = Matrix.CreateRotationZ((float)(pose[3] * Math.PI / 180));
            Matrix Ry = Matrix.CreateRotationY((float)(pose[4] * Math.PI / 180));
            Matrix Rx = Matrix.CreateRotationX((float)(pose[5] * Math.PI / 180));
            Matrix poseout = R(Rz, Ry, Rx);
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


        public static void getAxisAngle(Quaternion quaternion, out Vector3 outAxis, out float outAngle)
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


    }

    // CONVENTIONS:
    // Identity matrix. Forwards =   0, 0,-1
    //                  Down     =   0,-1, 0
    //                  Left     =  -1, 0, 0
    ///
    // Matrix are row basis, where litrature is column basis!
    // This requires transpose before and after any multiplication
}
