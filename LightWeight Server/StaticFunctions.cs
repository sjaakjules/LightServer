using Microsoft.Xna.Framework;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace LightWeight_Server
{

    public struct Pose
    {
        double _x, _y, _z, _v;
        float _angle;
        Quaternion _Orientation;
        Vector3 _axis;
        float[] _kukaValues;

        public Pose(string[] newPose)
        {
            double.TryParse(newPose[0], out _v);
            double.TryParse(newPose[0], out _x);
            double.TryParse(newPose[0], out _y);
            double.TryParse(newPose[0], out _z);
            if (newPose.Length == 7)
            {
                
            }
        }

        public Pose(string[] newPose, Quaternion DesiredOrientation)
        {
            if (newPose.Length == 4)
            {

            }
        }

        public Pose(Matrix Pose) : this(Quaternion.CreateFromRotationMatrix(Pose), Pose.Translation) { }

        public Pose(Quaternion Orientation, Vector3 Position)
        {
            this._v = 0;
            this._x = Position.X;
            this._y = Position.Y;
            this._z = Position.Z;
            this._Orientation = Orientation;
            SF.getAxisAngle(Orientation, out _axis, out _angle);
            _kukaValues = new float[6];
            SF.getKukaAngles(Orientation, ref _kukaValues);
            _kukaValues[0] = (float)_x;
            _kukaValues[1] = (float)_y;
            _kukaValues[2] = (float)_z;
        }

        public Pose(TimeCoordinate Pose) : this(Pose.Orientation, Pose.Translation) { }


        public Vector3 Translation
        {
            get { return new Vector3((float)_x, (float)_y, (float)_z); }
            set { _x = value.X; _y = value.Y; _z = value.Z; }
        }

        public Quaternion Orientation
        {
            get { return _Orientation; }
            set
            {
                _Orientation = value;
                SF.getAxisAngle(value, out _axis, out _angle);
            }
        }

        public float[] kukaValues
        {
            get { return _kukaValues; }
        }

        public Vector3 zAxis
        {
            get { return this * new Vector3(0, 0, 1); }
        }

        public Vector3 Velocity
        {
            get { return _axis * _angle; }
        }

        public Vector3 axis
        {
            get { return _axis; }
        }

        public float angle
        {
            get { return _angle; }
        }

        public static Pose inverse(Pose pose)
        {
            return new Pose(Quaternion.Inverse(pose.Orientation), -Vector3.Transform(pose.Translation, Quaternion.Inverse(pose.Orientation)));
        }

        public Pose invert
        {
            get { return new Pose(Quaternion.Inverse(this.Orientation), -Vector3.Transform(this.Translation, Quaternion.Inverse(this.Orientation))); }
        }

        public static Vector3 operator *(Pose Pose, Vector3 Position)
        {
            Matrix PoseM = Matrix.Transpose(Matrix.CreateFromQuaternion(Pose._Orientation));
            return new Vector3( (float)Pose._x + PoseM.M11 * Position.X + PoseM.M12 * Position.Y + PoseM.M13 * Position.Z,
                                (float)Pose._y + PoseM.M21 * Position.X + PoseM.M22 * Position.Y + PoseM.M23 * Position.Z,
                                (float)Pose._z + PoseM.M31 * Position.X + PoseM.M32 * Position.Y + PoseM.M33 * Position.Z);
        }

        public static Pose operator *(Pose Pose1, Pose Pose2)
        {
            return new Pose(Pose1.Orientation * Pose2.Orientation, Pose1 * Pose2.Translation);
        }

    }

    public struct TimeCoordinate
    {
        public readonly long Ipoc;
        public readonly int LocalIpoc;
        public double x, y, z, a, b, c;
        public float angle;
        Quaternion _Orientation;
        public Vector3 axis;
        double[] _kukaValues;

        public TimeCoordinate(double x, double y, double z, double a, double b, double c, long ipoc)
        {
            this.x = x;
            this.y = y;
            this.z = z;
            this.a = a;
            this.b = b;
            this.c = c;
            _kukaValues = new double[] { x, y, z, a, b, c };
            _Orientation = SF.MakeQuaternionFromKuka(a, b, c);
            SF.getAxisAngle(_Orientation, out axis, out angle);
            this.Ipoc = ipoc;
            this.LocalIpoc = Environment.TickCount;
        }

        public TimeCoordinate(Vector3 Position, Quaternion Orientation, long ipoc)
        {
            this.x = Position.X;
            this.y = Position.Y;
            this.z = Position.Z;
            Vector3 angles = SF.getKukaAngles(Orientation);
            this.a = angles.X;
            this.b = angles.Y;
            this.c = angles.Z;
            _kukaValues = new double[] { x, y, z, a, b, c };
            _Orientation = new Quaternion(Orientation.X, Orientation.Y, Orientation.Z, Orientation.W);
            SF.getAxisAngle(Orientation, out axis, out angle);
            this.Ipoc = ipoc;
            this.LocalIpoc = Environment.TickCount;
        }

        public TimeCoordinate(Pose Pose, long ipoc) : this(Pose.Translation, Pose.Orientation, ipoc) { }

        public TimeCoordinate(double x, double y, double z, Quaternion Orientation, long ipoc) : this(new Vector3((float)x, (float)y, (float)z), Orientation, ipoc) { }

        public TimeCoordinate(double x, double y, double z, Vector3 axis, float angle, long ipoc) : this(new Vector3((float)x, (float)y, (float)z), Quaternion.CreateFromAxisAngle(axis, angle), ipoc) { }

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
                KukaAngles = SF.getKukaAngles(Orientation);
            }
        }

        public Vector3 KukaAngles
        {
            get { return new Vector3((float)a, (float)b, (float)c); }
            set
            {
                this.a = value.X;
                this.b = value.Y;
                this.c = value.Z;
            }
        }

        public double[] kukaValues
        {
            get { return _kukaValues; }
        }

        public Pose Pose
        {
            get { return new Pose(Orientation, Translation); }
            set
            {
                Orientation = value.Orientation;
                Translation = value.Translation;
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
                double delTime = 1.0 * (pose1.Ipoc - this.Ipoc) / TimeSpan.TicksPerMillisecond;
                if (delTime == 0)
                {
                    delTime = 1;
                }
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
                double delTime = 1.0 * (this.Ipoc - pose1.Ipoc) / TimeSpan.TicksPerMillisecond;
                if (delTime == 0)
                {
                    delTime = 1;
                }
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

        public void AxisAngle(Vector3 newAxis, float newAngle)
        {
            this.axis = newAxis;
            this.angle = newAngle;
            this._Orientation = Quaternion.CreateFromAxisAngle(axis, angle);
        }

        /// <summary>
        /// Finds the angle in radians to transform paseB towards this pose.
        /// </summary>
        /// <param name="poseB"></param>
        /// <returns></returns>
        public float OrientationDisplacementBtoA(TimeCoordinate poseB)
        {
            Quaternion tempOrientation = this._Orientation * Quaternion.Inverse(poseB.Orientation);
            Vector3 axis;
            float angle;
            SF.getAxisAngle(tempOrientation, out axis, out angle);
            return angle;
        }

        /// <summary>
        /// Finds the angle in radians to transform this pose towards paseB.
        /// </summary>
        /// <param name="poseB"></param>
        /// <returns></returns>
        public float OrientationDisplacementAtoB(TimeCoordinate poseB)
        {
            Quaternion tempOrientation = Quaternion.Inverse(this._Orientation) * poseB.Orientation;
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
        public static Matrix MatMultiply(Matrix M1, Matrix M2)
        {
            return new Matrix(  M1.M11 * M2.M11 + M1.M21 * M2.M12 + M1.M31 * M2.M13 + M1.M41 * M2.M14, M1.M12 * M2.M11 + M1.M22 * M2.M12 + M1.M32 * M2.M13 + M1.M42 * M2.M14, M1.M13 * M2.M11 + M1.M23 * M2.M12 + M1.M33 * M2.M13 + M1.M43 * M2.M14, M1.M14 * M2.M11 + M1.M24 * M2.M12 + M1.M34 * M2.M13 + M1.M44 * M2.M14,
                                M1.M11 * M2.M21 + M1.M21 * M2.M22 + M1.M31 * M2.M23 + M1.M41 * M2.M24, M1.M12 * M2.M21 + M1.M22 * M2.M22 + M1.M32 * M2.M23 + M1.M42 * M2.M24, M1.M13 * M2.M21 + M1.M23 * M2.M22 + M1.M33 * M2.M23 + M1.M43 * M2.M24, M1.M14 * M2.M21 + M1.M24 * M2.M22 + M1.M34 * M2.M23 + M1.M44 * M2.M24,
                                M1.M11 * M2.M31 + M1.M21 * M2.M32 + M1.M31 * M2.M33 + M1.M41 * M2.M34, M1.M12 * M2.M31 + M1.M22 * M2.M32 + M1.M32 * M2.M33 + M1.M42 * M2.M34, M1.M13 * M2.M31 + M1.M23 * M2.M32 + M1.M33 * M2.M33 + M1.M43 * M2.M34, M1.M14 * M2.M31 + M1.M24 * M2.M32 + M1.M34 * M2.M33 + M1.M44 * M2.M34,
                                M1.M11 * M2.M41 + M1.M21 * M2.M42 + M1.M31 * M2.M43 + M1.M41 * M2.M44, M1.M12 * M2.M41 + M1.M22 * M2.M42 + M1.M32 * M2.M43 + M1.M42 * M2.M44, M1.M13 * M2.M41 + M1.M23 * M2.M42 + M1.M33 * M2.M43 + M1.M43 * M2.M44, M1.M14 * M2.M41 + M1.M24 * M2.M42 + M1.M34 * M2.M43 + M1.M44 * M2.M44); 
                                
        }

        public static double[] multiplyJacobian(double[,] Jacobian, double[] taskVelocity)
        {
            // TODO basic matrix multiplication
            return new double[] { 0, 0, 0, 0, 0, 0 };
        }

        public static Vector3 getOrientationError(Matrix reference, Matrix measured)
        {
            return Vector3.Multiply((Vector3.Cross(getAxisfromMatrix(measured, 1), getAxisfromMatrix(reference, 1)) + Vector3.Cross(getAxisfromMatrix(measured, 2), getAxisfromMatrix(reference, 2)) + Vector3.Cross(getAxisfromMatrix(measured, 3), getAxisfromMatrix(reference, 3))), 0.5f);
        }

        static Vector3 getAxisfromMatrix(Matrix rotation, int column)
        {
            if (column == 1)
            {
                return new Vector3(rotation.M11, rotation.M12, rotation.M13);
            }
            if (column == 2)
            {
                return new Vector3(rotation.M21, rotation.M22, rotation.M23);
            }
            if (column == 3)
            {
                return new Vector3(rotation.M31, rotation.M32, rotation.M33);
            }
            return Vector3.Zero;
        }

        public static TimeCoordinate AverageRateOfChange(TimeCoordinate[] list)
        {
            TimeCoordinate averageRate = list[0].getRateOfChange(list[1]); ;

            for (int i = 2; i < list.Length; i++)
            {
                averageRate += list[i - 1].getRateOfChange(list[i]);
            }
            return averageRate/(float)(list.Length-1);
        }


        public static readonly String[] cardinalKeys = new String[] { "X", "Y", "Z", "A", "B", "C" };
        public static readonly String[] axisKeys = new String[] { "A1", "A2", "A3", "A4", "A5", "A6" };
        public static readonly String[] rotationKeys = new String[] { "X1", "X2", "X3", "Y1", "Y2", "Y3", "Z1", "Z2", "Z3" };
        public static readonly String[] axisVecotrKeys = new String[] { "XX", "XY", "XZ", "ZX", "ZY", "ZZ"};

        public static Matrix M(Matrix mat1, Matrix mat2)
        {
            return Matrix.Transpose(Matrix.Transpose(mat1) * Matrix.Transpose(mat2));
        }

        static Matrix R(Matrix Rz, Matrix Ry, Matrix Rx)
        {
            return M(M(Rz,Ry),Rx);
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

        /*
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
        */

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
                if (quaternion.Z == double.NaN)
                {
                    outAxis.Z = 1;
                }
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
                    outAxis = new Vector3(0,0,1);
                }

            }
            else
            {
                Vector3 axis = new Vector3(quaternion.X / s, quaternion.Y / s, quaternion.Z / s);
                axis.Normalize();
                s = quaternion.X == 0 ? s : quaternion.X / axis.X;
                s = quaternion.Y == 0 ? s : quaternion.Y / axis.Y;
                s = quaternion.Z == 0 ? s : quaternion.Z / axis.Z;

                if (float.IsNaN(quaternion.Z))
                {
                    outAxis.Z = 1;
                }
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



        public static void getKukaAngles(TimeCoordinate Pose, ref float[] KukaAngleOut)
        {
            getKukaAngles(Pose.Orientation, ref KukaAngleOut);
            KukaAngleOut[0] = (float)Pose.x;
            KukaAngleOut[1] = (float)Pose.y;
            KukaAngleOut[2] = (float)Pose.z;
        }

        public static Vector3 getKukaAngles(Quaternion rotation)
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

            return new Vector3(MathHelper.ToDegrees(A), MathHelper.ToDegrees(B), MathHelper.ToDegrees(C));
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
        public static void getKukaAngles(Matrix pose, ref float[] kukaOut)
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

            kukaOut[0] = pose.Translation.X;
            kukaOut[1] = pose.Translation.Y;
            kukaOut[2] = pose.Translation.Z;
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
