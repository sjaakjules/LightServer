using Microsoft.Xna.Framework;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace LightWeight_Server
{
    // CONVENTIONS of Matrix class:
    // Identity matrix. Forwards =   0, 0,-1
    //                  Down     =   0,-1, 0
    //                  Left     =  -1, 0, 0
    ///
    // Matrix are row basis, where litrature is column basis!
    // This requires transpose before and after any multiplication

    public struct Pose : IFormattable
    {
        double _x, _y, _z;
        float _angle;
        Quaternion _Orientation;
        Vector3 _axis;
        double[] _kukaValues;

        public static Pose Zero { get { return new Pose(Quaternion.Identity, Vector3.Zero); } }

        /// <summary>
        /// Loads a new Pose using a string with the infomation of position, velocity and orientation. Can handle partial information.
        /// </summary>
        /// <param name="newPose"></param>
        /// <param name="LastPose"></param>
        public Pose(string[] newPose, Pose LastPose)
        {
            int shift = newPose.Length % 3;
            // The following If statements catch error while trying to load positions, use last position and keep loading.
            // TODO: Add signals and more complex error handeling to send back to external server. Perhaps the string which was received for error checking?
            if (!double.TryParse(newPose[0 + shift], out _x))
            {
                _x = LastPose._x;
            }
            if (!double.TryParse(newPose[1 + shift], out _y))
            {
                _y = LastPose._y;
            }
            if (!double.TryParse(newPose[2 + shift], out _z))
            {
                _z = LastPose._z;
            }
            double[] Axis = new double[newPose.Length - (3 + shift)];
            bool loadedAxis = true;
            for (int i = 3 + shift; i < newPose.Length; i++)
            {
                if (!double.TryParse(newPose[i], out Axis[i - (3 + shift)]))
                {
                    loadedAxis = false;
                    break;
                }
            }
            if (loadedAxis)
            {
                // only Z axis loaded
                if (Axis.Length == 3)
                {
                    this._Orientation = SF.QfromZaxis(new Vector3((float)Axis[0], (float)Axis[1], (float)Axis[2]), LastPose.Orientation);
                    SF.getAxisAngle(_Orientation, out _axis, out _angle);
                    SF.getKukaAngles(_Orientation, out _kukaValues);
                    _kukaValues[0] = (float)_x;
                    _kukaValues[1] = (float)_y;
                    _kukaValues[2] = (float)_z;
                }
                    // Z axis and X axis loaded
                else if (Axis.Length == 6)
                {
                    this._Orientation = SF.QfromZX(new Vector3((float)Axis[0], (float)Axis[1], (float)Axis[2]), new Vector3((float)Axis[3], (float)Axis[4], (float)Axis[5]));
                    SF.getAxisAngle(_Orientation, out _axis, out _angle);
                    SF.getKukaAngles(_Orientation, out _kukaValues);
                    _kukaValues[0] = (float)_x;
                    _kukaValues[1] = (float)_y;
                    _kukaValues[2] = (float)_z;
                }
                    // No rotation loaded, position movement only
                else
                {
                    this._Orientation = LastPose.Orientation;
                    SF.getAxisAngle(LastPose.Orientation, out _axis, out _angle);
                    SF.getKukaAngles(LastPose.Orientation, out _kukaValues);
                    _kukaValues[0] = (float)_x;
                    _kukaValues[1] = (float)_y;
                    _kukaValues[2] = (float)_z;
                }
            }
                // Failed while trying to load rotaition, use last orientation and keep moving.
                // TODO: Add signals and more complex error handeling to send back to external server. Perhaps the string which was received for error checking?
            else
            {
                this._Orientation = LastPose.Orientation;
                SF.getAxisAngle(LastPose.Orientation, out _axis, out _angle);
                SF.getKukaAngles(LastPose.Orientation, out _kukaValues);
                _kukaValues[0] = (float)_x;
                _kukaValues[1] = (float)_y;
                _kukaValues[2] = (float)_z;
            }
        }
        
        public Pose(double[] KukaValues)
        {
            _kukaValues = new double[6];
            KukaValues.CopyTo(_kukaValues, 0);
            this._x = KukaValues[0];
            this._y = KukaValues[1];
            this._z = KukaValues[2];
            _Orientation = SF.MakeQuaternionFromKuka(KukaValues);
            SF.getAxisAngle(_Orientation, out _axis, out _angle);
        }

        public Pose(Matrix Pose) : this(Quaternion.CreateFromRotationMatrix(Pose), Pose.Translation) { }

        public Pose(Quaternion Orientation, Vector3 Position)
        {
            this._x = Position.X;
            this._y = Position.Y;
            this._z = Position.Z;
            this._Orientation = Orientation;
            SF.getAxisAngle(Orientation, out _axis, out _angle);
            SF.getKukaAngles(Orientation, out _kukaValues);
            _kukaValues[0] = (float)_x;
            _kukaValues[1] = (float)_y;
            _kukaValues[2] = (float)_z;
        }

        public Pose(TimeCoordinate Pose) : this(Pose.Orientation, Pose.Translation) { }


        public override int GetHashCode()
        {
            return ShiftAndWrap(_x.GetHashCode(), 2) ^ _y.GetHashCode();
        }

        public bool Equals(Pose pose2, double error)
        {
            if (Vector3.Distance(this.Translation, pose2.Translation) <= error)// && SF.isOrientationAligned(this.Orientation, pose2.Orientation, error))
            {
                return true;
            }
            return false;
        }

        private int ShiftAndWrap(int value, int positions)
        {
            positions = positions & 0x1F;

            // Save the existing bit pattern, but interpret it as an unsigned integer.
            uint number = BitConverter.ToUInt32(BitConverter.GetBytes(value), 0);
            // Preserve the bits to be discarded.
            uint wrapped = number >> (32 - positions);
            // Shift and wrap the discarded bits.
            return BitConverter.ToInt32(BitConverter.GetBytes((number << positions) | wrapped), 0);
        }

        public override bool Equals(object o)
        {
            if (Vector3.Distance(this.Translation, ((Pose)o).Translation) == 1e-6 && SF.isOrientationAligned(this.Orientation, ((Pose)o).Orientation, 1e-6))
            {
                return true;
            }
            return false;
        }

        public static bool operator ==(Pose pose1, Pose pose2)
        {
            if (Vector3.Distance( pose1.Translation,pose2.Translation) == 1e-6 && pose1.Orientation.Equals(pose2.Orientation))
            {
                return true;
            }
            return false;
        }

        public static bool operator !=(Pose pose1, Pose pose2)
        {
            if (Vector3.Distance(pose1.Translation, pose2.Translation) == 1e-6 && pose1.Orientation.Equals(pose2.Orientation))
            {
                return false;
            }
            return true;
        }
         

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

        public double[] kukaValues
        {
            get { return _kukaValues; }
        }

        public Vector3 zAxis
        {
            get { return Vector3.Transform(new Vector3(0, 0, 1),_Orientation); }
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

        public string ToString(string format, IFormatProvider formatProvider)
        {

            if (format == null) format = "G";

            if (formatProvider != null)
            {
                ICustomFormatter formatter = formatProvider.GetFormat(this.GetType()) as ICustomFormatter;
                if (formatter != null)
                    return formatter.Format(format, this, formatProvider);
            }

            switch (format)
            {

                case "data": return string.Format("{0},{1},{2},{3},{4},{5},{6}", this._x, this._y, this._z, this.angle, this.axis.X, this.axis.Y, this.axis.Z);

                case "Display": return string.Format("({0:0.000},{1:0.000},{2:0.000},{3:0.000},{4:0.000},{5:0.000})", this._x, this._y, this._z, this._kukaValues[3], this._kukaValues[4], this._kukaValues[5]);

                case "G":

                default: return String.Format("({0,5},{1,5},{2,5})", this._x, this._y, this._z);

            }

        }


        public override string ToString()
        {
            return ToString("G", null);
        }



        public string ToString(string format)
        {
            return ToString(format, null);
        }

        public string ToString(IFormatProvider formatProvider)
        {
            return ToString(null, formatProvider);
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
        
        public TimeCoordinate getRateOfChange(TimeCoordinate pose1, double framerate)
        {
            Vector3 changeAxis;
            float changeAngle;
            if (this.Ipoc < pose1.Ipoc)
            {
                Quaternion changeOrientation = Quaternion.Inverse(this._Orientation) * pose1._Orientation;
                SF.getAxisAngle(changeOrientation, out changeAxis, out changeAngle);
                return new TimeCoordinate((pose1.x - this.x) / framerate,
                                            (pose1.y - this.y) / framerate,
                                            (pose1.z - this.z) / framerate,
                                            Vector3.Transform(changeAxis, pose1._Orientation),
                                            changeAngle / (float)framerate,
                                            pose1.Ipoc);
            }
            else
            {
                Quaternion changeOrientation = Quaternion.Inverse(pose1._Orientation) * this._Orientation;
                SF.getAxisAngle(changeOrientation, out changeAxis, out changeAngle);
                return new TimeCoordinate((this.x - pose1.x) / framerate,
                                            (this.y - pose1.y) / framerate,
                                            (this.z - pose1.z) / framerate,
                                            Vector3.Transform(changeAxis, this._Orientation),
                                            changeAngle / (float)framerate,
                                            this.Ipoc);
            }
        }


        public TimeCoordinate getRateOfChange(TimeCoordinate pose1)
        {
            Vector3 changeAxis;
            float changeAngle;
            if (this.Ipoc < pose1.Ipoc)
            {
                double delTime = 1.0 * (pose1.Ipoc - this.Ipoc);
                if (delTime == 0)
                {
                    delTime = 1;
                }
                Quaternion changeOrientation = Quaternion.Inverse(this._Orientation) * pose1._Orientation;
                SF.getAxisAngle(changeOrientation, out changeAxis, out changeAngle);
                return new TimeCoordinate((pose1.x - this.x) / delTime,
                                            (pose1.y - this.y) / delTime,
                                            (pose1.z - this.z) / delTime,
                                            Vector3.Transform(changeAxis,this._Orientation),
                                            changeAngle / (float)delTime,
                                            pose1.Ipoc);
            }
            else
            {
                double delTime = 1.0 * (this.Ipoc - pose1.Ipoc) ;
                if (delTime == 0)
                {
                    delTime = 1;
                }
                Quaternion changeOrientation = Quaternion.Inverse(pose1._Orientation) * this._Orientation;
                SF.getAxisAngle(changeOrientation, out changeAxis, out changeAngle);
                return new TimeCoordinate((this.x - pose1.x) / delTime,
                                            (this.y - pose1.y) / delTime,
                                            (this.z - pose1.z) / delTime,
                                            Vector3.Transform(changeAxis, pose1._Orientation),
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

        public T[] ThreadSafeToArray
        {
            get
            {
                lock (syncObject)
                {
                    return base.ToArray();
                }
            }
        }

    }

    /// <summary>
    /// Exception class for Matrix class, derived from System.Exception
    /// </summary>
    public class MatrixException : Exception
    {
        public MatrixException()
            : base()
        { }

        public MatrixException(string Message)
            : base(Message)
        { }

        public MatrixException(string Message, Exception InnerException)
            : base(Message, InnerException)
        { }
    }

    /// <summary>
    /// Exception class for Kuka related functions, derived from System.Exception
    /// </summary>
    public class KukaException : Exception
    {
        public KukaException()
            : base()
        { }

        public KukaException(string Message)
            : base(Message)
        { }

        public KukaException(string Message, Exception InnerException)
            : base(Message, InnerException)
        { }
    }

    /// <summary>
    /// Exception class for general orientation and rotation related functions, derived from System.Exception
    /// </summary>
    public class OrientationException : Exception
    {
        public OrientationException()
            : base()
        { }

        public OrientationException(string Message)
            : base(Message)
        { }

        public OrientationException(string Message, Exception InnerException)
            : base(Message, InnerException)
        { }
    }

    /// <summary>
    /// Exception class for general orientation and rotation related functions, derived from System.Exception
    /// </summary>
    public class TrajectoryException : Exception
    {
        public TrajectoryException()
            : base()
        { }

        public TrajectoryException(string Message)
            : base(Message)
        { }

        public TrajectoryException(string Message, Exception InnerException)
            : base(Message, InnerException)
        { }
    }

    public static class SF
    {
        #region Printing Functions
        static string printDouble(double[] array)
        {
            StringBuilder printingstring = new StringBuilder();
            printingstring.Append(string.Format("{0}", array[0]));
            for (int i = 1; i < array.Length; i++)
            {
                printingstring.Append(string.Format(",{0}", array[i]));
            }
            return printingstring.ToString();
        }

        public static void updateDataFile(Pose refPos, Pose refVel, Pose actPos, Pose actVel, double time,double[] tipVel, double[] axisComand, StringBuilder tableRow)
        {
            tableRow.Append(string.Format("{0:0.0},{1:data},{2:data},{3:data},{4:data},{5},{6};", time, refPos, actPos, refVel, actVel, printDouble(tipVel), printDouble(axisComand)));
        }

        #endregion

        #region Matrix Functions

        public static double[] multiplyMatrix(double[,] M, double[] value)
        {
            int m_iRows = M.GetLength(0);
            int m_iCols = M.GetLength(1);
            if (m_iCols != value.Length)
                throw new MatrixException("Invalid Matrix specified, not correct size");
            double[] Mout = new double[value.Length];
            for (int i = 0; i < m_iRows; i++)
            {
                for (int j = 0; j < m_iCols; j++)
                {
                    Mout[i] += M[i, j] * value[j];
                }
            }
            return Mout;
        }

        public static double[,] multiplyMatrix(double[,] M, double[,] value)
        {
            int m_iRows = M.GetLength(0);
            int m_iCols = M.GetLength(1);
            int v_iRows = value.GetLength(0);
            int v_iCols = value.GetLength(1);
            if (m_iCols != v_iRows)
            {
                return multiplyMatrix(value, M);
                throw new MatrixException("Invalid Matrix specified, not correct size");
            }
            double[,] Mout = new double[m_iRows, v_iCols];
            for (int i = 0; i < m_iRows; i++)
            {
                for (int j = 0; j < v_iCols; j++)
                {
                    Mout[i, j] = dotProduct(getRow(M, i), getCol(value, j));
                }
            }
            return Mout;
        }

        public static double[] getRow(double[,] M, int row)
        {
            int m_iRows = M.GetLength(0);
            int m_iCols = M.GetLength(1);
            if (m_iRows < row)
                throw new MatrixException("Invalid Matrix specified, not correct size");
            double[] Rout = new double[m_iCols];
            for (int i = 0; i < m_iCols; i++)
            {
                Rout[i] = M[row, i];
            }
            return Rout;
        }

        public static double[] getCol(double[,] M, int col)
        {
            int m_iRows = M.GetLength(0);
            int m_iCols = M.GetLength(1);
            if (m_iCols < col)
                throw new MatrixException("Invalid Matrix specified, not correct size");
            double[] Cout = new double[m_iRows];
            for (int i = 0; i < m_iRows; i++)
            {
                Cout[i] = M[i, col];
            }
            return Cout;
        }

        public static double dotProduct(double[] V1, double[] V2)
        {
            int m_iRows = V1.Length;
            int m_iCols = V2.Length;
            if (m_iCols != m_iRows)
                throw new MatrixException("Invalid Matrix specified, not correct size");
            double product = 0;
            for (int i = 0; i < m_iRows; i++)
            {
                product += V1[i] * V2[i];
            }
            return product;
        }

        #endregion

        #region Orientation Functions

        public static Quaternion QfromZX(Vector3 zAxis, Vector3 xAxis)
        {
            xAxis.Normalize();
            zAxis.Normalize();
            Vector3 yAxis = Vector3.Cross(zAxis, xAxis);
            yAxis.Normalize();
            return Quaternion.CreateFromRotationMatrix(new Matrix(xAxis.X, xAxis.Y, xAxis.Z, 0, yAxis.X, yAxis.Y, yAxis.Z, 0, zAxis.X, zAxis.Y, zAxis.Z, 0, 0, 0, 0, 1));
        }

        public static Quaternion QfromZaxis(Vector3 Zaxis, Quaternion _currentOrientation)
        {
            Matrix _currentPose = Matrix.CreateFromQuaternion(_currentOrientation);
            Vector3 axis = Vector3.Cross(Vector3.Normalize(_currentPose.Backward), Vector3.Normalize(Zaxis));
            float angle = (float)Math.Asin((double)axis.Length());
            if (Math.Abs(angle) < MathHelper.ToRadians(0.2f))
            {
                return _currentOrientation;
            }
            if (Vector3.Transform(Zaxis, Quaternion.Inverse(_currentOrientation)).Z < 0)
            {
                angle = Math.Sign(angle) * ((float)Math.PI - Math.Abs(angle));
            }
            return Quaternion.CreateFromAxisAngle(Vector3.Normalize(axis), angle) * _currentOrientation;
        }

        public static bool isOrientationAligned(Quaternion Q1, Quaternion Q2, double error)
        {
            if (getOrientationError(Matrix.CreateFromQuaternion(Q1),Matrix.CreateFromQuaternion(Q2)).Length() < error)
            {
                return true;
            }
            return false;
        }

        public static Vector3 getOrientationError(Quaternion reference, Quaternion measured)
        {
            Vector3 changeAxis = Vector3.Zero;
            float changeAngle = 0;
            Quaternion changeOrientation = Quaternion.Inverse(measured) * reference;
            SF.getAxisAngle(changeOrientation, out changeAxis, out changeAngle);
            return Vector3.Transform(changeAxis, measured) * changeAngle;
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

        public static Matrix M(Matrix mat1, Matrix mat2)
        {
            return Matrix.Transpose(Matrix.Transpose(mat1) * Matrix.Transpose(mat2));
        }

        static Matrix R(Matrix Rz, Matrix Ry, Matrix Rx)
        {
            return M(M(Rz, Ry), Rx);
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
                    outAxis = new Vector3(0, 0, 1);
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

        #endregion

        #region Kuka Functions


        public static readonly String[] cardinalKeys = new String[] { "X", "Y", "Z", "A", "B", "C" };
        public static readonly String[] axisKeys = new String[] { "A1", "A2", "A3", "A4", "A5", "A6" };
        public static readonly String[] rotationKeys = new String[] { "X1", "X2", "X3", "Y1", "Y2", "Y3", "Z1", "Z2", "Z3" };
        public static readonly String[] axisVecotrKeys = new String[] { "XX", "XY", "XZ", "ZX", "ZY", "ZZ" };

        public static TimeCoordinate AverageRateOfChange(TimeCoordinate[] list, double framerate)
        {
            TimeCoordinate averageRate = list[0].getRateOfChange(list[1],framerate); 

            for (int i = 2; i < list.Length; i++)
            {
                averageRate += list[i - 1].getRateOfChange(list[i],framerate);
            }
            return averageRate / (float)(list.Length - 1);
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
        public static void getKukaAngles(Quaternion rotation, out double[] KukaAngleOut)
        {
            KukaAngleOut = new double[6];
            double A = 0;
            double B = 0;
            double C = 0;

            Matrix rotationMat = Matrix.CreateFromQuaternion(rotation);
            rotationMat = Matrix.Transpose(rotationMat);

            B = Math.Atan2(-rotationMat.M31, Math.Sqrt(rotationMat.M32 * rotationMat.M32 + rotationMat.M33 * rotationMat.M33));

            if (Math.Abs(Math.Abs(B) - Math.PI / 2) < 1e-6)
            {
                // Gimbal lock situation! A and C form a line of infinate solutions.
                C = 0;// (float)Math.PI / 5f;
                A = Math.Atan2(Math.Sign(B) * rotationMat.M23, Math.Sign(B) * rotationMat.M13) + Math.Sign(B) * C;
            }
            else
            {
                A = Math.Atan2(rotationMat.M21, rotationMat.M11);
                C = Math.Atan2(rotationMat.M32, rotationMat.M33);
            }

            KukaAngleOut[3] = A * 180 / Math.PI;
            KukaAngleOut[4] = B * 180 / Math.PI;
            KukaAngleOut[5] = C * 180 / Math.PI;
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

        /// <summary>
        /// Computes the inverse jacobian to the WRIST where if singularity occures the result aproximates the jacobian.
        /// </summary>
        /// <param name="t"></param> 6 angles in radians.
        /// <param name="error"></param> Error for singularity detection, 1e-6 specified if greater than 1e-2.
        /// <returns></returns>
        public static double[,] InverseJacobianWrist(double[] t, double error)
        {
            if (t.Length == 6)
            {
                if (t.Max() > 2*Math.PI || Math.Abs(t.Min()) < - 2*Math.PI)
                {
                    throw new KukaException(string.Format("Angles given are out of bounds, -pi<t<pi. Max angle is: {0}. Min angle is: {1}", t.Max(), t.Min()));
                }
                error = (Math.Abs(error) < 1e-2) ? 1e-6 : Math.Abs(error);
                return InverseJacobian(t[0], t[1], t[2], t[3], t[4], t[5], error);
            }
            else
            {
                throw new KukaException(string.Format("Incorrect number of angles supplied. Need 6 angles but {0} were given.", t.Length));
            }
        }


        /// <summary>
        /// Computes the inverse jacobian to the WRIST where if singularity occures aproximates the jacobian.
        /// Angles are in radians!!!
        /// </summary>
        /// <param name="t1"></param>
        /// <param name="t2"></param>
        /// <param name="t3"></param>
        /// <param name="t4"></param>
        /// <param name="t5"></param>
        /// <param name="t6"></param>
        /// <param name="EE"></param>
        /// <returns></returns>
        static double[,] InverseJacobian(double t1, double t2, double t3, double t4, double t5, double t6, double error)
        {
            double[,] inverseJoc = new double[6, 6];
            double s1 = Math.Sin(t1);
            double c1 = Math.Cos(t1);
            double s2 = Math.Sin(t2);
            double s2p3 = Math.Sin(t2 + t3);
            double c2p3 = Math.Cos(t2 + t3);
            double c2 = Math.Cos(t2);
            double s3 = Math.Sin(t3);
            double c3 = Math.Cos(t3);
            double s3p = Math.Sin(t3 - Math.PI / 2);
            double c3p = Math.Cos(t3 - Math.PI / 2);
            double s4 = Math.Sin(t4);
            double c4 = Math.Cos(t4);
            double s5 = Math.Sin(t5);
            double c5 = Math.Cos(t5);
            double s6 = Math.Sin(t6);
            double c6 = Math.Cos(t6);
            double c234 = Math.Cos(t2 + t3 + t4);
            // These are the denominators withing the inverse Jacobian, if they tend to zero singularity is reached and velocities will tend to inf!
            double cot5 = (Math.Abs(s5) < error) ? c5 / error : c5 / s5;
            double singularity1 = (Math.Abs(103 * c2p3 + 7 * s2p3 + 112 * c2 + 5) < error) ? Math.Sign(103 * c2p3 + 7 * s2p3 + 112 * c2 + 5) * error : (103 * c2p3 + 7 * s2p3 + 112 * c2 + 5);
            double singularity2 = (Math.Abs(1960 * c3 - 28840 * s3) < error) ? Math.Sign(1960 * c3 - 28840 * s3) * error : (1960 * c3 - 28840 * s3);
            double singularity3 = (Math.Abs(7 * c3 - 103 * s3) < error) ? Math.Sign(7 * c3 - 103 * s3) * error : (7 * c3 - 103 * s3);
            double singularity4 = (Math.Abs(3920 * c3 - 57680 * s3) < error) ? Math.Sign(3920 * c3 - 57680 * s3) * error : (3920 * c3 - 57680 * s3);
            double Sing5 = (3605 * c2 * s5 - 175 * c3 * s5 - 53045 * s2 * s5 + 2575 * s3 * s5 + 57680 * c2 * s3 * s5 - 7210 * c2 * c3 * c3 * s5 + 52800 * c3 * c3 * s2 * s5 - 3920 * c2 * c3 * s5 + 52800 * c2 * c3 * s3 * s5 + 7210 * c3 * s2 * s3 * s5);
            double singularity5 = (Math.Abs(Sing5) < error) ? Math.Sign(Sing5) * error : (Sing5);
            double Sing6 = (c2p3 * c2p3 * s5 - c2p3 * c2p3 * c4 * c4 * s5 + s2p3 * c2 * s3 * s5 + s2p3 * c3 * s2 * s5 - s2p3 * c4 * c4 * s2 * s5 * s3p + c2p3 * c2 * c4 * c5 * c3p - c2p3 * c4 * c5 * s2 * s3p + s2p3 * c2 * c4 * c5 * s3p + s2p3 * c4 * c5 * c3p * s2 - c2p3 * c2 * c4 * c4 * s5 * s3p - c2p3 * c4 * c4 * c3p * s2 * s5 + s2p3 * c2 * c4 * c4 * c3p * s5 - s2p3 * c2 * c4 * c4 * s3 * s5 - s2p3 * c3 * c4 * c4 * s2 * s5);
            double singularity6 = (Math.Abs(Sing6) < error) ? Math.Sign(Sing6) * error : (Sing6);
            double singularity7 = (Math.Abs(7 * c3 * s5 - 103 * s3 * s5) < error) ? Math.Sign(7 * c3 * s5 - 103 * s3 * s5) * error : (7 * c3 * s5 - 103 * s3 * s5);
            double singularity8 = ((Math.Abs(s5) < error) ? Math.Sign(s5) * error : s5);
            double Sing9 = (5 * s5 * (721 * c2 - 35 * c3 - 10609 * s2 + 515 * s3 - 784 * c2 * c3 + 11536 * c2 * s3 - 1442 * c2 * c3 * c3 + 10560 * c3 * c3 * s2 + 1442 * c3 * s2 * s3 + 10560 * c2 * c3 * s3));
            double singularity9 = ((Math.Abs(Sing9) < error) ? Math.Sign(Sing9) * error : Sing9);
            double Sing10 = (3605 * c2 - 175 * c3 - 53045 * s2 + 2575 * s3 - 3920 * c2 * c3 + 57680 * c2 * s3 - 7210 * c2 * c3 * c3 + 52800 * c3 * c3 * s2 + 7210 * c3 * s2 * s3 + 52800 * c2 * c3 * s3);
            double singularity10 = ((Math.Abs(Sing10) < error) ? Math.Sign(Sing10) * error : Sing10);

            inverseJoc[0, 0] = -s1 / (5 * singularity1);
            inverseJoc[0, 1] = -c1 / (5 * singularity1);
            inverseJoc[0, 2] = 0;
            inverseJoc[0, 3] = 0;
            inverseJoc[0, 4] = 0;
            inverseJoc[0, 5] = 0;
            inverseJoc[1, 0] = -(103 * c1 * c2 * c3 - 103 * c1 * s2 * s3 + 7 * c1 * c2 * s3 + 7 * c1 * c3 * s2) / (2 * singularity2);
            inverseJoc[1, 1] = (7 * c2 * s1 * s3 + 7 * c3 * s1 * s2 - 103 * s1 * s2 * s3 + 103 * c2 * c3 * s1) / (2 * singularity2);
            inverseJoc[1, 2] = -(7 * c2p3 - 103 * s2p3) / (560 * singularity3);
            inverseJoc[1, 3] = 0;
            inverseJoc[1, 4] = 0;
            inverseJoc[1, 5] = 0;
            inverseJoc[2, 0] = (112 * c1 * c2 - 103 * c1 * s2 * s3 + 103 * c1 * c2 * c3 + 7 * c1 * c2 * s3 + 7 * c1 * c3 * s2) / singularity4;
            inverseJoc[2, 1] = -(112 * c2 * s1 + 7 * c2 * s1 * s3 + 7 * c3 * s1 * s2 - 103 * s1 * s2 * s3 + 103 * c2 * c3 * s1) / singularity4;
            inverseJoc[2, 2] = -(103 * s2p3 - 7 * c2p3 + 112 * s2) / (560 * singularity3);
            inverseJoc[2, 3] = 0;
            inverseJoc[2, 4] = 0;
            inverseJoc[2, 5] = 0;
            inverseJoc[3, 0] = -(103 * c2 * s1 * s5 + 112 * c1 * c2 * c2 * c5 * s4 - 103 * c2 * c3 * c3 * s1 * s5 - 7 * c3 * c3 * s1 * s2 * s5 + 5 * c1 * c2 * c5 * s4 + 103 * c4 * c5 * s1 * s2 + 103 * c1 * c2 * c2 * c3 * c5 * s4 + 7 * c2 * c3 * c3 * c4 * c5 * s1 + 7 * c1 * c2 * c2 * c5 * s3 * s4 - 103 * c3 * c3 * c4 * c5 * s1 * s2 - 7 * c2 * c3 * s1 * s3 * s5 + 103 * c3 * s1 * s2 * s3 * s5 + 7 * c1 * c2 * c3 * c5 * s2 * s4 - 103 * c2 * c3 * c4 * c5 * s1 * s3 - 103 * c1 * c2 * c5 * s2 * s3 * s4 - 7 * c3 * c4 * c5 * s1 * s2 * s3) / singularity5;
            inverseJoc[3, 1] = (103 * c1 * c2 * c3 * c3 * s5 - 103 * c1 * c2 * s5 + 7 * c1 * c3 * c3 * s2 * s5 + 112 * c2 * c2 * c5 * s1 * s4 - 103 * c1 * c4 * c5 * s2 + 5 * c2 * c5 * s1 * s4 + 103 * c1 * c3 * c3 * c4 * c5 * s2 + 103 * c2 * c2 * c3 * c5 * s1 * s4 + 7 * c2 * c2 * c5 * s1 * s3 * s4 + 7 * c1 * c2 * c3 * s3 * s5 - 103 * c1 * c3 * s2 * s3 * s5 - 7 * c1 * c2 * c3 * c3 * c4 * c5 + 103 * c1 * c2 * c3 * c4 * c5 * s3 + 7 * c1 * c3 * c4 * c5 * s2 * s3 + 7 * c2 * c3 * c5 * s1 * s2 * s4 - 103 * c2 * c5 * s1 * s2 * s3 * s4) / singularity5;
            inverseJoc[3, 2] = -(c5 * s2 * s4) / (5 * singularity7);
            inverseJoc[3, 3] = (c2p3 * c1 * c4 * c4 * s5 - c2p3 * c1 * s5 + c1 * c2 * c4 * c4 * s5 * s3p + c1 * c4 * c4 * c3p * s2 * s5 - c1 * c2 * c4 * c5 * c3p + c1 * c4 * c5 * s2 * s3p + c3 * c5 * s1 * s4 * s3p - c2 * c2 * c3 * c5 * s1 * s4 * s3p - c2 * c2 * c5 * c3p * s1 * s3 * s4 + c2p3 * c2 * c5 * s1 * s4 * s3p + c2p3 * c5 * c3p * s1 * s2 * s4 + c3 * c4 * c3p * s1 * s4 * s5 + c2 * c2 * c4 * s1 * s3 * s4 * s5 * s3p + c2p3 * c2 * c4 * c3p * s1 * s4 * s5 - c2p3 * c4 * s1 * s2 * s4 * s5 * s3p - c2 * c3 * c5 * c3p * s1 * s2 * s4 + c2 * c5 * s1 * s2 * s3 * s4 * s3p - c2 * c2 * c3 * c4 * c3p * s1 * s4 * s5 + c2 * c3 * c4 * s1 * s2 * s4 * s5 * s3p + c2 * c4 * c3p * s1 * s2 * s3 * s4 * s5) / singularity6;
            inverseJoc[3, 4] = (c2p3 * s1 * s5 - c2p3 * c4 * c4 * s1 * s5 - c2 * c4 * c4 * s1 * s5 * s3p - c4 * c4 * c3p * s1 * s2 * s5 + c2 * c4 * c5 * c3p * s1 + c1 * c3 * c5 * s4 * s3p - c4 * c5 * s1 * s2 * s3p + c2p3 * c1 * c2 * c5 * s4 * s3p + c2p3 * c1 * c5 * c3p * s2 * s4 + c1 * c3 * c4 * c3p * s4 * s5 - c1 * c2 * c2 * c3 * c5 * s4 * s3p - c1 * c2 * c2 * c5 * c3p * s3 * s4 + c2p3 * c1 * c2 * c4 * c3p * s4 * s5 - c2p3 * c1 * c4 * s2 * s4 * s5 * s3p - c1 * c2 * c3 * c5 * c3p * s2 * s4 + c1 * c2 * c5 * s2 * s3 * s4 * s3p - c1 * c2 * c2 * c3 * c4 * c3p * s4 * s5 + c1 * c2 * c2 * c4 * s3 * s4 * s5 * s3p + c1 * c2 * c3 * c4 * s2 * s4 * s5 * s3p + c1 * c2 * c4 * c3p * s2 * s3 * s4 * s5) / singularity6;
            inverseJoc[3, 5] = s2p3 - (Math.Cos(t2 + t3 + t4) * cot5) / 2 - (Math.Cos(t2 + t3 - t4) * cot5) / 2;
            inverseJoc[4, 0] = (112 * c1 * c2 * c2 * c4 - 103 * s1 * s2 * s4 + 5 * c1 * c2 * c4 + 103 * c1 * c2 * c2 * c3 * c4 + 7 * c1 * c2 * c2 * c4 * s3 - 7 * c2 * c3 * c3 * s1 * s4 + 103 * c3 * c3 * s1 * s2 * s4 + 7 * c1 * c2 * c3 * c4 * s2 - 103 * c1 * c2 * c4 * s2 * s3 + 103 * c2 * c3 * s1 * s3 * s4 + 7 * c3 * s1 * s2 * s3 * s4) / singularity10;
            inverseJoc[4, 1] = -(103 * c1 * s2 * s4 + 112 * c2 * c2 * c4 * s1 + 5 * c2 * c4 * s1 + 7 * c1 * c2 * c3 * c3 * s4 + 103 * c2 * c2 * c3 * c4 * s1 - 103 * c1 * c3 * c3 * s2 * s4 + 7 * c2 * c2 * c4 * s1 * s3 + 7 * c2 * c3 * c4 * s1 * s2 - 103 * c1 * c2 * c3 * s3 * s4 - 103 * c2 * c4 * s1 * s2 * s3 - 7 * c1 * c3 * s2 * s3 * s4) / singularity10;
            inverseJoc[4, 2] = (c4 * s2) / (5 * singularity3);
            inverseJoc[4, 3] = c4 * s1 - c1 * c2 * s3 * s4 - c1 * c3 * s2 * s4;
            inverseJoc[4, 4] = c1 * c4 + c2 * s1 * s3 * s4 + c3 * s1 * s2 * s4;
            inverseJoc[4, 5] = -c2p3 * s4;
            inverseJoc[5, 0] = -(103 * c4 * s1 * s2 + 112 * c1 * c2 * c2 * s4 + 5 * c1 * c2 * s4 + 103 * c1 * c2 * c2 * c3 * s4 + 7 * c2 * c3 * c3 * c4 * s1 + 7 * c1 * c2 * c2 * s3 * s4 - 103 * c3 * c3 * c4 * s1 * s2 + 7 * c1 * c2 * c3 * s2 * s4 - 103 * c2 * c3 * c4 * s1 * s3 - 103 * c1 * c2 * s2 * s3 * s4 - 7 * c3 * c4 * s1 * s2 * s3) / singularity9;
            inverseJoc[5, 1] = (5 * c2 * s1 * s4 + 112 * c2 * c2 * s1 * s4 - 103 * c1 * c4 * s2 - 7 * c1 * c2 * c3 * c3 * c4 + 103 * c1 * c3 * c3 * c4 * s2 + 103 * c2 * c2 * c3 * s1 * s4 + 7 * c2 * c2 * s1 * s3 * s4 + 103 * c1 * c2 * c3 * c4 * s3 + 7 * c1 * c3 * c4 * s2 * s3 + 7 * c2 * c3 * s1 * s2 * s4 - 103 * c2 * s1 * s2 * s3 * s4) / singularity9;
            inverseJoc[5, 2] = -(s2 * s4) / (5 * singularity7);
            inverseJoc[5, 3] = -(s1 * s4 + c1 * c2 * c4 * s3 + c1 * c3 * c4 * s2) / singularity8;
            inverseJoc[5, 4] = (c2 * c4 * s1 * s3 - c1 * s4 + c3 * c4 * s1 * s2) / singularity8;
            inverseJoc[5, 5] = -(c2p3 * c4) / singularity8;
            return inverseJoc;
        }


    }
        #endregion


}
