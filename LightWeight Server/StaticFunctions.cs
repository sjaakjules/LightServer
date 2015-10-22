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


        /// <summary>
        /// Loads a new Pose using a string with the infomation of position, velocity and orientation. Can handle partial information.
        /// </summary>
        /// <param name="PositionInfo"></param>
        /// <param name="LastPose"></param>
        public Pose(string[] PositionInfo, string[] OrientationInfo, Pose LastPose)
        {
            // The following If statements catch error while trying to load positions, use last position and keep loading.
            // TODO: Add signals and more complex error handeling to send back to external server. Perhaps the string which was received for error checking?
            if (PositionInfo == null || PositionInfo.Length != 3)
            {
                _x = LastPose._x;
                _y = LastPose._y;
                _z = LastPose._z;
            }
            else
            {
                if (!double.TryParse(PositionInfo[0], out _x))
                {
                    _x = LastPose._x;
                }
                if (!double.TryParse(PositionInfo[1], out _y))
                {
                    _y = LastPose._y;
                }
                if (!double.TryParse(PositionInfo[2], out _z))
                {
                    _z = LastPose._z;
                }
            }
            if (OrientationInfo == null)
            {
                this._Orientation = LastPose.Orientation;
                SF.getAxisAngle(LastPose.Orientation, out _axis, out _angle);
                SF.getKukaAngles(LastPose.Orientation, out _kukaValues);
                _kukaValues[0] = (float)_x;
                _kukaValues[1] = (float)_y;
                _kukaValues[2] = (float)_z;
            }
            else
            {
                double[] Axis = new double[OrientationInfo.Length];
                bool loadedAxis = true;
                for (int i = 0; i < OrientationInfo.Length; i++)
                {
                    if (!double.TryParse(OrientationInfo[i], out Axis[i]))
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


        public double[,] getMatrix
        {
            get
            {
                Matrix orientation = Matrix.CreateFromQuaternion(this.Orientation);
                double[,] matrix = new double[,] {  { orientation.M11, orientation.M21, orientation.M31, this.Translation.X }, 
                                                        { orientation.M12, orientation.M22, orientation.M32, this.Translation.Y }, 
                                                        { orientation.M13, orientation.M23, orientation.M33, this.Translation.Z }, { 0, 0, 0, 1 } };
                return matrix;
            }
        }


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
            return new TimeCoordinate(1.0f * Pose1.x / Value, 1.0f * Pose1.y / Value, 1.0f * Pose1.z / Value, Vector3.Divide(Pose1.axis, Value), 1.0f * Pose1.angle / Value, Pose1.Ipoc);
        }
        
        public TimeCoordinate getRateOfChange(TimeCoordinate pose1, double framerate)
        {
            Vector3 changeAxis;
            float changeAngle;
            if (this.Ipoc < pose1.Ipoc)
            {
                Quaternion changeOrientation = Quaternion.Inverse(this._Orientation) * pose1._Orientation;
                SF.getAxisAngle(changeOrientation, out changeAxis, out changeAngle);
                return new TimeCoordinate(1.0f * (pose1.x - this.x) / framerate,
                                            1.0f * (pose1.y - this.y) / framerate,
                                            1.0f * (pose1.z - this.z) / framerate,
                                            Vector3.Transform(changeAxis, pose1._Orientation),
                                            1.0f * changeAngle / (float)framerate,
                                            pose1.Ipoc);
            }
            else
            {
                Quaternion changeOrientation = Quaternion.Inverse(pose1._Orientation) * this._Orientation;
                SF.getAxisAngle(changeOrientation, out changeAxis, out changeAngle);
                return new TimeCoordinate(1.0f * (this.x - pose1.x) / framerate,
                                            1.0f * (this.y - pose1.y) / framerate,
                                            1.0f * (this.z - pose1.z) / framerate,
                                            Vector3.Transform(changeAxis, this._Orientation),
                                            1.0f * changeAngle / (float)framerate,
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
                return new TimeCoordinate(1.0f * (pose1.x - this.x) / delTime,
                                            1.0f * (pose1.y - this.y) / delTime,
                                            1.0f * (pose1.z - this.z) / delTime,
                                            Vector3.Transform(changeAxis,this._Orientation),
                                            1.0f * changeAngle / (float)delTime,
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
                return new TimeCoordinate(1.0f * (this.x - pose1.x) / delTime,
                                            1.0f * (this.y - pose1.y) / delTime,
                                            1.0f * (this.z - pose1.z) / delTime,
                                            Vector3.Transform(changeAxis, pose1._Orientation),
                                            1.0f * changeAngle / (float)delTime,
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



    /// <summary>
    /// Exception class for general orientation and rotation related functions, derived from System.Exception
    /// </summary>
    public class InverseKinematicsException : Exception
    {
        public InverseKinematicsException()
            : base()
        { }

        public InverseKinematicsException(string Message)
            : base(Message)
        { }

        public InverseKinematicsException(string Message, Exception InnerException)
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

        public static void plotDoubles(double[] array, string Heading)
        {
            if (Heading.Length < 24)
            {
                string newheading = string.Concat(Heading, ":", new string(' ', 24 - Heading.Length));
            }
            Console.Write(String.Format("{0,-25}\t (", Heading));
            for (int i = 0; i < array.Length; i++)
            {
                Console.Write(String.Format(" {0:0.0} ,", array[i]));
            }
            Console.WriteLine(")");
        }

        public static string DoublesToString(double[] array)
        {
            StringBuilder strOut = new StringBuilder();
            strOut.Append("(");
            for (int i = 0; i < array.Length; i++)
            {
                strOut.Append(String.Format(" {0:0.0} ,", array[i]));
            }
            strOut.AppendLine(")");
            return strOut.ToString();
        }

        public static double[] getRadian(double[] array)
        {
            double[] newArray = new double[array.Length];
            for (int i = 0; i < array.Length; i++)
            {
                newArray[i] = 1.0 * array[i] * Math.PI / 180;
            }
            return newArray;
        }

        public static double[] getDegree(double[] array)
        {
            double[] newArray = new double[array.Length];
            for (int i = 0; i < array.Length; i++)
            {
                newArray[i] = 180.0 * array[i] / Math.PI;
            }
            return newArray;
        }

        public static bool IsClose(double[] A1, double[] A2)
        {
            for (int i = 0; i < A1.Length; i++)
            {
                if (Math.Abs(A1[i] - A2[i]) > 1e-1)
                {
                    return false;
                }
            }
            return true;
        }

        static public double[] createDouble(double value, int length)
        {
            double[] arrayout = new double[length];
            for (int i = 0; i < length; i++)
            {
                arrayout[i] = value;
            }
            return arrayout;
        }

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
            float s = (float)Math.Sin(1.0 * angle / 2);
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
                Vector3 axis = new Vector3(1.0f * quaternion.X / s, 1.0f * quaternion.Y / s, 1.0f * quaternion.Z / s);
                axis.Normalize();
                s = quaternion.X == 0 ? s : 1.0f * quaternion.X / axis.X;
                s = quaternion.Y == 0 ? s : 1.0f * quaternion.Y / axis.Y;
                s = quaternion.Z == 0 ? s : 1.0f * quaternion.Z / axis.Z;

                if (float.IsNaN(quaternion.Z))
                {
                    outAxis.Z = 1;
                }
                outAxis = new Vector3(1.0f * quaternion.X / s, 1.0f * quaternion.Y / s, 1.0f * quaternion.Z / s);
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
            Quaternion Rz = Quaternion.CreateFromRotationMatrix(Matrix.CreateRotationZ((float)(1.0 * pose[3] * Math.PI / 180)));
            Quaternion Ry = Quaternion.CreateFromRotationMatrix(Matrix.CreateRotationY((float)(1.0 * pose[4] * Math.PI / 180)));
            Quaternion Rx = Quaternion.CreateFromRotationMatrix(Matrix.CreateRotationX((float)(1.0 * pose[5] * Math.PI / 180)));
            return (Rz * Ry * Rx);
        }

        public static Quaternion MakeQuaternionFromKuka(double A, double B, double C)
        {
            Quaternion Rz = Quaternion.CreateFromRotationMatrix(Matrix.CreateRotationZ((float)(1.0 * A * Math.PI / 180)));
            Quaternion Ry = Quaternion.CreateFromRotationMatrix(Matrix.CreateRotationY((float)(1.0 * B * Math.PI / 180)));
            Quaternion Rx = Quaternion.CreateFromRotationMatrix(Matrix.CreateRotationX((float)(1.0 * C * Math.PI / 180)));
            return (Rz * Ry * Rx);
        }

        public static Matrix MakeMatrixFromKuka(double[] pose)
        {
            Matrix Rz = Matrix.CreateRotationZ((float)(1.0 * pose[3] * Math.PI / 180));
            Matrix Ry = Matrix.CreateRotationY((float)(1.0 * pose[4] * Math.PI / 180));
            Matrix Rx = Matrix.CreateRotationX((float)(1.0 * pose[5] * Math.PI / 180));
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

            if (Math.Abs(Math.Abs(B) - 1.0 * Math.PI / 2) < 1e-6)
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

            KukaAngleOut[3] = 1.0 * A * 180 / Math.PI;
            KukaAngleOut[4] = 1.0 * B * 180 / Math.PI;
            KukaAngleOut[5] = 1.0 * C * 180 / Math.PI;
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

            if (Math.Abs(Math.Abs(B) - 1.0 * Math.PI / 2) < 1e-6)
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

            if (Math.Abs(Math.Abs(B) - 1.0 * Math.PI / 2) < 1e-6)
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

        #endregion


        internal static void updateDataFile(Pose referencePosition, Pose measuredPosition, Pose measuredVelocity, double time, double[] referenceAngles, double[] controlAngles, double[] measuredAngles, StringBuilder DataWriter)
        {
            DataWriter.Append(string.Format("{0:0.0},{1:data},{2:data},{3:data},{4:data},{5},{6};", time, referencePosition, measuredPosition, measuredVelocity, printDouble(referenceAngles), printDouble(measuredAngles), printDouble(controlAngles)));
        }
    }


}
