using Microsoft.Xna.Framework;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Net;
using System.Text;
using System.Threading.Tasks;

namespace LightWeight_Server
{
    class RobotInfo
    {
        object TrajectoryPrintLock = new object();
        object trajectoryLock = new object();
        object RobotInfoLock = new object();
        object gripperLock = new object();
        object linearVelocityLock = new object();
        object linearAccelerationLock = new object();
        object angularVelocityLock = new object();
        object angularAccelerationLock = new object();
        object axisComandLock = new object();
        object axisRotateLock = new object();
        object EEposeLock = new object();

        Stopwatch _KukaCycleTime = new Stopwatch();
        public Stopwatch IPOC = new Stopwatch();

        // Time of loop in SECONDS
        double _loopTime = 0;
        Stopwatch _ConnectionTimer = new Stopwatch();
        double _JacobienAverTimes = 0;
        Stopwatch A1 = new Stopwatch(), A2 = new Stopwatch(), jacobianTimer = new Stopwatch();
        public FixedSizedQueue<double> _maxProcessDataTimer = new FixedSizedQueue<double>(10);
        public FixedSizedQueue<double> _trajectoryLoaderTime = new FixedSizedQueue<double>(10);
        public FixedSizedQueue<double> _processDataTimer = new FixedSizedQueue<double>(10);
        public FixedSizedQueue<double> JocTimer = new FixedSizedQueue<double>(10);
        public FixedSizedQueue<double> MaxJocTimer = new FixedSizedQueue<double>(10);
        public FixedSizedQueue<double> serverTimer = new FixedSizedQueue<double>(10);
        public FixedSizedQueue<double> MaxserverTimer = new FixedSizedQueue<double>(10);

        FixedSizedQueue<TimeCoordinate> _Position = new FixedSizedQueue<TimeCoordinate>(6);
        FixedSizedQueue<TimeCoordinate> _velocity = new FixedSizedQueue<TimeCoordinate>(6);
        FixedSizedQueue<TimeCoordinate> _acceleration = new FixedSizedQueue<TimeCoordinate>(6);
        FixedSizedQueue<TimeCoordinate> _Torque = new FixedSizedQueue<TimeCoordinate>(6);
        FixedSizedQueue<double[]> _Angles = new FixedSizedQueue<double[]>(6);
        FixedSizedQueue<Pose> _ReferencePosition = new FixedSizedQueue<Pose>(6);
        FixedSizedQueue<Pose> _ReferenceVelocity = new FixedSizedQueue<Pose>(6);

        public readonly Guid _RobotID;
        ScreenWriter _GUI;
        IPEndPoint ConnectedIP;

        Pose[] _T = new Pose[7];
        Matrix[] _T0 = new Matrix[7];
        double[,] _Jacobian = new double[6, 6];
        double[,] _InverseJacobian = new double[6, 6];

        Pose _newPose;
        Pose[] _newPoses;
        double[] _newVelocitys;
        TrajectoryOld[] _TrajectoryList, _NewTrajectoryList;
        TrajectoryHandler _TrajectoryHandler;

        TimeCoordinate _CommandPose;

        Pose _StartPose, _StartTipPose;
        Pose _EndEffectorPose;
        Vector3 _EndEffector;

        // Thread safe lists for updating and storing of robot information.
        // public ConcurrentStack<StateObject> DataHistory;


        //Trajectory _CurrentTrajectory;
        TrajectoryOld _CurrentTrajectory;
        int _currentSegment = 1;
        Controller _Controller;

        bool _gripperIsOpen = true;
        public ElbowPosition _elbow = ElbowPosition.up;
        public BasePosition _base = BasePosition.front;

        // T1 < 250mm/s   T1 > 250mm/s   = .25mm/ms  = 1mm/cycle
        readonly double _MaxCartesianChange = 1;
        readonly double _MaxAngularChange = 0.1;
        readonly double _MaxAxisChange = 0.00015;

        double _maxLinearVelocity = .12; // in mm/ms
        double _maxAngularVelocity = .012; // in mm/ms
        float _maxLinearAcceleration = 0.005f;// in mm/ms2
        float _maxAngularAcceleration = 0.00005f; // in deg/ms2
        
        bool _isConnected = false;
        bool _isConnecting = false;
        bool _isCommanded = false;
        bool _isCommandedPosition = false;
        bool _isCommandedOrientation = false;

        bool _newCommandLoaded = false;
        bool _newOrientationLoaded = false;
        bool _newPositionLoaded = false;
        bool _newPosesLoaded = false;
        bool _isVia = false;

        public double[] _axisCommand = new double[] { 0, 0, 0, 0, 0, 0 };
        public readonly double[] _defaultHomeAngles = new double[] { 0, 0 };
        double[] _homePosition = new double[6];
        double[] _HomeAngles = new double[6];


        #region Properties
        public double[] homePosition { get { return _homePosition; } private set { _homePosition = value; } }
        public double[] homeAngles { get { return _HomeAngles; } private set { _HomeAngles = value; } }

        public Pose currentPose { get { return _Position.LastElement.Pose; } }

        public Pose currentVelocity { get { return _velocity.LastElement.Pose; } }

        public Pose currentAcceleration { get { return _acceleration.LastElement.Pose; } }

        public double[] currentAxisAngle { get { return _Angles.LastElement; } }

        public Vector3 EndEffector { get { return _EndEffector; } }

        public Pose currentDesiredPositon { get { return _TrajectoryHandler.DesiredPose; } }

        public Pose currentReferencePosition { get { return _ReferencePosition.LastElement; } }

        public Pose currentReferenceVelocity { get { return _ReferenceVelocity.LastElement; } }

        public IPEndPoint EndPoint { get { return ConnectedIP; } set { ConnectedIP = value; } }

        public double LinearVelocity
        {
            get
            {
                lock (linearVelocityLock)
                {
                    return _maxLinearVelocity;
                }
            }
            set
            {
                lock (linearVelocityLock)
                {
                    if (value > _MaxCartesianChange)
                    {
                        // Assume new velocity given in mm/s
                        _maxLinearVelocity = 1.0 * value / 1000;
                    }
                    else { _maxLinearVelocity = value; }
                    
                }
            }
        }

        public double AngularVelocity
        {
            get
            {
                lock (angularVelocityLock)
                {
                    return _maxAngularVelocity;
                }
            }
            set
            {
                lock (angularVelocityLock)
                {
                    if (value > _MaxAngularChange)
                    {
                        // assume new velocity given in deg/s
                        _maxAngularVelocity = 1.0 * value / 1000;
                    }
                    else
                    {
                        _maxAngularVelocity = value;
                    }
                }
            }
        }

        public float LinearAcceleration
        {
            get
            {
                lock (linearAccelerationLock)
                {
                    return _maxLinearAcceleration;
                }
            }
            set
            {
                lock (linearAccelerationLock)
                {
                    _maxLinearAcceleration = value;
                }
            }
        }

        public float AngularAcceleration
        {
            get
            {
                lock (angularAccelerationLock)
                {
                    return _maxAngularAcceleration;
                }
            }
            set
            {
                lock (angularAccelerationLock)
                {
                    _maxAngularAcceleration = value;
                }
            }
        }


        public bool isVia
        {
            get { return _isVia; }
            set { _isVia = value; }
        }

        public bool gripperIsOpen
        {
            get
            {
                lock (gripperLock)
                {
                    return _gripperIsOpen;
                }
            }
            set
            {
                lock (gripperLock)
                {
                    _gripperIsOpen = value;
                }
            }
        }

        #endregion

        public RobotInfo(Guid id, ScreenWriter GUI, int RobotNumber)
        {
            this._RobotID = id;
            this._GUI = GUI;
            _TrajectoryHandler = new TrajectoryHandler(this);
            _CurrentTrajectory = new TrajectoryOld();
            _Controller = new Controller(this);
            _Position.Enqueue(new TimeCoordinate(540.5, -18.1, 833.3, 180.0, 0.0, 180.0, 0));
            _velocity.Enqueue(new TimeCoordinate(540.5, -18.1, 833.3, 180.0, 0.0, 180.0, 0));
            _acceleration.Enqueue(new TimeCoordinate(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0));
            _Torque.Enqueue(new TimeCoordinate(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0));
            _Angles.Enqueue(new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
            _CommandPose = new TimeCoordinate(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);
            _ReferencePosition.Enqueue(Pose.Zero);
            _ReferenceVelocity.Enqueue(Pose.Zero);
            MaxJocTimer.Enqueue(0);
            MaxserverTimer.Enqueue(0);
            GUI.ConnectRobot(this, RobotNumber);
        }

        public void Connect()
        {
            if (_isConnecting)
            {
                IPOC.Start();
                _KukaCycleTime.Start();
                homeAngles = _Angles.LastElement;
                homePosition = _Position.LastElement.kukaValues;
                _StartPose = forwardKinimatics(homeAngles, Vector3.Zero);
                _StartTipPose = _Position.LastElement.Pose;
                _EndEffectorPose = Pose.inverse(_StartPose) * _StartTipPose ;
                _EndEffector = _EndEffectorPose.Translation;
                getLinkTransforms(homeAngles[0], homeAngles[1], homeAngles[2], homeAngles[3], homeAngles[4], homeAngles[5], _EndEffector, out _T, out _T0);
                double[] inverseAngles = IKSolver(_StartTipPose, _EndEffector, _Angles.LastElement, ref _elbow, ref _base);
                if (!SF.IsClose(homeAngles,inverseAngles))
                {
                    updateError(string.Format("Inverse Home not equal\nMeasured: (0}\nInverse: {1}",SF.DoublesToString(homeAngles),SF.DoublesToString(inverseAngles)),new KukaException("IK solver error"));
                }
                _isConnected = true;
                _GUI.IsConnected = true;
                _isConnecting = false;
            }
            if (!_isConnected)
            {
                _isConnecting = true;
            }
        }



        public void Disconnect()
        {
            lock (trajectoryLock)
            {
                try
                {
                    _isConnected = false;
                    _KukaCycleTime.Reset();

                }
                catch (Exception e)
                {

                }

            }
        }


        public void updateError(string newError, Exception Error)
        {
            _GUI.updateError(newError, Error);
        }

        #region Movement
        public void updateRobotPosition(long LIpoc, long Ipoc, double x, double y, double z, double a, double b, double c)
        {
            //updateError(x.ToString() + " : " + y.ToString() + " : " + z.ToString() + " : " + a.ToString() + " : " + b.ToString() + " : " + c.ToString());
            _loopTime = _KukaCycleTime.Elapsed.TotalMilliseconds;
            _KukaCycleTime.Restart();
            a = (a == -180) ? 180 : a;
            b = (a == -180) ? 180 : b;
            c = (a == -180) ? 180 : c;
            TimeCoordinate newPosition = new TimeCoordinate(x, y, z, a, b, c, Ipoc);
            _Position.Enqueue(newPosition);
            TimeCoordinate[] positions = _Position.ThreadSafeToArray;
            _velocity.Enqueue(SF.AverageRateOfChange(positions,_loopTime));
            TimeCoordinate[] velocities = _velocity.ThreadSafeToArray;
            _acceleration.Enqueue(SF.AverageRateOfChange(velocities,_loopTime));
        }


        /// <summary>
        /// Assume Angles are in radians!
        /// </summary>
        /// <param name="t1"></param>
        /// <param name="t2"></param>
        /// <param name="t3"></param>
        /// <param name="t4"></param>
        /// <param name="t5"></param>
        /// <param name="t6"></param>
        /// <returns></returns>
        void getLinkTransforms(double t1, double t2, double t3, double t4, double t5, double t6, Vector3 End, out Pose[] T, out Matrix[] T0)
        {
            Matrix T01 = Matrix.Transpose(new Matrix((float)Math.Cos(t1), (float)-Math.Sin(t1), 0, 0, (float)-Math.Sin(t1), (float)-Math.Cos(t1), 0, 0, 0, 0, -1, 400, 0, 0, 0, 1));
            Matrix T12 = Matrix.Transpose(new Matrix((float)Math.Cos(t2), (float)-Math.Sin(t2), 0, 25, 0, 0, -1, 0, (float)Math.Sin(t2), (float)Math.Cos(t2), 0, 0, 0, 0, 0, 1));
            Matrix T23 = Matrix.Transpose(new Matrix((float)Math.Sin(t3), (float)Math.Cos(t3), 0, 560, (float)-Math.Cos(t3), (float)Math.Sin(t3), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1));
            Matrix T34 = Matrix.Transpose(new Matrix((float)Math.Cos(t4), (float)-Math.Sin(t4), 0, 35, 0, 0, -1, 515, (float)Math.Sin(t4), (float)Math.Cos(t4), 0, 0, 0, 0, 0, 1));
            Matrix T45 = Matrix.Transpose(new Matrix((float)Math.Cos(t5), (float)-Math.Sin(t5), 0, 0, 0, 0, 1, 0, (float)-Math.Sin(t5), (float)-Math.Cos(t5), 0, 0, 0, 0, 0, 1));
            Matrix T56 = Matrix.Transpose(new Matrix((float)-Math.Cos(t6), (float)-Math.Sin(t6), 0, 0, 0, 0, 1, 0, (float)-Math.Sin(t6), (float)Math.Cos(t6), 0, 0, 0, 0, 0, 1));
            Matrix T67 = Matrix.Transpose(new Matrix(1, 0, 0, End.X, 0, 1, 0, End.Y, 0, 0, 1, End.Z+80, 0, 0, 0, 1));
            Matrix T02 = SF.M(T01, T12);
            Matrix T03 = SF.M(T02, T23);
            Matrix T04 = SF.M(T03, T34);
            Matrix T05 = SF.M(T04, T45);
            Matrix T06 = SF.M(T05, T56);
            Matrix T07 = SF.M(T06, T67);
            T = new Pose[] { new Pose(T01), new Pose(T12), new Pose(T23), new Pose(T34), new Pose(T45), new Pose(T56), new Pose(T67) };
            T0 = new Matrix[] {T01,T02,T03,T04,T05,T06,T07};
        }

        double[,] InverseJacobian(double error)
        {
            double[,] InvWristJacobian = SF.InverseJacobianWrist(_Angles.LastElement, error);
            return getTipInverseJacobian(InvWristJacobian, _EndEffector);
        }

        double[,] Jacobian(Pose[] T, Matrix[] T0)
        {
            // TODO: return jacobian
            Vector3[] rit = new Vector3[6];
            Vector3[] Jr = new Vector3[6];
            Vector3[] Jw = new  Vector3[6];
            rit[5] = T[6].Translation;
            for (int i = 4; i >= 0; i--)
			{
			    rit[i] = T[i+1].Translation + Vector3.Transform(rit[i+1],T[i+1].Orientation);
			}
            for (int i = 0; i < 6; i++)
			{
                T0[i].Translation = Vector3.Zero;
			    Jr[i] = Vector3.Transform(Vector3.Cross(new Vector3(0,0,1),rit[i]),T0[i]);
                Jw[i] = new Vector3(T0[i].M31,T0[i].M32,T0[i].M33);
			}

            return new double[,] {{Jr[0].X,Jr[1].X,Jr[2].X,Jr[3].X,Jr[4].X,Jr[5].X},
                                  {Jr[0].Y,Jr[1].Y,Jr[2].Y,Jr[3].Y,Jr[4].Y,Jr[5].Y},
                                  {Jr[0].Z,Jr[1].Z,Jr[2].Z,Jr[3].Z,Jr[4].Z,Jr[5].Z},
                                  {Jw[0].X,Jw[1].X,Jw[2].X,Jw[3].X,Jw[4].X,Jw[5].X},
                                  {Jw[0].Y,Jw[1].Y,Jw[2].Y,Jw[3].Y,Jw[4].Y,Jw[5].Y},
                                  {Jw[0].Z,Jw[1].Z,Jw[2].Z,Jw[3].Z,Jw[4].Z,Jw[5].Z}};
        }


        /// <summary>
        /// Computers the forwards kinimatics using a known EndEffector displacement aligned with T67. The angles are in DEGREES, EE is in mm
        /// </summary>
        /// <param name="angles"></param>
        /// <param name="EE"></param>
        /// <returns></returns>
        public Pose forwardKinimatics(double[] angles, Vector3 EE)
        {
            return forwardKinimatics(angles[0], angles[1], angles[2], angles[3], angles[4], angles[5], EE);
        }

        /// <summary>
        /// Computers the forwards kinimatics using a known EndEffector displacement aligned with T67. The angles are in radians EE is in mm
        /// </summary>
        /// <param name="a1"></param>
        /// <param name="a2"></param>
        /// <param name="a3"></param>
        /// <param name="a4"></param>
        /// <param name="a5"></param>
        /// <param name="a6"></param>
        /// <param name="EE"></param>
        /// <returns></returns>
        public Pose forwardKinimatics(double a1, double a2, double a3, double a4, double a5, double a6, Vector3 EE)
        {
            double s1 = Math.Sin(a1);
            double c1 = Math.Cos(a1);
            double s2 = Math.Sin(a2);
            double c2 = Math.Cos(a2);
            double s3p = Math.Sin(a3 - 1.0*Math.PI / 2);
            double c3p = Math.Cos(a3 - 1.0 * Math.PI / 2);
            double s4 = Math.Sin(a4);
            double c4 = Math.Cos(a4);
            double s5 = Math.Sin(a5);
            double c5 = Math.Cos(a5);
            double s6 = Math.Sin(a6);
            double c6 = Math.Cos(a6);

            double a = (c4 * s1 + s4 * (c1 * s2 * s3p - c1 * c2 * c3p));
            double b1 = (s1 * s4 - c4 * (c1 * s2 * s3p - c1 * c2 * c3p));
            double b2 = (c1 * c2 * s3p + c1 * c3p * s2);
            double b = (c5 * b1 - s5 * b2);

            double m11 = -s6 * a - c6 * b;
            double m12 = c6 * a - s6 * b;
            double m13 = -s5 * b1 - c5 * b2;
            double m14 = 25 * c1 + 560 * c1 * c2 - EE.X * (s6 * a + c6 * b) + EE.Y * (c6 * (c4 * s1 + s4 * (c1 * s2 * s3p - c1 * c2 * c3p)) - s6 * b) - (s5 * b1 + c5 * b2) * (EE.Z + 80) - 515 * c1 * c2 * s3p - 515 * c1 * c3p * s2 - 35 * c1 * s2 * s3p + 35 * c1 * c2 * c3p;

            a = (c1 * c4 + s4 * (c2 * c3p * s1 - s1 * s2 * s3p));
            b1 = (c1 * s4 - c4 * (c2 * c3p * s1 - s1 * s2 * s3p));
            b2 = (c2 * s1 * s3p + c3p * s1 * s2);
            b = (c5 * b1 + s5 * b2);
            double m21 = -s6 * a - c6 * b;
            double m22 = c6 * a - s6 * b;
            double m23 = c5 * b2 - s5 * b1;
            double m24 = EE.Y * (c6 * a - s6 * b) - 560 * c2 * s1 - EE.X * (s6 * a + c6 * b) - 25 * s1 - (s5 * b1 - c5 * b2) * (EE.Z + 80) - 35 * c2 * c3p * s1 + 515 * c2 * s1 * s3p + 515 * c3p * s1 * s2 + 35 * s1 * s2 * s3p;

            a = (c2 * s3p + c3p * s2);
            b1 = (c2 * c3p - s2 * s3p);
            b = (s5 * b1 + c4 * c5 * a);
            double m31 = c6 * b - s4 * s6 * a;
            double m32 = s6 * b + c6 * s4 * a;
            double m33 = c4 * s5 * a - c5 * b1;
            double m34 = 515 * s2 * s3p - 515 * c2 * c3p - 35 * c2 * s3p - 35 * c3p * s2 - 560 * s2 - (c5 * b1 - c4 * s5 * a) * (EE.Z + 80) + EE.X * (c6 * b - s4 * s6 * a) + EE.Y * (s6 * b + c6 * s4 * a) + 400;

            double m41 = 0;
            double m42 = 0;
            double m43 = 0;
            double m44 = 1;


            Matrix M = new Matrix((float)m11, (float)m12, (float)m13, (float)m14, (float)m21, (float)m22, (float)m23, (float)m24, (float)m31, (float)m32, (float)m33, (float)m34, (float)m41, (float)m42, (float)m43, (float)m44);
            M = Matrix.Transpose(M);
            return new Pose(M);
        }


        public enum ElbowPosition { up, down, stretched };
        public enum BasePosition { front, back, vertical };


        double[] IK1to3(Pose des, Vector3 EE, double[] lastVal, ref ElbowPosition elbow, ref BasePosition basePos)
        {

            double wristOffset = Math.Atan2(35, 515);
            double theta1a, theta1b, theta1, theta2u, theta2d, theta2, theta3u, theta3d, theta3;
            Vector3 Wrist = des * (-EE - new Vector3(0, 0, 80));
            if (Wrist.Z < 0) throw new InverseKinematicsException("Out of workspace");
            theta1a = -Math.Atan2(Wrist.Y, Wrist.X);
            theta1b = (theta1a > 0) ? theta1a - Math.PI : theta1a + Math.PI;
            if (Math.Abs(lastVal[0] - theta1a) < Math.Abs(lastVal[0] - theta1b))
            {
                theta1 = theta1a;
                basePos = BasePosition.front;
            }
            else
            {
                theta1 = theta1b;
                basePos = BasePosition.back;
            }
            if (theta1 < -170.0 * Math.PI / 180 || theta1 > 170.0 * Math.PI / 180)
            {
                throw new InverseKinematicsException("Out of workspace");
            }
            Vector3 Base = new Vector3(25 * (float)Math.Cos(-theta1), 25 * (float)Math.Sin(-theta1), 400f);
            Vector3 LinkBW = Wrist - Base;
            if (Math.Abs(LinkBW.Length() - (560 + Math.Sqrt(515 * 515 + 35 * 35))) < 1e-6)
            {
                throw new InverseKinematicsException("Out of workspace");
            }
            if (LinkBW.Length() == (560 + Math.Sqrt(515 * 515 + 35 * 35)))
            {
                theta3 = -Math.Atan2(35, 515);
            }
            Vector3 xHat = new Vector3(LinkBW.X, LinkBW.Y, 0);
            xHat.Normalize();
            double beta = Math.Atan2(LinkBW.Z, Math.Sqrt(LinkBW.X * LinkBW.X + LinkBW.Y * LinkBW.Y));
            double gamma = Math.Acos((1.0 * LinkBW.LengthSquared() + 560 * 560 - 35 * 35 - 515 * 515) / (2 * 560 * LinkBW.Length()));
            double alpha = Math.Acos((1.0 * 560 * 560 + 35 * 35 + 515 * 515 - LinkBW.LengthSquared()) / (2.0 * 560 * Math.Sqrt(35 * 35 + 515 * 515)));
            if (double.IsNaN(gamma))
            {
                gamma = 0;
            }
            if (double.IsNaN(alpha))
            {
                alpha = Math.PI;
            }
            if (basePos == BasePosition.front)
            {
                theta2u = -(beta + gamma);
                theta2d = -(beta - gamma);
                theta3u = Math.PI + wristOffset - alpha;
                theta3d = -(Math.PI - alpha - wristOffset);
            }
            else
            {
                theta2u = -Math.PI + (beta - gamma);
                theta2d = -Math.PI + (beta + gamma);
                theta3u = Math.PI + wristOffset - alpha;
                theta3d = -Math.PI + alpha + wristOffset;
            }
            if (Math.Abs(lastVal[1] - theta2u) < Math.Abs(lastVal[1] - theta2d))
            {
                theta2 = theta2u;
                theta3 = theta3u;
                elbow = ElbowPosition.up;
            }
            else
            {
                theta2 = theta2d;
                theta3 = theta3d;
                elbow = ElbowPosition.down;
            }

            if (elbow == ElbowPosition.up)
            {
                if (theta2u > (-190.0 * Math.PI / 180) && theta2u < (1.0 * Math.PI / 4) && theta3u < (156.0 * Math.PI / 180) && theta3u > (-120.0 * Math.PI / 180))
                {
                    theta2 = theta2u;
                    theta3 = theta3u;
                }
                else if ((theta2d > (-190.0 * Math.PI / 180) && theta2d < (1.0 * Math.PI / 4)) && (theta3d < (156.0 * Math.PI / 180) && (theta3d > (-120.0 * Math.PI / 180))))
                {
                    elbow = ElbowPosition.down;
                    theta2 = theta2d;
                    theta3 = theta3d;
                }
                else
                {
                    throw new InverseKinematicsException("Out of workspace");
                }
            }
            else if (elbow == ElbowPosition.down)
            {
                if ((theta2d > (-190.0 * Math.PI / 180) && theta2d < (1.0 * Math.PI / 4)) && (theta3d < (156.0 * Math.PI / 180) && (theta3d > (-120.0 * Math.PI / 180))))
                {
                    theta2 = theta2d;
                    theta3 = theta3d;
                }
                else if (theta2u > (-190.0 * Math.PI / 180) && theta2u < (1.0 * Math.PI / 4) && theta3u < (156.0 * Math.PI / 180) && theta3u > (-120.0 * Math.PI / 180))
                {
                    elbow = ElbowPosition.up;
                    theta2 = theta2u;
                    theta3 = theta3u;
                }
                else
                {
                    throw new InverseKinematicsException("Out of workspace");
                }
            }
            else
            {

                if (theta2u > (-190.0 * Math.PI / 180) && theta2u < (1.0 * Math.PI / 4) && theta3u < (156.0 * Math.PI / 180) && theta3u > (-120.0 * Math.PI / 180))
                {
                    elbow = ElbowPosition.up;
                    theta2 = theta2u;
                    theta3 = theta3u;
                }
                else if ((theta2d > (-190.0 * Math.PI / 180) && theta2d < (1.0 * Math.PI / 4)) && (theta3d < (156.0 * Math.PI / 180) && (theta3d > (-120.0 * Math.PI / 180))))
                {
                    elbow = ElbowPosition.down;
                    theta2 = theta2d;
                    theta3 = theta3d;
                }
                else
                {
                    throw new InverseKinematicsException("Out of workspace");
                }
            }
            return new double[] { theta1, theta2, theta3 };
        }

        public double[] IKSolver(Pose DesiredPose, Vector3 EE, double[] thetaLast, ref ElbowPosition elbow, ref BasePosition basePos)
        {

            double theta1, theta2, theta3, theta4, theta5, theta6;
            double[] angles1to3 = IK1to3(DesiredPose, EE, thetaLast, ref elbow, ref basePos);
            theta1 = angles1to3[0];
            theta2 = angles1to3[1];
            theta3 = angles1to3[2];

            double[,] r = DesiredPose.getMatrix;
            double[,] T30 = new double[,] { { Math.Sin(theta2 + theta3) * Math.Cos(theta1),     -Math.Sin(theta2 + theta3) * Math.Sin(theta1),  Math.Cos(theta2 + theta3),  -400 * Math.Cos(theta2 + theta3) - 25 * Math.Sin(theta2 + theta3) - 560 * Math.Sin(theta3) }, 
                                            { Math.Cos(theta2 + theta3) * Math.Cos(theta1),     -Math.Cos(theta2 + theta3) * Math.Sin(theta1),  -Math.Sin(theta2 + theta3), 400 * Math.Sin(theta2 + theta3) - 25 * Math.Cos(theta2 + theta3) - 560 * Math.Cos(theta3) }, 
                                            { Math.Sin(theta1), Math.Cos(theta1), 0, 0 }, { 0, 0, 0, 1 } };

            double[,] T3t = SF.multiplyMatrix(T30, r);
            /*
                theta4 = Math.Atan2(-T3t[2, 2], -T3t[0, 2]);
            
            double s1 = Math.Sin(theta1);
            double c1 = Math.Cos(theta1);
            double s2 = Math.Sin(theta2);
            double c2 = Math.Cos(theta2);
            double s3 = Math.Sin(theta3);
            double c3 = Math.Cos(theta3);
            double s23 = Math.Sin(theta2+theta3);
            double c23 = Math.Cos(theta2+theta3);
            double s4 = Math.Sin(theta4);
            double c4 = Math.Cos(theta4);

            double s5 = -(r[0,2]*(c1*c23*c4+s1*s4)+r[1,2]*(s1*c23*c4-c1*s4)-r[2,2]*s23*c4);
            double c5 = r[0,2]*((-c1)*s23)+r[1,2]*(-s1)*s23+r[2,2]*(-c23);

             */
            //theta5 = Math.Atan2(s5, c5);
            if (Math.Abs(T3t[1, 1]) < 1e-6 && Math.Abs(T3t[1, 0]) < 1e-6)
            {
                // Singularity! set angles on last known theta4
                theta4 = thetaLast[3];
                theta5 = 0;
                theta6 = Math.Atan2(-T3t[0, 1], T3t[2, 1]) - thetaLast[3];
            }
            else
            {
                theta4 = Math.Atan2(-T3t[2, 2], -T3t[0, 2]);
                while (Math.Abs(thetaLast[3] - theta4) > 1.0 * Math.PI / 2)
                {
                    theta4 = (thetaLast[3] < theta4) ? theta4 - Math.PI : theta4 + Math.PI;
                }
                theta6 = Math.Atan2(-T3t[1, 1], -T3t[1, 0]);
                while (Math.Abs(thetaLast[5] - theta6) > 1.0 * Math.PI / 2)
                {
                    theta6 = (thetaLast[5] < theta6) ? (theta6 - Math.PI) : (theta6 + Math.PI);
                }
                // if (Math.Abs(thetaLast[5] - theta6) > 1.0 * Math.PI/2)
                //   {
                //       theta6 = (thetaLast[5] < theta6) ? theta6 -  Math.PI : theta6 +  Math.PI;
                //  }
                theta5 = (Math.Abs(Math.Cos(theta4)) > Math.Abs(Math.Sin(theta4))) ? Math.Atan2((-1.0 * T3t[0, 2] / (Math.Cos(theta4))), T3t[1, 2]) : Math.Atan2((-1.0 * T3t[2, 2] / (Math.Sin(theta4))), T3t[1, 2]);
                // double theta5b = (Math.Abs(Math.Cos(theta6)) > Math.Abs(Math.Sin(theta6))) ? Math.Atan2((-1.0 * T3t[1, 0] / (Math.Cos(theta6))), T3t[1, 2]) : Math.Atan2((-1.0 * T3t[1, 1] / (Math.Sin(theta6))), T3t[1, 2]);
                //  theta5 = (Math.Abs(thetaLast[4] - theta5a) < Math.Abs(thetaLast[4] - theta5b)) ? theta5a : theta5b;

            }
            if (theta4 > (185.0 * Math.PI / 180) || theta4 < (-185.0 * Math.PI / 180))
            {
                throw new InverseKinematicsException("Out of workspace");
            }
            if (theta5 > (120.0 * Math.PI / 180) || theta5 < (-120.0 * Math.PI / 180))
            {
                throw new InverseKinematicsException("Out of workspace");
            }
            if (theta5 > (350.0 * Math.PI / 180) || theta5 < (-350.0 * Math.PI / 180))
            {
                throw new InverseKinematicsException("Out of workspace");
            }
            return new double[] { theta1, theta2, theta3, theta4, theta5, theta6 };
        }

        public void updateSignal(double a, double b, long Ipoc)
        {
            if (a == 65)
            {
                // Robot has not moved
            }
            if (b == 3)
            {
                // Robot Drives are on
            }
            else if (b == 4)
            {
                // Robot Drives are off
            }
        }

        public void updateRobotAngles(double a1, double a2, double a3, double a4, double a5, double a6, long Ipoc)
        {
            _Angles.Enqueue(new double[]{a1,a2,a3,a4,a5,a6});
            if (_isConnected)
            {
                jacobianTimer.Restart();
                getLinkTransforms(a1, a2, a3, a4, a5, a6, _EndEffector, out _T, out _T0);
                _Jacobian = Jacobian(_T, _T0);
                _InverseJacobian = InverseJacobian(1e-12);

                jacobianTimer.Stop();
                double newTime = jacobianTimer.Elapsed.TotalMilliseconds;
                JocTimer.Enqueue(newTime);
                double[] jocTimes = JocTimer.ThreadSafeToArray;
                double[] maxJoc = MaxJocTimer.ThreadSafeToArray;
                _JacobienAverTimes = jocTimes.Average();
                if (newTime > maxJoc.Average() || Math.Abs(newTime - maxJoc.Average() ) < 0.1)
                {
                    MaxJocTimer.Enqueue(newTime);
                }
            }
        }

        public void updateServerTime(double t)
        {
            serverTimer.Enqueue(t);
            if (t > 3 )//|| Math.Abs(t - MaxserverTimer.ToArray().Average()) < 0.01)
            {
                MaxserverTimer.Enqueue(t);
            }
        }


        public void updateRobotTorque(double a1, double a2, double a3, double a4, double a5, double a6, long Ipoc)
        {
            _Torque.Enqueue(new TimeCoordinate(a1, a2, a3, a4, a5, a6, Ipoc));
        }

        public void LoadedCommand()
        {
            lock (trajectoryLock)
            {
                _newCommandLoaded = true;
            }
        }

        /// <summary>
        /// Loads the _desiredPosition data in Base coordinates and the _desiredRotation in local SartPose coordinates. 
        /// The _desiredRotation is a rotation, which when applied, will rotate the current pose to desired final pose.
        /// </summary>
        public void LoadCommand()
        {
            lock (trajectoryLock)
            {
                try
                {
                    if (_newCommandLoaded)
                    {
                        if (_newPosesLoaded)
                        {
                            _TrajectoryHandler.LodeBuffer(currentPose, currentVelocity);
                          //  _TrajectoryList = new TrajectoryOld[_NewTrajectoryList.Length];
                           // _NewTrajectoryList.CopyTo(_TrajectoryList, 0);
                          //  _NewTrajectoryList = null;
                           // _CurrentTrajectory = _TrajectoryList[0];
                            _newPosesLoaded = false;
                            _newOrientationLoaded = false;
                            _newPositionLoaded = false;
                            _newCommandLoaded = false;
                            _isCommanded = true;
                            _isCommandedPosition = true;
                            _isCommandedOrientation = true;
                        //   _CurrentTrajectory.start();
                        }
                        if (_newOrientationLoaded)
                        {

                            // long orientationDuration = (long)(TimeSpan.TicksPerSecond * (_desiredPose.angle / (MaxOrientationDisplacement * 10)));
                            if (_newPositionLoaded)
                            {
                                _CurrentTrajectory.load(0, _newPose, _Position.LastElement.Pose, _velocity.LastElement.Pose, new Pose(currentPose.Orientation, new Vector3(0,0,0)));
                                _newOrientationLoaded = false;
                                _newPositionLoaded = false;
                                _newCommandLoaded = false;
                                _isCommanded = true;
                                _isCommandedPosition = true;
                            }
                            else
                            {
                                _CurrentTrajectory.load(-1, _newPose, _Position.LastElement.Pose, _velocity.LastElement.Pose, new Pose(currentPose.Orientation, new Vector3(0, 0, 0)));
                                _newOrientationLoaded = false;
                                _newCommandLoaded = false;
                                _isCommanded = true;
                            }
                            _isCommandedOrientation = true;
                        }
                        else if (_newPositionLoaded)
                        {
                            _CurrentTrajectory.load(1, _newPose, _Position.LastElement.Pose, _velocity.LastElement.Pose, new Pose(currentPose.Orientation, new Vector3(0, 0, 0)));
                            _newPositionLoaded = false;
                            _newCommandLoaded = false;
                            _isCommanded = true;
                            _isCommandedPosition = true;
                        }



                    }
                }
                catch (Exception e)
                {
                    updateError("Exception " + e.Message, e);
                    _newCommandLoaded = false;
                    _isCommanded = false;
                    _newOrientationLoaded = false;
                    _newPositionLoaded = false;
                    _isCommandedPosition = false;
                    _isCommandedOrientation = false;
                }
            }
        }
        /*
        public void LoadTrajectory()
        {
            lock (trajectoryLock)
            {
                try
                {
                    if (_newCommandLoaded && _isCommanded)
                    {
                        Vector3 currentAcc = new Vector3((float)_acceleration["X"], (float)_acceleration["Y"], (float)_acceleration["Z"]);
                        Matrix tempFinalPose = Matrix.CreateFromQuaternion(_DesiredRotation);
                        tempFinalPose.Translation = new Vector3((float)_DesiredPosition["X"], (float)_DesiredPosition["Y"], (float)_DesiredPosition["Z"]);
                        _CurrentTrajectory = new Trajectory(tempFinalPose, this);
                        _CurrentTrajectory.Start(currentPose, currentAcc);
                        _newCommandLoaded = false;
                        _isCommanded = true;
                    }
                    else if (_newCommandLoaded)
                    {
                        Matrix tempFinalPose = Matrix.CreateFromQuaternion(_DesiredRotation);
                        tempFinalPose.Translation = new Vector3((float)_DesiredPosition["X"], (float)_DesiredPosition["Y"], (float)_DesiredPosition["Z"]);
                        _CurrentTrajectory = new Trajectory(tempFinalPose, this);
                        _CurrentTrajectory.Start(currentPose);
                        _newCommandLoaded = false;
                        _isCommanded = true;
                    }

                }
                catch (Exception)
                {
                    _CurrentTrajectory = new Trajectory(currentPose, this);
                    _CurrentTrajectory.Stop();
                    _newCommandLoaded = false;
                    _isCommanded = false;
                }
            }
        }

         * 
         */

        public void goHome()
        {
            _TrajectoryHandler.goHome(new Trajectory[] { new JointLinearTrajectory(_Angles.LastElement, homeAngles, _MaxAxisChange, Guid.NewGuid()) });
        }

        public bool newPoses(int n, Pose[] new_Poses, double[] AverageVelocity)
        {
            try
            {
                
            if (!_newPosesLoaded)
            {
                Guid PoseList = Guid.NewGuid();
                TrajectoryQuintic[] QuinticTrajectories = new TrajectoryQuintic[n];
                Stopwatch trajectoryLoader = new Stopwatch();
                trajectoryLoader.Start();
               //  Check if no veloicty was specified or if mm/s or mm/ms was specified. Must used mm/ms for trajectory generation
                AverageVelocity[0] = (AverageVelocity[0] == -1) ? 1.0 * _maxLinearVelocity / 2 : ((AverageVelocity[0] > 0.1) ? 1.0 * AverageVelocity[0] / 1000 : AverageVelocity[0]);
                for (int i = 1; i < AverageVelocity.Length; i++)
                {
                    AverageVelocity[i] = (AverageVelocity[i] == -1) ? AverageVelocity[i - 1] : ((AverageVelocity[i] > 0.1) ? 1.0 * AverageVelocity[i] / 1000 : AverageVelocity[i]);
                }
                //_NewTrajectoryList = new TrajectoryOld[n];
                Vector3[] PointVelocitys = new Vector3[n+1];
                PointVelocitys[0] = Vector3.Zero;
                PointVelocitys[n] = Vector3.Zero; 
                PointVelocitys[1] = (float)((AverageVelocity[0] + 0.2 * AverageVelocity[1]) / 1.2) * (Vector3.Normalize(Vector3.Normalize(new_Poses[0].Translation - currentPose.Translation) + Vector3.Normalize(new_Poses[1].Translation - new_Poses[0].Translation)));
                
                for (int i = 2; i < n ; i++)
                {
                    PointVelocitys[i] = (float)((AverageVelocity[i - 1] + 0.2 * AverageVelocity[i]) / 1.2) * (Vector3.Normalize(Vector3.Normalize(new_Poses[i-1].Translation - new_Poses[i - 2].Translation) + Vector3.Normalize(new_Poses[i].Translation - new_Poses[i-1].Translation)));
                }
                //_NewTrajectoryList[0] = new TrajectoryOld(new_Poses[0], AverageVelocity[0], currentPose, PointVelocitys[0], PointVelocitys[1]);
                QuinticTrajectories[0] = new TrajectoryQuintic(new_Poses[0], AverageVelocity[0], currentPose, PointVelocitys[0], PointVelocitys[1], PoseList);
                for (int i = 1; i < n; i++)
                {
                    //_NewTrajectoryList[i] = new TrajectoryOld(new_Poses[i], AverageVelocity[i], new_Poses[i - 1], PointVelocitys[i], PointVelocitys[i + 1]);
                    QuinticTrajectories[i] = new TrajectoryQuintic(new_Poses[i], AverageVelocity[i], new_Poses[i - 1], PointVelocitys[i], PointVelocitys[i + 1], PoseList);
                }
                _TrajectoryHandler.Load(QuinticTrajectories);
                _newPosesLoaded = true;
                trajectoryLoader.Stop();
                _trajectoryLoaderTime.Enqueue(trajectoryLoader.Elapsed.TotalMilliseconds);
                return true;
            }
            }
            catch (Exception e)
            {
                updateError("Error loading: " + e.Message, e);
            }
            return false;
        }

        public void newPosition(double x, double y, double z)
        {
            lock (trajectoryLock)
            {
                _newPose.Translation = new Vector3((float)x, (float)y, (float)z);
                _newPositionLoaded = true;
            }
        }

        public void newRotation(float x1, float x2, float x3, float y1, float y2, float y3, float z1, float z2, float z3)
        {
            lock (trajectoryLock)
            {
                _newPose.Orientation = Quaternion.CreateFromRotationMatrix(new Matrix(x1, x2, x3, 0, y1, y2, y3, 0, z1, z2, z3, 0, 0, 0, 0, 1));
                _newOrientationLoaded = true;

            }
        }


        public void newConOrientation(float xx, float xy, float xz, float zx, float zy, float zz)
        {
            lock (trajectoryLock)
            {
                Vector3 xAxis = new Vector3(xx, xy, xz);
                Vector3 zAxis = new Vector3(zx, zy, zz);
                Vector3 yAxis = Vector3.Cross(zAxis, xAxis);

                xAxis.Normalize();
                yAxis.Normalize();
                zAxis.Normalize();
                Quaternion newOrientation = Quaternion.CreateFromRotationMatrix(new Matrix(xAxis.X, xAxis.Y, xAxis.Z, 0, yAxis.X, yAxis.Y, yAxis.Z, 0, zAxis.X, zAxis.Y, zAxis.Z, 0, 0, 0, 0, 1));
                _newPose.Orientation = new Quaternion(newOrientation.X, newOrientation.Y, newOrientation.Z, newOrientation.W);
                _newOrientationLoaded = true;
            }
        }

        public void newConOrientation(float x, float y, float z)
        {
            lock (trajectoryLock)
            {
                Vector3 newOrientation = new Vector3(x, y, z);
                newOrientation = Vector3.Normalize(newOrientation);
                _newOrientationLoaded = setupController(newOrientation, ref _newPose);
            }
        }

        /// <summary>
        /// Updates the desired rotation with a quaternion representing a change from Base to the final orientation, EEVector
        /// The quiternion updated is in global frame
        /// </summary>
        /// <param name="EEvector"></param>
        /// <param name="DesiredRotationOut"></param>
        /// <returns></returns>
        bool setupController(Vector3 EEvector, ref Pose DesiredRotationOut)
        {
            Quaternion _currentOrientation = _Position.LastElement.Orientation;
            Matrix _currentPose = Matrix.CreateFromQuaternion(_currentOrientation);
            Vector3 axis = Vector3.Cross(Vector3.Normalize(_currentPose.Backward), Vector3.Normalize(EEvector));
            float angle = (float)Math.Asin((double)axis.Length());
            if (Math.Abs(angle) < MathHelper.ToRadians(0.2f))
            {
                return false;
            }
            else
            {
                DesiredRotationOut.Orientation = Quaternion.CreateFromAxisAngle(Vector3.Normalize(axis), angle) * _currentOrientation;
                return true;
            }
            // The output is the transform from Base to final.
        }

        double[,] getTipInverseJacobian(double[,] InvWristJacobian, Vector3 EE)
        {
            double[,] InvSkewEE = new double[,] { { 1, 0, 0, 0, -EE.Z, EE.Y }, { 0, 1, 0, EE.Z, 0, -EE.X }, { 0, 0, 1, -EE.Y, EE.X, 0 }, { 0, 0, 0, 1, 0, 0 }, { 0, 0, 0, 0, 1, 0 }, { 0, 0, 0, 0, 0, 1 } };
            return SF.multiplyMatrix(InvWristJacobian,InvSkewEE);
        }

        double[] checkLimits(double[] axisComand)
        {
            double[] saturatedComand = new double[axisComand.Length];
            for (int i = 0; i < axisComand.Length; i++)
            {
                saturatedComand[i] = (Math.Abs(axisComand[i]) > _MaxAxisChange) ? Math.Sign(axisComand[i]) * _MaxAxisChange : axisComand[i];
            }
            return saturatedComand;
        }

        public void updateComandPosition()
        {
            lock (trajectoryLock)
            {

                if (_isConnected && _isCommanded && _TrajectoryHandler.IsActive)
                {
                    double[] axisComand = _TrajectoryHandler.RobotChange(currentPose, currentVelocity, _InverseJacobian);
                    _axisCommand = checkLimits(axisComand);
                    _CommandPose = new TimeCoordinate(new Pose(_axisCommand), _Position.LastElement.Ipoc);
                    /*
                    Pose CurrentPose = currentPose;

                //    TGetReference = IPOC.Elapsed.TotalMilliseconds;

                    _Reference = _CurrentTrajectory.getReferencePosition(CurrentPose);
                    _referenceVelocity = _CurrentTrajectory.getReferenceVelocity(CurrentPose);
               //     PGetReference = IPOC.Elapsed.TotalMilliseconds - TGetReference;

             //       TGetController = IPOC.Elapsed.TotalMilliseconds;

             //       T2 = IPOC.Elapsed.TotalMilliseconds;
                    double[,] InvWristJacobian = SF.InverseJacobian(_Angles.LastElement,1e-5);
            //        P2 = IPOC.Elapsed.TotalMilliseconds - T2;

             //       T2 = IPOC.Elapsed.TotalMilliseconds;
                    double[,] inverseJoc = getTipJacobian(InvWristJacobian, _EndEffector);
            //        P3 = IPOC.Elapsed.TotalMilliseconds - T2;

          //          T2 = IPOC.Elapsed.TotalMilliseconds;
                    _axisCommand = _Controller.getControllerEffort(_Reference, _referenceVelocity, CurrentPose, currentVelocity, inverseJoc);
           //         P4 = IPOC.Elapsed.TotalMilliseconds - T2;

          //          PGetController = IPOC.Elapsed.TotalMilliseconds - TGetController;

                    Pose commandPose = new Pose(_axisCommand);
                   // Pose commandPose = _CurrentTrajectory.reference(currentPose, currentVelocity, _maxLinearVelocity, (float)_maxAngularVelocity, _maxLinearAcceleration, _maxAngularAcceleration);
                   // Vector3 comandPos = _CurrentTrajectory.getDisplacement(currentPose.Translation, MaxDisplacement);
                   // Vector3 commandOri = _CurrentTrajectory.getOrientation(currentRotation, (float)MaxOrientationDisplacement);
                    // Update the command position all lights green
                   // Vector3 kukaAngles = SF.getKukaAngles(commandPose.Orientation);
                    _CommandPose = new TimeCoordinate(commandPose, _Position.LastElement.Ipoc);
                    if (_CurrentTrajectory.checkFinish(CurrentPose, 1e-2))
                    {
                        if (_currentSegment < _TrajectoryList.Length)
                        {

                            _CurrentTrajectory.stop();
                            _CurrentTrajectory = _TrajectoryList[_currentSegment];
                            _CurrentTrajectory.start();
                            _currentSegment++;
                            updateError("Segment Finished\nCurrent Segment: " + _currentSegment.ToString());
                        }
                        else
                        {
                            _currentSegment = 1;
                            _isCommanded = false;
                            _CurrentTrajectory.stop();
                        }
                    }
                     * 
                     */
                }


                    /*
                else if (_isRotating)
                {
                    Vector3 commandOri = Vector3.Zero;
                    if (_rotatingTimer.Elapsed.TotalMilliseconds > 6000.0)
                    {
                        resetRotation();
                    }
                    if (_isRotatingX)
                    {
                        commandOri.X = _degreePerSec * 4 / 1000;
                    }
                    if (_isRotatingY)
                    {
                        commandOri.Y = _degreePerSec * 4 / 1000;
                    }
                    if (_isRotatingZ)
                    {
                        commandOri.Z = _degreePerSec * 4 / 1000;
                    }

                    _CommandedPosition["A"] = commandOri.X;
                    _CommandedPosition["B"] = commandOri.Y;
                    _CommandedPosition["C"] = commandOri.Z;
                }
                     * 
                     */
                else
                {
                    // End condition, or disconnected half way command position is zero
                    flushCommands();
                }

            }
        }

        private void resetRotation()
        {
            flushCommands();
        }


        public void flushCommands()
        {
            _CommandPose = new TimeCoordinate(0, 0, 0, 0, 0, 0, _Position.LastElement.Ipoc);
            _axisCommand = new double[] { 0, 0, 0, 0, 0, 0 };
        }

        #endregion
        /*
        #region Dictionary Setup
        
        void setupCardinalDictionaries(ConcurrentDictionary<string, double> dic)
        {
            for (int i = 0; i < 6; i++)
            {
                if (!dic.TryAdd(SF.getCardinalKey(i), 0))
                {
                    dic[SF.getCardinalKey(i)] = 0;
                }
            }
        }
        void setupCardinalDictionaries(ConcurrentDictionary<string, double> dic, double[] Values)
        {
            for (int i = 0; i < Values.Length; i++)
            {
                if (!dic.TryAdd(SF.getCardinalKey(i), Values[i]))
                {
                    dic[SF.getCardinalKey(i)] = Values[i];
                }
            }
        }

        void setupAxisDictionaries(ConcurrentDictionary<string, double> dic)
        {
            for (int i = 0; i < 6; i++)
            {
                if (!dic.TryAdd(SF.getAxisKey(i), 0))
                {
                    dic[SF.getAxisKey(i)] = 0;
                }
            }
        }    
        #endregion
        */
    }
}
