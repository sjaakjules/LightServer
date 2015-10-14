using Microsoft.Xna.Framework;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace LightWeight_Server
{
    class RobotInfo
    {
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
        double _processDataTimer = 0;
        double _maxProcessDataTimer = 0;

        FixedSizedQueue<TimeCoordinate> _Position = new FixedSizedQueue<TimeCoordinate>(6);
        FixedSizedQueue<TimeCoordinate> _velocity = new FixedSizedQueue<TimeCoordinate>(6);
        FixedSizedQueue<TimeCoordinate> _acceleration = new FixedSizedQueue<TimeCoordinate>(6);
        FixedSizedQueue<TimeCoordinate> _Torque = new FixedSizedQueue<TimeCoordinate>(6);
        FixedSizedQueue<double[]> _Angles = new FixedSizedQueue<double[]>(6);
        Pose[] _T = new Pose[7];
        Matrix[] _T0 = new Matrix[7];
        double[,] _Jacobian = new double[6, 6];
        ConcurrentQueue<TimeCoordinate> _DesiredPose;
        Pose _newPose;
        Pose[] _newPoses;
        double[] _newVelocitys;
        TrajectoryOld[] _TrajectoryList, _NewTrajectoryList;

        TimeCoordinate _CurrentDesiredPose;
        TimeCoordinate _CommandPose;

        Pose _StartPose, _StartTipPose;
        Pose _EndEffectorPose;
        Pose _Reference;
        Vector3 _EndEffector;

        // Thread safe lists for updating and storing of robot information.
        // public ConcurrentStack<StateObject> DataHistory;

        ConcurrentDictionary<String, StringBuilder> _text = new ConcurrentDictionary<string, StringBuilder>();

        //Trajectory _CurrentTrajectory;
        TrajectoryOld _CurrentTrajectory;
        int _currentSegment = 1;
        Controller _Controller;

        bool _gripperIsOpen = true;

        readonly double _MaxCartesianChange = 0.8;
        readonly double _MaxAngularChange = 0.1;

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
        bool _A1axis = false, _A2axis = false, _A3axis = false, _A4axis = false, _A5axis = false, _A6axis = false;
        Stopwatch A1 = new Stopwatch(), A2 = new Stopwatch(), jacobianTimer = new Stopwatch();
        TimeSpan A3 = TimeSpan.Zero, A4 = TimeSpan.Zero, A5 = TimeSpan.Zero, A6 = TimeSpan.Zero;

        FixedSizedQueue<double> JocTimer = new FixedSizedQueue<double>(10);
        FixedSizedQueue<double> MaxJocTimer = new FixedSizedQueue<double>(10);
        FixedSizedQueue<double> serverTimer = new FixedSizedQueue<double>(10);
        FixedSizedQueue<double> MaxserverTimer = new FixedSizedQueue<double>(10);
        double _JacobienAverTimes = 0, _trajectoryLoaderTime = 0;
        // Test variables
        bool _isRotatingX = false;
        bool _isRotatingY = false;
        bool _isRotatingZ = false;
        bool _isRotating = false;
        Stopwatch _rotatingTimer = new Stopwatch();
        Stopwatch _ConnectionTimer = new Stopwatch();

        StringBuilder _DisplayMsg = new StringBuilder();
        int _errorMsgs = 0;

        double TGetReference, TGetController, TloadTrajectory, T1, T2, T3;
        public double PGetReference, PGetController, PloadTrajectory, P1, P2, P3, P4;

        public readonly double[] homePosition = new double[] { 540.5, -18.1, 833.3, 180.0, 0.0, 180.0 };


        #region Properties

        public bool A1axis
        {
            get
            {
                lock (axisRotateLock)
                {
                    return _A1axis;
                }
            }
            set
            {
                lock (axisRotateLock)
                {
                    _A1axis = value;
                }
            }
        }

        public bool A2axis
        {
            get
            {
                lock (axisRotateLock)
                {
                    return _A2axis;
                }
            }
            set
            {
                lock (axisRotateLock)
                {
                    _A2axis = value;
                }
            }
        }

        bool A3axis
        {
            get
            {
                lock (axisRotateLock)
                {
                    return _A3axis;
                }
            }
            set
            {
                lock (axisRotateLock)
                {
                    _A3axis = value;
                }
            }
        }

        bool A4axis
        {
            get
            {
                lock (axisRotateLock)
                {
                    return _A4axis;
                }
            }
            set
            {
                lock (axisRotateLock)
                {
                    _A4axis = value;
                }
            }
        }

        bool A5axis
        {
            get
            {
                lock (axisRotateLock)
                {
                    return _A5axis;
                }
            }
            set
            {
                lock (axisRotateLock)
                {
                    _A5axis = value;
                }
            }
        }

        bool A6axis
        {
            get
            {
                lock (axisRotateLock)
                {
                    return _A6axis;
                }
            }
            set
            {
                lock (axisRotateLock)
                {
                    _A6axis = value;
                }
            }
        }

        public double ProcessDataTimer
        {
            get { return _processDataTimer; }
            set
            {
                if (_maxProcessDataTimer <= value)
                {
                    _maxProcessDataTimer = value;
                }
                if (value > 13)
                {
                    updateError("Took too long!!! :(");
                }
                _processDataTimer = value;
            }
        }

        public TimeCoordinate commandPose
        {
            get { return _CommandPose; }
        }
        /*
        // inb degrees
        public double[] currentDoublePose { get { return SF.getCardinalDoubleArray(_ReadPosition); } }

        Matrix currentPose
        {
            get
            {
                return SF.MakeMatrixFromKuka(currentDoublePose);
            }
        }

        public Quaternion currentRotation { get { return SF.MakeQuaternionFromKuka(currentDoublePose); } }
        */

        public Pose currentPose
        {
            get { return _Position.LastElement.Pose; }
        }

        public Pose currentVelocity
        {
            get { return _velocity.LastElement.Pose; }
        }

        public Pose currentAcceleration
        {
            get { return _acceleration.LastElement.Pose; }
        }

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
                        _maxLinearVelocity = value / 1000;
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
                        _maxAngularVelocity = value / 1000;
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

        public bool rotateX
        {
            get
            {
                return _isRotatingX;
            }
            set
            {
                _isRotatingX = value;
            }
        }
        public bool rotateY
        {
            get
            {
                return _isRotatingY;
            }
            set
            {
                _isRotatingY = value;
            }
        }
        public bool rotateZ
        {
            get
            {
                return _isRotatingZ;
            }
            set
            {
                _isRotatingZ = value;
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

        public RobotInfo()
        {
            _CurrentTrajectory = new TrajectoryOld();
            _Controller = new Controller(this);
            _text.TryAdd("msg", new StringBuilder());
            _text.TryAdd("Error", new StringBuilder());
            _text.TryAdd("Controller", new StringBuilder());
            _Position.Enqueue(new TimeCoordinate(540.5, -18.1, 833.3, 180.0, 0.0, 180.0, 0));
            _velocity.Enqueue(new TimeCoordinate(540.5, -18.1, 833.3, 180.0, 0.0, 180.0, 0));
            _acceleration.Enqueue(new TimeCoordinate(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0));
            _Torque.Enqueue(new TimeCoordinate(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0));
            _Angles.Enqueue(new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
            _CurrentDesiredPose = new TimeCoordinate(540.5, -18.1, 833.3, 180.0, 0.0, 180.0, 0);
            _CommandPose = new TimeCoordinate(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);
            MaxJocTimer.Enqueue(0);
            MaxserverTimer.Enqueue(0);

            /*
            setupCardinalDictionaries(_ReadPosition, homePosition);
            setupCardinalDictionaries(_DesiredPosition, homePosition);
            setupCardinalDictionaries(_LastPosition);
            setupCardinalDictionaries(_Velocity);
            setupCardinalDictionaries(_LastVelocity);
            setupCardinalDictionaries(_acceleration);
            setupCardinalDictionaries(_CommandedPosition);

            setupAxisDictionaries(_Torque);
            */
            _text["Error"].Append("---------------------------------\n             Errors:\n");
        }

        public void Connect()
        {
            if (_isConnecting)
            {
                IPOC.Start();
                double[] startAngles = _Angles.LastElement;
                _StartPose = forwardKinimatics(startAngles, Vector3.Zero);
                _StartTipPose = _Position.LastElement.Pose;
                _EndEffectorPose = Pose.inverse(_StartPose) * _StartTipPose ;
                _EndEffector = _EndEffectorPose.Translation;
                getLinkTransforms(startAngles[0], startAngles[1], startAngles[2], startAngles[3], startAngles[4], startAngles[5], _EndEffector, out _T, out _T0);
                _isConnected = true;
            }
            if (!_isConnected)
            {
                _ConnectionTimer.Restart();
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
                    _processDataTimer = 0;
                    _maxProcessDataTimer = 0; bool hasMsg = false;
                    StringBuilder finalError = new StringBuilder();
                    while (!hasMsg)
                    {
                        hasMsg = _text.TryGetValue("Error", out finalError);
                    }
                    _errorMsgs++;
                    StreamWriter errormsg = new StreamWriter("Error_" + _errorMsgs.ToString() + ".txt");
                    errormsg.Write(finalError.ToString());
                    errormsg.Flush();
                    errormsg.Close();
                    _text["Error"].Clear();
                    _text["Error"].Append("---------------------------------\n             Errors:\n");

                }
                catch (Exception e)
                {

                }

            }
        }

        /*
        void resetKukaInfo()
        {
            if (!_isCommanded)
            {
                gripperIsOpen = true;
                setupCardinalDictionaries(_ReadPosition, _homePosition);
                setupCardinalDictionaries(_DesiredPosition, _homePosition);
                _maxSpeed = 0.5;
                loopTime = 0;
                processDataTimer = 0;
                maxProcessDataTimer = 0;
            }
            else
            {
                loopTime = 0;
                processDataTimer = 0;
                maxProcessDataTimer = 0;
                setupCardinalDictionaries(_ReadPosition, _homePosition);
            }
        }
        */

        #region ScreenDisplay

        public void updateError(string newError)
        {
            _text["Error"].AppendLine(newError);
        }

        // Dedicated loop thread
        public void UpdateScreen()
        {
            while (true)
            {
                try
                {
                    bool hasMsg = false;
                    while (!hasMsg)
                    {
                        hasMsg = _text.TryGetValue("Error", out _DisplayMsg);
                    }

                    StreamWriter file = new StreamWriter("ErrorMsg" + _errorMsgs.ToString() + ".txt");

                    file.WriteLine(_DisplayMsg);
                    file.Flush();
                    file.Close();

                    /*
                    
                    hasMsg = false;
                    while (!hasMsg)
                    {
                        hasMsg = _text.TryGetValue("Controller", out _PrintMsg);
                    }
                    file = new StreamWriter("Control.csv");
                    file.WriteLine(_PrintMsg);
                    file.Flush();
                    file.Close();
                    */
                    //Console.WriteLine(_PrintMsg.ToString());

                    if (_isConnected)
                    {
                        updateMsg();
                        /*
                        hasMsg = false;
                        while (!hasMsg)
                        {
                            hasMsg = _text.TryGetValue("msg", out _DisplayMsg);
                            Console.Clear();
                            Console.WriteLine(_DisplayMsg.ToString());
                        }
                         * 
                         */
                    }
                    else
                    {
                        Console.Clear();
                        Console.WriteLine("---------------------------------\n   Not Connected to Kuka Robot");
                        TimeCoordinate currentTimePose = _Position.LastElement;
                        Vector3 currentEndEffector = currentTimePose.Pose * new Vector3(0, 0, 1);
                        Console.WriteLine("IPOC: {0}", currentTimePose.Ipoc);
                        Console.WriteLine("Position: ({0} , {1} , {2}) \n"
                                         + "Angle:    ({3} , {4} , {5})\n"
                                         + "Tip:      ({6} , {7} , {8})", currentTimePose.x, currentTimePose.y, currentTimePose.z, currentTimePose.a, currentTimePose.b, currentTimePose.c, currentEndEffector.X, currentEndEffector.Y, currentEndEffector.Z);
                    }
                    if (_KukaCycleTime.ElapsedMilliseconds > 5000)
                    {
                        Disconnect();
                    }
                    System.Threading.Thread.Sleep(100);
                }
                catch (Exception e)
                {
                    Console.WriteLine("Error printing to Screen");
                    Console.WriteLine(e.Message);
                    _errorMsgs++;
                }

            }
        }

        void updateMsg()
        {
            /*
            double[] displayAxis = new double[6];
            lock (axisComandLock)
            {
                for (int i = 0; i < 6; i++)
                {
                    displayAxis[i] = _axisCommand[i];
                }
            }
             * 
             */
            TimeCoordinate _DisplayPosition = _Position.LastElement;
            TimeCoordinate _DisplayVelocity = _velocity.LastElement;
            TimeCoordinate _DisplayAcceleration = _acceleration.LastElement;
            TimeCoordinate _DisplayTorque = _Torque.LastElement;
            TimeCoordinate _DisplayDesiredPosition = _CurrentDesiredPose;
            double[] _DisplayCommandPosition = _axisCommand;
            double[] _DisplayAngle = _Angles.LastElement;
            Vector3 currentZAxis = _DisplayPosition.Pose.zAxis;
            Vector3 desiredZAxis = _DisplayDesiredPosition.Pose.zAxis;
            Vector3 currentVelocity = _DisplayVelocity.Pose.Velocity;

            Console.Clear();
            Console.WriteLine("---------------------------------\n              Info:\n");
            Console.WriteLine("Number of Angles: " + _Angles.ToArray().Length.ToString());
            Console.WriteLine("Current Angles:       (" + String.Format("{0:0.000}", _DisplayAngle[0]) + " , " +
                                                                    String.Format("{0:0.000}", _DisplayAngle[1]) + " , " +
                                                                    String.Format("{0:0.000}", _DisplayAngle[2]) + " , " +
                                                                    String.Format("{0:0.000}", _DisplayAngle[3]) + " , " +
                                                                    String.Format("{0:0.000}", _DisplayAngle[4]) + " , " +
                                                                    String.Format("{0:0.000}", _DisplayAngle[5]) + ")");

            plotPose(forwardKinimatics(_DisplayAngle, _EndEffector), "FK Position:            (");
            plotPose(_Reference, "Reference Position:     (");
            plotPose(currentPose, "Measured Position:      (");
            Console.WriteLine("Current Position:     (" + String.Format("{0:0.00}", _DisplayPosition.x) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayPosition.y) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayPosition.z) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayPosition.a) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayPosition.b) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayPosition.c) + ")");
            Console.WriteLine("Tip Vector:           (" + String.Format("{0:0.00}", currentZAxis.X) + " , " +
                                                                        String.Format("{0:0.00}", currentZAxis.Y) + " , " +
                                                                        String.Format("{0:0.00}", currentZAxis.Z) + ")");
            Console.WriteLine("Current Velocity:     (" + String.Format("{0:0.00}", _DisplayVelocity.x) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayVelocity.y) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayVelocity.z) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayVelocity.a) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayVelocity.b) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayVelocity.c) + " , " + ")");
            Console.WriteLine("Axis Velocity:        (" + String.Format("{0:0.00}", currentVelocity.X) + " , " +
                                                                        String.Format("{0:0.00}", currentVelocity.Y) + " , " +
                                                                        String.Format("{0:0.00}", currentVelocity.Z) + ") Total: " +
                                                                    String.Format("{0:0.00}", _DisplayVelocity.angle));
            Console.WriteLine("Desired Position:     (" + String.Format("{0:0.00}", _DisplayDesiredPosition.x) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayDesiredPosition.y) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayDesiredPosition.z) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayDesiredPosition.a) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayDesiredPosition.b) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayDesiredPosition.c) + ")");
            Console.WriteLine("Desired Tip Vector:   (" + String.Format("{0:0.00}", desiredZAxis.X) + " , " +
                                                                        String.Format("{0:0.00}", desiredZAxis.Y) + " , " +
                                                                        String.Format("{0:0.00}", desiredZAxis.Z) + ")");
            Console.WriteLine("Command Position:     (" + String.Format("{0:0.00}", _DisplayCommandPosition[0]) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayCommandPosition[1]) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayCommandPosition[2]) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayCommandPosition[3]) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayCommandPosition[4]) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayCommandPosition[5]) + ")");

            plotPose(_EndEffectorPose, "End Effector Position:     (");
            /*
            Console.WriteLine("Command Axis:         (" + String.Format("{0:0.0000}", displayAxis[0]) + " , " +
                                                                    String.Format("{0:0.0000}", displayAxis[1]) + " , " +
                                                                    String.Format("{0:0.0000}", displayAxis[2]) + " , " +
                                                                    String.Format("{0:0.0000}", displayAxis[3]) + " , " +
                                                                    String.Format("{0:0.0000}", displayAxis[4]) + " , " +
                                                                    String.Format("{0:0.0000}", displayAxis[5]) + ")");
             * 
             * 
             */


            Console.WriteLine("Linear Velocity: " + LinearVelocity.ToString() + "mm per ms");
            Console.WriteLine("Angular Velocity: " + AngularVelocity.ToString() + "mm per ms");
            Console.WriteLine("Linear Acceleration: " + LinearAcceleration.ToString() + "mm per ms2");
            Console.WriteLine("Angular Acceleration: " + AngularAcceleration.ToString() + "mm per ms2");
            if (gripperIsOpen)
            {
                Console.WriteLine("Gripper is OPEN.");
            }
            else
            {
                Console.WriteLine("Gripper is CLOSED");
            }
            if (_isCommanded)
            {
                Console.WriteLine("Robot is Commanded");
            }
            else
            {
                Console.WriteLine("Robot is NOT Commanded");
            }
            if (_CurrentTrajectory.isRotating)
            {
                Console.WriteLine("You spin me right round baby... right round....");
            }
            else
            {
                Console.WriteLine("no rotating");
            }
            Console.WriteLine("Process data time:       " + _processDataTimer.ToString() + "ms.");
            Console.WriteLine("Max Process data time:   " + _maxProcessDataTimer.ToString() + "ms.");
            Console.WriteLine("Kuka cycle time:         " + _loopTime.ToString() + "ms.");
            Console.WriteLine("Average Trajectory time: {0}", _trajectoryLoaderTime);
            Console.WriteLine("Reference time:          {0}", PGetReference);
            Console.WriteLine("Controller time:         {0}", PGetController);
            Console.WriteLine("Control loop time:       {0}", P1);
            Console.WriteLine("Inverse Wrist loop time: {0}", P2);
            Console.WriteLine("Inverse Jacobian time:   {0}", P3);
            Console.WriteLine("Control axis loop time:  {0}", P4);
            Console.WriteLine("Average Jacobian time:   {0}", _JacobienAverTimes);
            Console.WriteLine("Maximum Jacobian time: \n(");
            foreach (var item in MaxJocTimer.ToArray())
            {
                Console.Write(" {0} ", item.ToString());
            }
            Console.WriteLine(")");
            Console.WriteLine("server time:   \n(");
            foreach (var item in serverTimer.ToArray())
            {
                Console.Write(" {0} ", item.ToString());
            }
            Console.WriteLine(")");
            Console.WriteLine("Maximum server time:   \n(");
            foreach (var item in MaxserverTimer.ToArray())
            {
                Console.Write(" {0} ", item.ToString());
            }
            Console.WriteLine(")");


        }
        #endregion
        void plotPose(Pose pose, string msg)
        {
            Vector3 kukaAngles = SF.getKukaAngles(pose.Orientation);
            Console.WriteLine(msg + String.Format("{0:0.00}", pose.Translation.X) + " , " +
                                                                    String.Format("{0:0.00}", pose.Translation.Y) + " , " +
                                                                    String.Format("{0:0.00}", pose.Translation.Z) + " , " +
                                                                    String.Format("{0:0.00}", kukaAngles.X) + " , " +
                                                                    String.Format("{0:0.00}", kukaAngles.Y) + " , " +
                                                                    String.Format("{0:0.00}", kukaAngles.Z) + ")");

        }

        #region Movement
        public void updateRobotPosition(long LIpoc, long Ipoc, double x, double y, double z, double a, double b, double c)
        {
            //updateError(x.ToString() + " : " + y.ToString() + " : " + z.ToString() + " : " + a.ToString() + " : " + b.ToString() + " : " + c.ToString());
            _KukaCycleTime.Stop();
            _loopTime = _KukaCycleTime.Elapsed.TotalMilliseconds;
            _KukaCycleTime.Restart();

            TimeCoordinate newPosition = new TimeCoordinate(x, y, z, a, b, c, Ipoc);
            _Position.Enqueue(newPosition);
            TimeCoordinate[] positions = _Position.ToArray();
            _velocity.Enqueue(SF.AverageRateOfChange(positions));
            TimeCoordinate[] velocities = _velocity.ToArray();
            _acceleration.Enqueue(SF.AverageRateOfChange(velocities));
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
            double[,] InvWristJacobian = SF.InverseJacobian(_Angles.LastElement, error);
            return getTipJacobian(InvWristJacobian, _EndEffector);
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
            double s3p = Math.Sin(a3 - Math.PI / 2);
            double c3p = Math.Cos(a3 - Math.PI / 2);
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

        public void updateSignal(int a, int b, long Ipoc)
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
                double[,] invJoc = InverseJacobian(1e-6);
               // Mat Jocob = new Mat(_Jacobian);
              //  double[,] invJocob = Jocob.Inverse(1e-4).getArray;
                jacobianTimer.Stop();
                double newTime = jacobianTimer.Elapsed.TotalMilliseconds;
                JocTimer.Enqueue(newTime);
                double[] jocTimes = JocTimer.ToArray();
                double[] maxJoc = MaxJocTimer.ToArray();
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
            if (t > 4 )//|| Math.Abs(t - MaxserverTimer.ToArray().Average()) < 0.01)
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
                            _TrajectoryList = new TrajectoryOld[_NewTrajectoryList.Length];
                            _NewTrajectoryList.CopyTo(_TrajectoryList, 0);
                            _NewTrajectoryList = null;
                            _CurrentTrajectory = _TrajectoryList[0];
                            _newPosesLoaded = false;
                            _newOrientationLoaded = false;
                            _newPositionLoaded = false;
                            _newCommandLoaded = false;
                            _isCommanded = true;
                            _isCommandedPosition = true;
                            _isCommandedOrientation = true;
                            _CurrentTrajectory.start();
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
                    // New test code:
                    if (!_isRotating && (_isRotatingX || _isRotatingY || _isRotatingZ))
                    {
                        _rotatingTimer.Restart();
                        _isRotating = true;
                    }
                }
                catch (Exception e)
                {
                    updateError("Exception ");
                    updateError(e.Message);
                    updateError(e.StackTrace);
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
        public bool newPoses(int n, Pose[] new_Poses, double[] AverageVelocity)
        {
            if (!_newPosesLoaded)
            {
                Stopwatch trajectoryLoader = new Stopwatch();
                trajectoryLoader.Start();
                // Check if no veloicty was specified or if mm/s or mm/ms was specified. Must used mm/ms for trajectory generation
                AverageVelocity[0] = (AverageVelocity[0] == -1) ? _maxLinearVelocity / 2 : ((AverageVelocity[0] > 0.1) ? AverageVelocity[0] / 1000 : AverageVelocity[0]);
                for (int i = 1; i < AverageVelocity.Length; i++)
                {
                    AverageVelocity[i] = (AverageVelocity[i] == -1) ? AverageVelocity[i-1] : ((AverageVelocity[i] > 0.1) ? AverageVelocity[i] / 1000 : AverageVelocity[i]);
                }
                _NewTrajectoryList = new TrajectoryOld[n];
                Vector3[] PointVelocitys = new Vector3[n + 1];
                PointVelocitys[0] = Vector3.Zero;
                PointVelocitys[n] = Vector3.Zero;
                for (int i = 1; i < n - 1; i++)
                {
                    PointVelocitys[i] = (float)((AverageVelocity[i - 1] + 0.2 * AverageVelocity[i]) / 1.2) * (Vector3.Normalize(Vector3.Normalize(new_Poses[i].Translation - new_Poses[i - 1].Translation) + Vector3.Normalize(new_Poses[i + 1].Translation - new_Poses[i].Translation)));
                }
                _NewTrajectoryList[0] = new TrajectoryOld(new_Poses[0], AverageVelocity[0], currentPose, PointVelocitys[0], PointVelocitys[1]);
                for (int i = 1; i < n; i++)
                {
                    _NewTrajectoryList[i] = new TrajectoryOld(new_Poses[i], AverageVelocity[i], new_Poses[i - 1], PointVelocitys[i], PointVelocitys[i + 1]);
                }
                _newPosesLoaded = true;
                trajectoryLoader.Stop();
                _trajectoryLoaderTime = trajectoryLoader.Elapsed.TotalMilliseconds;
                return true;
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

        double[,] getTipJacobian(double[,] InvWristJacobian, Vector3 EE)
        {
            double[,] InvSkewEE = new double[,] { { 1, 0, 0, 0, -EE.Z, EE.Y }, { 0, 1, 0, EE.Z, 0, -EE.X }, { 0, 0, 1, -EE.Y, EE.X, 0 }, { 0, 0, 0, 1, 0, 0 }, { 0, 0, 0, 0, 1, 0 }, { 0, 0, 0, 0, 0, 1 } };
            return SF.multiplyMatrix(InvWristJacobian,InvSkewEE);
        }

        public void updateComandPosition()
        {
            T1 = IPOC.Elapsed.TotalMilliseconds;
            lock (trajectoryLock)
            {

                if (_isConnected && _isCommanded && _CurrentTrajectory.IsActive)
                {
                    Pose CurrentPose = currentPose;

                //    TGetReference = IPOC.Elapsed.TotalMilliseconds;

                    _Reference = _CurrentTrajectory.getReferencePosition(CurrentPose);

               //     PGetReference = IPOC.Elapsed.TotalMilliseconds - TGetReference;

             //       TGetController = IPOC.Elapsed.TotalMilliseconds;

             //       T2 = IPOC.Elapsed.TotalMilliseconds;
                    double[,] InvWristJacobian = SF.InverseJacobian(_Angles.LastElement,1e-5);
            //        P2 = IPOC.Elapsed.TotalMilliseconds - T2;

             //       T2 = IPOC.Elapsed.TotalMilliseconds;
                    double[,] inverseJoc = getTipJacobian(InvWristJacobian, _EndEffector);
            //        P3 = IPOC.Elapsed.TotalMilliseconds - T2;

          //          T2 = IPOC.Elapsed.TotalMilliseconds;
                    _axisCommand = _Controller.getControllerEffort(_Reference, _CurrentTrajectory.getReferenceVelocity(CurrentPose), CurrentPose, currentVelocity, inverseJoc);
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
                            _CurrentTrajectory = _TrajectoryList[_currentSegment];
                            _currentSegment++;
                        }
                        else
                        {
                            _currentSegment = 1;
                            _isCommanded = false;
                            _CurrentTrajectory.stop();
                        }
                    }
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
            P1 = IPOC.Elapsed.TotalMilliseconds - T1;
                
                if (A1axis && A1.Elapsed.TotalMilliseconds == 0)
                {
                    _axisCommand[4] = 0.01;
                    _axisCommand[3] = 0.01;
                    _axisCommand[5] = -0.01;
                    A1axis = false;
                    A1.Restart();
                }

                if (A2axis && A2.Elapsed.TotalMilliseconds == 0)
                {
                    _axisCommand[4] = -0.01;
                    _axisCommand[3] = -0.01;
                    _axisCommand[5] = 0.01;
                    A1axis = false;
                    A2.Restart();
                }
                if (A2.Elapsed.TotalMilliseconds > 5000)
                {
                    A2.Stop();
                    A2.Reset();
                    _axisCommand = new double[] { 0, 0, 0, 0, 0, 0 };
                }
                if (A1.Elapsed.TotalMilliseconds > 5000)
                {
                    A1.Stop();
                    A1.Reset();
                    _axisCommand = new double[] { 0, 0, 0, 0, 0, 0 };
                }
            

        }

        private void resetRotation()
        {
            flushCommands();
            _rotatingTimer.Reset();
            _isRotating = false;
            _isRotatingX = false;
            _isRotatingY = false;
            _isRotatingZ = false;
        }


        public void flushCommands()
        {
            _CommandPose = new TimeCoordinate(0, 0, 0, 0, 0, 0, _Position.LastElement.Ipoc);
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
