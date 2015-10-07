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

        Stopwatch _KukaCycleTime = new Stopwatch();

        // Time of loop in SECONDS
        double _loopTime = 0;
        double _processDataTimer = 0;
        double _maxProcessDataTimer = 0;

        FixedSizedQueue<TimeCoordinate> _Position = new FixedSizedQueue<TimeCoordinate>(6);
        FixedSizedQueue<TimeCoordinate> _velocity = new FixedSizedQueue<TimeCoordinate>(6);
        FixedSizedQueue<TimeCoordinate> _acceleration = new FixedSizedQueue<TimeCoordinate>(6);
        FixedSizedQueue<TimeCoordinate> _Torque = new FixedSizedQueue<TimeCoordinate>(6);
        FixedSizedQueue<TimeCoordinate> _Angles = new FixedSizedQueue<TimeCoordinate>(6);
        ConcurrentQueue<TimeCoordinate> _DesiredPose;
        Pose _newPose;
        TimeCoordinate _CurrentDesiredPose;
        TimeCoordinate _CommandPose;

        // Thread safe lists for updating and storing of robot information.
        // public ConcurrentStack<StateObject> DataHistory;

        ConcurrentDictionary<String, StringBuilder> _text = new ConcurrentDictionary<string, StringBuilder>();

        //Trajectory _CurrentTrajectory;
        Trajectory _CurrentTrajectory;

        bool _gripperIsOpen = true;

        readonly double _MaxCartesianChange = 0.8;
        readonly double _MaxAngularChange = 0.1;

        double _maxLinearVelocity = .12; // in mm/ms
        double _maxAngularVelocity = .012; // in mm/ms
        float _maxLinearAcceleration = 0.005f;// in mm/ms2
        float _maxAngularAcceleration = 0.00005f; // in deg/ms2

        bool _isConnected = false;
        bool _isCommanded = false;
        bool _isCommandedPosition = false;
        bool _isCommandedOrientation = false;

        bool _newCommandLoaded = false;
        bool _newOrientationLoaded = false;
        bool _newPositionLoaded = false;
        bool _isVia = false;

        public double[] _axisCommand = new double[] { 0, 0, 0, 0, 0, 0 };
        bool _A1axis = false, _A2axis = false, _A3axis = false, _A4axis = false, _A5axis = false, _A6axis = false;
        Stopwatch A1 = new Stopwatch(), A2 = new Stopwatch();
        TimeSpan A3 = TimeSpan.Zero, A4 = TimeSpan.Zero, A5 = TimeSpan.Zero, A6 = TimeSpan.Zero;

        // Test variables
        bool _isRotatingX = false;
        bool _isRotatingY = false;
        bool _isRotatingZ = false;
        bool _isRotating = false;
        Stopwatch _rotatingTimer = new Stopwatch();
        Stopwatch _ConnectionTimer = new Stopwatch();

        StringBuilder _DisplayMsg = new StringBuilder();
        int _errorMsgs = 0;

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
            _CurrentTrajectory = new Trajectory();
            _text.TryAdd("msg", new StringBuilder());
            _text.TryAdd("Error", new StringBuilder());
            _text.TryAdd("Controller", new StringBuilder());
            _Position.Enqueue(new TimeCoordinate(540.5, -18.1, 833.3, 180.0, 0.0, 180.0, 0));
            _velocity.Enqueue(new TimeCoordinate(540.5, -18.1, 833.3, 180.0, 0.0, 180.0, 0));
            _acceleration.Enqueue(new TimeCoordinate(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0));
            _Torque.Enqueue(new TimeCoordinate(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0));
            _Angles.Enqueue(new TimeCoordinate(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0));
            _CurrentDesiredPose = new TimeCoordinate(540.5, -18.1, 833.3, 180.0, 0.0, 180.0, 0);
            _CommandPose = new TimeCoordinate(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);

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
            if (!_isConnected)
            {
                _ConnectionTimer.Restart();
            }
            _isConnected = true;
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
            TimeCoordinate _DisplayCommandPosition = _CommandPose;
            Vector3 currentZAxis = _DisplayPosition.Pose.zAxis;
            Vector3 desiredZAxis = _DisplayDesiredPosition.Pose.zAxis;
            Vector3 currentVelocity = _DisplayVelocity.Pose.Velocity;

            Console.Clear();
            Console.WriteLine("---------------------------------\n              Info:\n");
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
            Console.WriteLine("Command Position:     (" + String.Format("{0:0.00}", _DisplayCommandPosition.x) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayCommandPosition.y) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayCommandPosition.z) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayCommandPosition.a) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayCommandPosition.b) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayCommandPosition.c) + ")");

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
            Console.WriteLine("Process data time: " + _processDataTimer.ToString() + "ms.");
            Console.WriteLine("Max Process data time: " + _maxProcessDataTimer.ToString() + "ms.");
            Console.WriteLine("Kuka cycle time: " + _loopTime.ToString() + "ms.");


        }
        #endregion

        #region Movement
        public void updateRobotPosition(long LIpoc, long Ipoc, double x, double y, double z, double a, double b, double c)
        {
            //updateError(x.ToString() + " : " + y.ToString() + " : " + z.ToString() + " : " + a.ToString() + " : " + b.ToString() + " : " + c.ToString());
            _KukaCycleTime.Stop();
            if (_KukaCycleTime.ElapsedTicks == 0)
            {
                _loopTime = 1.0 / 250;
            }
            else
            {
                if (Math.Abs(_loopTime - 1.0 * _KukaCycleTime.ElapsedTicks / TimeSpan.TicksPerSecond) > 5.0 / 1000)
                {
                    _loopTime = 1.0 / 250;
                }
                else
                {
                    _loopTime = 1.0 * _KukaCycleTime.ElapsedTicks / TimeSpan.TicksPerSecond;
                }

            }
            _KukaCycleTime.Restart();

            TimeCoordinate newPosition = new TimeCoordinate(x, y, z, a, b, c, Ipoc);
            _Position.Enqueue(newPosition);
            TimeCoordinate[] positions = _Position.ToArray();
            _velocity.Enqueue(SF.AverageRateOfChange(positions));
            TimeCoordinate[] velocities = _velocity.ToArray();
            _acceleration.Enqueue(SF.AverageRateOfChange(velocities));
        }

        public TimeCoordinate forwardKinimatics(double a1, double a2, double a3, double a4, double a5, double a6, long Ipoc)
        {
            double s1 = Math.Sin(a1 * Math.PI / 180);
            double c1 = Math.Cos(a1 * Math.PI / 180);
            double s2 = Math.Sin(a2 * Math.PI / 180);
            double c2 = Math.Cos(a2 * Math.PI / 180);
            double s3 = Math.Sin(a3 * Math.PI / 180);
            double c3 = Math.Cos(a3 * Math.PI / 180);
            double s4 = Math.Sin(a4 * Math.PI / 180);
            double c4 = Math.Cos(a4 * Math.PI / 180);
            double s5 = Math.Sin(Math.PI / 2 + a5 * Math.PI / 180);
            double c5 = Math.Cos(Math.PI / 2 + a5 * Math.PI / 180);
            double s6 = Math.Sin(a6 * Math.PI / 180);
            double c6 = Math.Cos(a6 * Math.PI / 180);
            double m11 = s6 * (s4 * (s1 * s3 + c1 * c2 * c3) - c1 * c4 * s2) + c6 * (c5 * (c4 * (s1 * s3 + c1 * c2 * c3) + c1 * s2 * s4) - s5 * (c3 * s1 - c1 * c2 * s3));
            double m12 = c6 * (s4 * (s1 * s3 + c1 * c2 * c3) - c1 * c4 * s2) - s6 * (c5 * (c4 * (s1 * s3 + c1 * c2 * c3) + c1 * s2 * s4) - s5 * (c3 * s1 - c1 * c2 * s3));
            double m13 = s5 * (c4 * (s1 * s3 + c1 * c2 * c3) + c1 * s2 * s4) + c5 * (c3 * s1 - c1 * c2 * s3);
            double m14 = 25 * c1 + 560 * c1 * c2 + 515 * c3 * s1 + 35 * s1 * s3 + 35 * c1 * c2 * c3 - 515 * c1 * c2 * s3;
            double m21 = s6 * (s4 * (c1 * s3 - c2 * c3 * s1) + c4 * s1 * s2) + c6 * (c5 * (c4 * (c1 * s3 - c2 * c3 * s1) - s1 * s2 * s4) - s5 * (c1 * c3 + c2 * s1 * s3));
            double m22 = c6 * (s4 * (c1 * s3 - c2 * c3 * s1) + c4 * s1 * s2) - s6 * (c5 * (c4 * (c1 * s3 - c2 * c3 * s1) - s1 * s2 * s4) - s5 * (c1 * c3 + c2 * s1 * s3));
            double m23 = s5 * (c4 * (c1 * s3 - c2 * c3 * s1) - s1 * s2 * s4) + c5 * (c1 * c3 + c2 * s1 * s3);
            double m24 = 515 * c1 * c3 - 25 * s1 - 560 * c2 * s1 + 35 * c1 * s3 + 515 * c2 * s1 * s3 - 35 * c2 * c3 * s1;
            double m31 = c6 * (c5 * (c2 * s4 - c3 * c4 * s2) - s2 * s3 * s5) - s6 * (c2 * c4 + c3 * s2 * s4);
            double m32 = -s6 * (c5 * (c2 * s4 - c3 * c4 * s2) - s2 * s3 * s5) - c6 * (c2 * c4 + c3 * s2 * s4);
            double m33 = c2 * Math.Cos(a5 * Math.PI / 180) * s4 - s2 * s3 * Math.Sin(a5 * Math.PI / 180) - c3 * c4 * Math.Cos(a5 * Math.PI / 180) * s2;
            double m34 = 515 * s2 * s3 - 35 * c3 * s2 - 560 * s2 + 400;
            double m41 = 0;
            double m42 = 0;
            double m43 = 0;
            double m44 = 1;
            Matrix M = new Matrix((float)m11, (float)m12, (float)m13, (float)m14, (float)m21, (float)m22, (float)m23, (float)m24, (float)m31, (float)m32, (float)m33, (float)m34, (float)m41, (float)m42, (float)m43, (float)m44);
            return new TimeCoordinate(M.Translation.X, M.Translation.Y, M.Translation.Z, Quaternion.CreateFromRotationMatrix(Matrix.Transpose(M)), Ipoc);
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
        Pose[] getLinkTransforms(double t1, double t2, double t3, double t4, double t5, double t6)
        {
            Matrix T01 = new Matrix((float)Math.Cos(t1), (float)-Math.Sin(t1), 0, 0, (float)-Math.Sin(t1), (float)-Math.Cos(t1), 0, 0, 0, 0, -1, 400, 0, 0, 0, 1);
            Matrix T12 = new Matrix((float)Math.Cos(t2), (float)-Math.Sin(t2), 0, 25, 0, 0, -1, 0, (float)Math.Sin(t2), (float)Math.Cos(t2), 0, 0, 0, 0, 0, 1);
            Matrix T23 = new Matrix((float)Math.Sin(t3), (float)Math.Cos(t3), 0, 560, (float)-Math.Cos(t3), (float)Math.Sin(t3), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
            Matrix T34 = new Matrix((float)Math.Cos(t4), (float)-Math.Sin(t4), 0, 35, 0, 0, -1, 515, (float)Math.Sin(t4), (float)Math.Cos(t4), 0, 0, 0, 0, 0, 1);
            Matrix T45 = new Matrix((float)Math.Cos(t5), (float)-Math.Sin(t5), 0, 0, 0, 0, 1, 0, (float)-Math.Sin(t5), (float)-Math.Cos(t5), 0, 0, 0, 0, 0, 1);
            Matrix T56 = new Matrix((float)Math.Cos(t6), (float)-Math.Sin(t6), 0, 0, 0, 0, -1, 0, (float)Math.Sin(t6), (float)Math.Cos(t6), 0, 0, 0, 0, 0, 1);
            Matrix T67 = new Matrix(1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, -80, 0, 0, 0, 1);
            return new Pose[] { new Pose(Matrix.Transpose(T01)), new Pose(Matrix.Transpose(T12)), new Pose(Matrix.Transpose(T23)), new Pose(Matrix.Transpose(T34)), new Pose(Matrix.Transpose(T45)), new Pose(Matrix.Transpose(T56)), new Pose(Matrix.Transpose(T67)) };
        }


        double[,] Jacobian(Matrix[] T, Matrix[] T0)
        {
            // TODO: return jacobian
            return new double[1, 2];
        }


        public Pose forwardKinimatics(double a1, double a2, double a3, double a4, double a5, double a6)
        {
            double s1 = Math.Sin(a1);
            double c1 = Math.Cos(a1);
            double s2 = Math.Sin(a2);
            double c2 = Math.Cos(a2);
            double s23 = Math.Sin(a2-Math.PI/2+a3);
            double c23 = Math.Cos(a2-Math.PI/2+a3);
            double s3 = Math.Sin(a3);
            double c3 = Math.Cos(a3);
            double s3p = Math.Sin(a3-Math.PI/2);
            double c3p = Math.Cos(a3-Math.PI/2);
            double s4 = Math.Sin(a4);
            double c4 = Math.Cos(a4);
            double s45 = Math.Sin(a4 + a5);
            double s4m5 = Math.Sin(a4 - a5);
            double s5 = Math.Sin(a5);
            double c5 = Math.Cos(a5);
            double s6 = Math.Sin(a6);
            double c6 = Math.Cos(a6);
            double a = (c4 * s1 + s4 * (c1 * s2 * s3p - c1 * c2 * c3p));
            double b1 = (s1 * s4 - c4 * (c1 * s2 * s3p - c1 * c2 * c3p));
            double b2 = (c1 * c2 * s3p + c1 * c3p * s2);
            double b = (c5 * b1 - s5 * b2);
            double m11 = s6 * a + c6 * b;
            double m12 = s6 * b - c6 * a;
            double m13 = -s5 * b1 - c5 * b2;
            double m14 = 25 * c1 + 560 * c1 * c2 - 515 * c1 * s2 * s3 - 80 * s1 * s4 * s5 + 515 * c1 * c2 * c3 + 35 * c1 * c2 * s3 + 35 * c1 * c3 * s2 + 80 * c1 * c2 * c3 * c5 - 80 * c1 * c5 * s2 * s3 - 80 * c1 * c2 * c4 * s3 * s5 - 80 * c1 * c3 * c4 * s2 * s5;
            a = (c1 * c4 + s4 * (c2 * c3p * s1 - s1 * s2 * s3p));
            b1 = (c1 * s4 - c4 * (c2 * c3p * s1 - s1 * s2 * s3p));
            b2 = (c2 * s1 * s3p + c3p * s1 * s2);
            b = (c5 * b1 + s5 * b2);
            double m21 = s6 * a + c6 * b;
            double m22 = s6 * b - c6 * a;
            double m23 = c5 * b2 - s5 * b1;
            double m24 = 515 * s1 * s2 * s3 - 560 * c2 * s1 - 35 * c2 * s1 * s3 - 35 * c3 * s1 * s2 - 80 * c1 * s4 * s5 - 25 * s1 - 515 * c2 * c3 * s1 - 80 * c2 * c3 * c5 * s1 + 80 * c5 * s1 * s2 * s3 + 80 * c2 * c4 * s1 * s3 * s5 + 80 * c3 * c4 * s1 * s2 * s5;
            a = (c2 * s3p + c3p * s2);
            b1 = (c2 * c3p - s2 * s3p);
            b = (s5 * b1 + c4 * c5 * a);
            double m31 = s4 * s6 * a - c6 * b;
            double m32 = -s6 * b - c6 * s4 * a;
            double m33 = c4 * s5 * a - c5 * b1;
            double m34 = 40 * s23 * s45 - 35 * s23 - 560 * s2 - 515 * c23 - 80 * c23 * c5 - 40 * s4m5 * s23 + 400;
            double m41 = 0;
            double m42 = 0;
            double m43 = 0;
            double m44 = 1;
            Matrix M = new Matrix((float)m11, (float)m12, (float)m13, (float)m14, (float)m21, (float)m22, (float)m23, (float)m24, (float)m31, (float)m32, (float)m33, (float)m34, (float)m41, (float)m42, (float)m43, (float)m44);
            return new Pose(Matrix.Transpose(M));
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
            _Angles.Enqueue(new TimeCoordinate(forwardKinimatics(a1 * Math.PI / 180, a2 * Math.PI / 180, a3 * Math.PI / 180, a4 * Math.PI / 180, a5 * Math.PI / 180, a6 * Math.PI / 180), Ipoc));
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
                catch (Exception)
                {
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

        public void updateComandPosition()
        {
            lock (trajectoryLock)
            {

                if (_isConnected && _isCommanded && _CurrentTrajectory.IsActive)
                {
                    Pose commandPose = _CurrentTrajectory.reference(currentPose, currentVelocity, _maxLinearVelocity, (float)_maxAngularVelocity, _maxLinearAcceleration, _maxAngularAcceleration);
                   // Vector3 comandPos = _CurrentTrajectory.getDisplacement(currentPose.Translation, MaxDisplacement);
                   // Vector3 commandOri = _CurrentTrajectory.getOrientation(currentRotation, (float)MaxOrientationDisplacement);
                    // Update the command position all lights green
                    Vector3 kukaAngles = SF.getKukaAngles(commandPose.Orientation);
                    _CommandPose = new TimeCoordinate(commandPose, _Position.LastElement.Ipoc);
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
