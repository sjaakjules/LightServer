﻿using Microsoft.Xna.Framework;
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
        object maxSpeedLock = new object();
        object maxOrientationSpeedLock = new object();
        object maxDisplacementLock = new object();
        object desiredAxisLock = new object();

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
        TimeCoordinate _DesiredPose;
        TimeCoordinate _CommandPose;
        TimeCoordinate _DisplayPosition, _DisplayVelocity, _DisplayAcceleration, _DisplayTorque, _DisplayDesiredPosition, _DisplayCommandPosition;

        //ConcurrentDictionary<String, double> _ReadPosition = new ConcurrentDictionary<string, double>();
        //ConcurrentDictionary<String, double> _LastPosition = new ConcurrentDictionary<string, double>();
        //ConcurrentDictionary<String, double> _Velocity = new ConcurrentDictionary<string, double>();
        //ConcurrentDictionary<String, double> _LastVelocity = new ConcurrentDictionary<string, double>();
        //ConcurrentDictionary<String, double> _acceleration = new ConcurrentDictionary<string, double>();
        //ConcurrentDictionary<String, double> _Torque = new ConcurrentDictionary<string, double>();
        //ConcurrentDictionary<String, double> _DesiredPosition = new ConcurrentDictionary<string, double>();
        //ConcurrentDictionary<String, double> _CommandedPosition = new ConcurrentDictionary<string, double>();
        //Quaternion _DesiredRotation = Quaternion.Identity;
        Vector3 desiredZaxis = new Vector3();
        // Thread safe lists for updating and storing of robot information.
        // public ConcurrentStack<StateObject> DataHistory;

        ConcurrentDictionary<String, StringBuilder> _text = new ConcurrentDictionary<string, StringBuilder>();

        //Trajectory _CurrentTrajectory;
        Trajectory _CurrentTrajectory;

        bool _gripperIsOpen = true;
        double _maxSpeed = 30;
        double _maxDisplacement = .5;
        double _maxOrientationSpeed = .05;

        bool _isConnected = false;
        bool _isCommanded = false;
        bool _isCommandedPosition = false;
        bool _isCommandedOrientation = false;

        bool _newCommandLoaded = false;
        bool _newOrientationLoaded = false;
        bool _newPositionLoaded = false;


        // Test variables
        bool _isRotatingX = false;
        bool _isRotatingY = false;
        bool _isRotatingZ = false;
        bool _isRotating = false;
        Stopwatch _rotatingTimer = new Stopwatch();
        Stopwatch _ConnectionTimer = new Stopwatch();
      //  StringBuilder _data = new StringBuilder();

        float _degreePerSec = 5;


        StringBuilder _DisplayMsg = new StringBuilder();
        int _errorMsgs = 0;

        public readonly double[] homePosition = new double[] { 540.5, -18.1, 833.3, 180.0, 0.0, 180.0 };


        #region Properties
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

        public double MaxOrientationDisplacement
        {
            get
            {
                lock (maxOrientationSpeedLock)
                {
                    return _maxOrientationSpeed;
                }
            }
            set
            {
                lock (maxOrientationSpeedLock)
                {
                    _maxOrientationSpeed = value;
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
        public float rotateSpeed
        {
            get
            {
                lock (maxOrientationSpeedLock)
                {
                    return _degreePerSec;
                }
            }
            set
            {
                lock (maxOrientationSpeedLock)
                {
                    _degreePerSec = value;
                }
            }
        }



        public double CurrentSpeed
        {
            get
            {
                lock (maxSpeedLock)
                {
                    return _maxSpeed;
                }
            }
            set
            {
                lock (maxSpeedLock)
                {
                    _maxSpeed = value;
                }
            }
        }

        public double MaxDisplacement
        {
            get
            {
                lock (maxDisplacementLock)
                {
                    return _maxDisplacement;
                }
            }
            set
            {
                lock (maxDisplacementLock)
                {
                    _maxDisplacement = value;
                }
            }
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
            _velocity.Enqueue(new TimeCoordinate(540.5, -18.1, 833.3, 180.0, 0.0, 180.0,0));
            _acceleration.Enqueue(new TimeCoordinate(0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0));
            _Torque.Enqueue(new TimeCoordinate(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0));
            _Angles.Enqueue(new TimeCoordinate(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0));
            _DesiredPose = new TimeCoordinate(540.5, -18.1, 833.3, 180.0, 0.0, 180.0, 0);
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
                    Console.Clear();
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
                        hasMsg = false;
                        while (!hasMsg)
                        {
                            hasMsg = _text.TryGetValue("msg", out _DisplayMsg);
                        }
                        Console.WriteLine(_DisplayMsg.ToString());
                    }
                    else
                    {
                        Console.WriteLine("---------------------------------\n   Not Connected to Kuka Robot");
                        TimeCoordinate currentposition = _Position.LastElement;
                        Console.WriteLine("IPOC: {0}", currentposition.Ipoc);
                        Console.WriteLine("{0} : {1} : {2} : {3} : {4} : {5}", currentposition.x, currentposition.y, currentposition.z, currentposition.a, currentposition.b, currentposition.c);
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
            _DisplayPosition = _Position.LastElement;
            _DisplayVelocity = _velocity.LastElement;
            _DisplayAcceleration = _acceleration.LastElement;
            _DisplayTorque = _Torque.LastElement;
            _DisplayDesiredPosition = _DesiredPose;
            _DisplayCommandPosition = _CommandPose;

            _text["msg"].Clear();
            _text["msg"].AppendLine("---------------------------------\n              Info:\n");
            _text["msg"].AppendLine("Current Position:     (" + String.Format("{0:0.00}", _DisplayPosition.x) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayPosition.y) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayPosition.z) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayPosition.a) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayPosition.b) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayPosition.c) + ")");
            _text["msg"].AppendLine("Current Velocity:     (" + String.Format("{0:0.00}", _DisplayVelocity.x) + " , " +
                                                                    String.Format("{0:0.00}",_DisplayVelocity.y) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayVelocity.z) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayVelocity.a) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayVelocity.b) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayVelocity.c) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayVelocity.angle) + ")");
            _text["msg"].AppendLine("Desired Position:     (" + String.Format("{0:0.00}", _DisplayDesiredPosition.x) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayDesiredPosition.y) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayDesiredPosition.z) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayDesiredPosition.a) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayDesiredPosition.b) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayDesiredPosition.c) + ")");
            _text["msg"].AppendLine("Command Position:     (" + String.Format("{0:0.00}", _DisplayCommandPosition.x) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayCommandPosition.y) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayCommandPosition.z) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayCommandPosition.a) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayCommandPosition.b) + " , " +
                                                                    String.Format("{0:0.00}", _DisplayCommandPosition.c) + ")");
            lock (desiredAxisLock)
            {
                _text["msg"].AppendLine("Desired Tip Vector:     (" + String.Format("{0:0.00}", desiredZaxis.X) + " , " +
                                                                        String.Format("{0:0.00}", desiredZaxis.Y) + " , " +
                                                                        String.Format("{0:0.00}", desiredZaxis.Z) + ")");

            }
            _text["msg"].AppendLine("Current Tip vector:     (" + String.Format("{0:0.00}", currentPose.Backward.X) + " , " +
                                                                    String.Format("{0:0.00}", currentPose.Backward.Y) + " , " +
                                                                    String.Format("{0:0.00}", currentPose.Backward.Z) + ")");

            _text["msg"].AppendLine("Max Speed: " + MaxDisplacement.ToString() + "mm per cycle");
            if (gripperIsOpen)
            {
                _text["msg"].AppendLine("Gripper is OPEN.");
            }
            else
            {
                _text["msg"].AppendLine("Gripper is CLOSED");
            }
            if (_isCommanded)
            {
                _text["msg"].AppendLine("Robot is Commanded");
            }
            else
            {
                _text["msg"].AppendLine("Robot is NOT Commanded");
            }
            if (_CurrentTrajectory._isRotating)
            {
                _text["msg"].AppendLine("You spin me right round baby... right round....");
            }
            else
            {
                _text["msg"].AppendLine("no rotating");
            }
            _text["msg"].AppendLine("Process data time: " + _processDataTimer.ToString() + "ms.");
            _text["msg"].AppendLine("Max Process data time: " + _maxProcessDataTimer.ToString() + "ms.");
            _text["msg"].AppendLine("Kuka cycle time: " + _loopTime.ToString() + "ms.");


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

        public void updateRobotAngles(double a1, double a2, double a3, double a4, double a5, double a6, long Ipoc)
        {
            _Angles.Enqueue(forwardKinimatics(a1, a2 - 90, a3 + 90, a4, a5 + 90, a6, Ipoc));
        }

        void checkStall()
        {

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
                                    _CurrentTrajectory.load(0, _DesiredPose, _Position.LastElement, _velocity.LastElement, _acceleration.LastElement);
                                    _newOrientationLoaded = false;
                                    _newPositionLoaded = false;
                                    _newCommandLoaded = false;
                                    _isCommanded = true;
                                    _isCommandedPosition = true;
                                }
                                else
                                {
                                    _CurrentTrajectory.load(-1, _DesiredPose, _Position.LastElement, _velocity.LastElement, _acceleration.LastElement);
                                    _newOrientationLoaded = false;
                                    _newCommandLoaded = false;
                                    _isCommanded = true;
                                }
                                _isCommandedOrientation = true;
                            }
                            else if (_newPositionLoaded)
                            {
                                _CurrentTrajectory.load(1, _DesiredPose, _Position.LastElement, _velocity.LastElement, _acceleration.LastElement);
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
                _DesiredPose.Translation = new Vector3((float)x, (float)y, (float)z);
                _newPositionLoaded = true;
            }
        }

        public void newRotation(float x1, float x2, float x3, float y1, float y2, float y3, float z1, float z2, float z3)
        {
            lock (trajectoryLock)
            {
                _DesiredPose.Orientation = Quaternion.CreateFromRotationMatrix(new Matrix(x1, x2, x3, 0, y1, y2, y3, 0, z1, z2, z3, 0, 0, 0, 0, 1));
                _newOrientationLoaded = true;

            }
        }

        public void newConOrientation(float x, float y, float z)
        {
            lock (trajectoryLock)
            {
                Vector3 newOrientation = new Vector3(x, y, z);
                newOrientation = Vector3.Normalize(newOrientation);
                lock (desiredAxisLock)
                {
                    desiredZaxis = newOrientation;
                }
                _newOrientationLoaded = setupController(newOrientation, ref _DesiredPose);
            }
        }

        /// <summary>
        /// Updates the desired rotation with a quaternion representing a change from Base to the final orientation, EEVector
        /// The quiternion updated is in global frame
        /// </summary>
        /// <param name="EEvector"></param>
        /// <param name="DesiredRotationOut"></param>
        /// <returns></returns>
        bool setupController(Vector3 EEvector, ref TimeCoordinate DesiredRotationOut)
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
                DesiredRotationOut.Orientation = _currentOrientation * Quaternion.CreateFromAxisAngle(Vector3.Normalize(axis), angle);
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
                    
                    Vector3 comandPos = _CurrentTrajectory.getDisplacement(currentPose.Translation, MaxDisplacement);
                    Vector3 commandOri = _CurrentTrajectory.getOrientation(currentRotation, (float)MaxOrientationDisplacement);
                    // Update the command position all lights green
                    _CommandedPosition["X"] = comandPos.X;
                    _CommandedPosition["Y"] = comandPos.Y;
                    _CommandedPosition["Z"] = comandPos.Z;
                    _CommandedPosition["A"] = commandOri.X;
                    _CommandedPosition["B"] = commandOri.Y;
                    _CommandedPosition["C"] = commandOri.Z;
                }
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
            _rotatingTimer.Reset();
            _isRotating = false;
            _isRotatingX = false;
            _isRotatingY = false;
            _isRotatingZ = false;
        }


        public void flushCommands()
        {
            for (int i = 0; i < 6; i++)
            {
                _CommandedPosition[SF.getCardinalKey(i)] = 0.0;
            }
        }

        #endregion

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


    }
}
