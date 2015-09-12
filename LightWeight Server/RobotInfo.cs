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
        FixedSizedQueue<TimeCoordinate> _DesiredPosition = new FixedSizedQueue<TimeCoordinate>(6);
        FixedSizedQueue<TimeCoordinate> _CommandPosition = new FixedSizedQueue<TimeCoordinate>(6);
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
        Controller _CurrrentController;

        bool _gripperIsOpen = true;
        double _maxSpeed = 30;
        double _maxDisplacement = .5;
        double _maxOrientationSpeed = .005;

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

        // Thread safe getter which blocks till value is given
        public double CommandedPosition(int index)
        {
            lock (trajectoryLock)
            {
                return SF.Getvalue(_CommandedPosition, SF.getCardinalKey(index));
            }
        }
        public double CurrentPosition(int index)
        {
            return SF.Getvalue(_ReadPosition, SF.getCardinalKey(index));
        }
        public double CurrentVelocity(int index)
        {
            return SF.Getvalue(_Velocity, SF.getCardinalKey(index));
        }
        public double CurrentAcceleration(int index)
        {
            return SF.Getvalue(_acceleration, SF.getCardinalKey(index));
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
            _CurrrentController = new Controller(this);
            _text.TryAdd("msg", new StringBuilder());
            _text.TryAdd("Error", new StringBuilder());
            _text.TryAdd("Controller", new StringBuilder());
            _Position.Enqueue(new TimeCoordinate(540.5, -18.1, 833.3, 180.0, 0.0, 180.0, 0));
            _velocity.Enqueue(new TimeCoordinate(540.5, -18.1, 833.3, 180.0, 0.0, 180.0,0));
            _acceleration.Enqueue(new TimeCoordinate(0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0));
            _Torque.Enqueue(new TimeCoordinate(0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0));
            _DesiredPosition.Enqueue(new TimeCoordinate(540.5, -18.1, 833.3, 180.0, 0.0, 180.0, 0));
            _CommandPosition.Enqueue(new TimeCoordinate(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0));

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
                        Console.WriteLine("{0} : {1} : {2} : {3} : {4} : {5}", _ReadPosition["X"], _ReadPosition["Y"], _ReadPosition["Z"], _ReadPosition["A"], _ReadPosition["B"], _ReadPosition["C"]);
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
            _DisplayDesiredPosition = _DesiredPosition.LastElement;
            _DisplayCommandPosition = _CommandPosition.LastElement;

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
            if (_CurrrentController._isRotating)
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
                        TimeCoordinate newCommand;
                        if (_DesiredPosition.TryDequeue(out newCommand))
                        {
                            if (_newOrientationLoaded)
                            {
                                Vector3 axis = Vector3.Zero;
                                float angle = 0;
                                SF.getAxisAngle(_DesiredRotation, ref axis, ref angle);
                                long orientationDuration = (long)(TimeSpan.TicksPerSecond * (angle / (MaxOrientationDisplacement * 10)));
                                if (_newPositionLoaded)
                                {
                                    _CurrrentController.load(new Vector3((float)_DesiredPosition["X"], (float)_DesiredPosition["Y"], (float)_DesiredPosition["Z"]), _DesiredRotation, currentPose, orientationDuration);
                                    _newOrientationLoaded = false;
                                    _newPositionLoaded = false;
                                    _newCommandLoaded = false;
                                    _isCommanded = true;
                                    _isCommandedPosition = true;
                                }
                                else
                                {
                                    _CurrrentController.load(_DesiredRotation, currentPose, orientationDuration);
                                    _newOrientationLoaded = false;
                                    _newCommandLoaded = false;
                                    _isCommanded = true;
                                }
                                _isCommandedOrientation = true;
                            }
                            else if (_newPositionLoaded)
                            {
                                _CurrrentController.load(new Vector3((float)_DesiredPosition["X"], (float)_DesiredPosition["Y"], (float)_DesiredPosition["Z"]), currentPose);
                                _newPositionLoaded = false;
                                _newCommandLoaded = false;
                                _isCommanded = true;
                                _isCommandedPosition = true;
                            }
                            
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
                    _CurrrentController.load(new Vector3((float)CurrentPosition(0), (float)CurrentPosition(1), (float)CurrentPosition(2)), Quaternion.Identity, currentPose, 100);
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
                _DesiredPosition["X"] = x;
                _DesiredPosition["Y"] = y;
                _DesiredPosition["Z"] = z;
                _newPositionLoaded = true;
            }
        }

        public void newRotation(float x1, float x2, float x3, float y1, float y2, float y3, float z1, float z2, float z3)
        {
            lock (trajectoryLock)
            {
                _DesiredRotation = Quaternion.CreateFromRotationMatrix(new Matrix(x1, x2, x3, 0, y1, y2, y3, 0, z1, z2, z3, 0, 0, 0, 0, 1));
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
                _newOrientationLoaded = setupController(newOrientation, ref _DesiredRotation);
            }
        }

        /// <summary>
        /// Updates the desired rotation with a quaternion representing a change from current pose to the final orientation, EEVector
        /// The quiternion updated is in the local reference frame NOT from base
        /// </summary>
        /// <param name="EEvector"></param>
        /// <param name="DesiredRotationOut"></param>
        /// <returns></returns>
        bool setupController(Vector3 EEvector, ref Quaternion DesiredRotationOut)
        {
            Matrix _currentPose = currentPose;
            Vector3 axis = Vector3.Cross(Vector3.Normalize(_currentPose.Backward), Vector3.Normalize(EEvector));
            float angle = (float)Math.Asin((double)axis.Length());
            if (Math.Abs(angle) < MathHelper.ToRadians(1.0f))
            {
                return false;
            }
            else
            {
                DesiredRotationOut = Quaternion.CreateFromAxisAngle(Vector3.Normalize(axis), angle);
                return true;
            }
            return true;
            // The output is the transform from origin to final.
        }

        public void updateComandPosition()
        {
            lock (trajectoryLock)
            {
                if (_isConnected && _isCommanded && _CurrrentController.IsActive)
                {
                    Vector3 comandPos = _CurrrentController.getDisplacement(currentPose.Translation, MaxDisplacement);
                    Vector3 commandOri = _CurrrentController.getOrientation(currentRotation, (float)MaxOrientationDisplacement);
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
