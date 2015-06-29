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

        ConcurrentDictionary<String, double> _ReadPosition = new ConcurrentDictionary<string, double>();
        ConcurrentDictionary<String, double> _LastPosition = new ConcurrentDictionary<string, double>();
        ConcurrentDictionary<String, double> _Velocity = new ConcurrentDictionary<string, double>();
        ConcurrentDictionary<String, double> _LastVelocity = new ConcurrentDictionary<string, double>();
        ConcurrentDictionary<String, double> _acceleration = new ConcurrentDictionary<string, double>();
        ConcurrentDictionary<String, double> _Torque = new ConcurrentDictionary<string, double>();
        ConcurrentDictionary<String, double> _DesiredPosition = new ConcurrentDictionary<string, double>();
        ConcurrentDictionary<String, double> _CommandedPosition = new ConcurrentDictionary<string, double>();
        Quaternion _DesiredRotation = Quaternion.Identity;
        Vector3 desiredZaxis = new Vector3();
        // Thread safe lists for updating and storing of robot information.
        // public ConcurrentStack<StateObject> DataHistory;

        ConcurrentDictionary<String, StringBuilder> _text = new ConcurrentDictionary<string, StringBuilder>();

        Trajectory _CurrentTrajectory;
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


        StringBuilder _PrintMsg = new StringBuilder();
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
                return StaticFunctions.Getvalue(_CommandedPosition, StaticFunctions.getCardinalKey(index));
            }
        }
        public double CurrentPosition(int index)
        {
            return StaticFunctions.Getvalue(_ReadPosition, StaticFunctions.getCardinalKey(index));
        }
        public double CurrentVelocity(int index)
        {
            return StaticFunctions.Getvalue(_Velocity, StaticFunctions.getCardinalKey(index));
        }
        public double CurrentAcceleration(int index)
        {
            return StaticFunctions.Getvalue(_acceleration, StaticFunctions.getCardinalKey(index));
        }

        // inb degrees
        public double[] currentDoublePose { get { return StaticFunctions.getCardinalDoubleArray(_ReadPosition); } }

        Matrix currentPose
        {
            get
            {
                return StaticFunctions.MakeMatrixFromKuka(currentDoublePose);
            }
        }

        public Quaternion currentRotation { get { return StaticFunctions.MakeQuaternionFromKuka(currentDoublePose); } }

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
            setupCardinalDictionaries(_ReadPosition, homePosition);
            setupCardinalDictionaries(_DesiredPosition, homePosition);
            setupCardinalDictionaries(_LastPosition);
            setupCardinalDictionaries(_Velocity);
            setupCardinalDictionaries(_LastVelocity);
            setupCardinalDictionaries(_acceleration);
            setupCardinalDictionaries(_CommandedPosition);

            setupAxisDictionaries(_Torque);

            _text["Error"].Append("---------------------------------\n             Errors:\n");
        }

        public void Connect()
        {
            _isConnected = true;
        }

        public void Disconnect()
        {
            lock (trajectoryLock)
            {
                try
                {
                    _CurrentTrajectory.Stop();
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
        public void writeMsgs()
        {
            while (true)
            {
                try
                {
                    Console.Clear();
                    bool hasMsg = false;
                    while (!hasMsg)
                    {
                        hasMsg = _text.TryGetValue("Error", out _PrintMsg);
                    }

                    StreamWriter file = new StreamWriter("ErrorMsg" + _errorMsgs.ToString() + ".txt");

                    file.WriteLine(_PrintMsg);
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
                            hasMsg = _text.TryGetValue("msg", out _PrintMsg);
                        }
                        Console.WriteLine(_PrintMsg.ToString());
                    }
                    else
                    {
                        Console.WriteLine("---------------------------------\n   Not Connected to Kuka Robot");
                        Console.WriteLine("{0} : {1} : {2} : {3} : {4} : {5}", _ReadPosition["X"], _ReadPosition["Y"], _ReadPosition["Z"], _ReadPosition["A"], _ReadPosition["B"], _ReadPosition["C"]);
                    }
                    if (_KukaCycleTime.ElapsedMilliseconds > 10000)
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
            _text["msg"].Clear();
            _text["msg"].AppendLine("---------------------------------\n              Info:\n");
            _text["msg"].AppendLine("Current Position:     (" + String.Format("{0:0.00}", _ReadPosition["X"]) + " , " +
                                                                    String.Format("{0:0.00}", _ReadPosition["Y"]) + " , " +
                                                                    String.Format("{0:0.00}", _ReadPosition["Z"]) + " , " +
                                                                    String.Format("{0:0.00}", _ReadPosition["A"]) + " , " +
                                                                    String.Format("{0:0.00}", _ReadPosition["B"]) + " , " +
                                                                    String.Format("{0:0.00}", _ReadPosition["C"]) + ")");
            _text["msg"].AppendLine("Current Velocity:     (" + String.Format("{0:0.00}", _Velocity["X"]) + " , " +
                                                                    String.Format("{0:0.00}", _Velocity["Y"]) + " , " +
                                                                    String.Format("{0:0.00}", _Velocity["Z"]) + " , " +
                                                                    String.Format("{0:0.00}", _Velocity["A"]) + " , " +
                                                                    String.Format("{0:0.00}", _Velocity["B"]) + " , " +
                                                                    String.Format("{0:0.00}", _Velocity["C"]) + ")");
            _text["msg"].AppendLine("Desired Position:     (" + String.Format("{0:0.00}", _DesiredPosition["X"]) + " , " +
                                                                    String.Format("{0:0.00}", _DesiredPosition["Y"]) + " , " +
                                                                    String.Format("{0:0.00}", _DesiredPosition["Z"]) + ")");
            _text["msg"].AppendLine("Command Position:     (" + String.Format("{0:0.0000}", _CommandedPosition["X"]) + " , " +
                                                                    String.Format("{0:0.0000}", _CommandedPosition["Y"]) + " , " +
                                                                    String.Format("{0:0.0000}", _CommandedPosition["Z"]) + "," +
                                                                     String.Format("{0:0.0000}", _CommandedPosition["A"]) + " , " +
                                                                    String.Format("{0:0.0000}", _CommandedPosition["B"]) + " , " +
                                                                    String.Format("{0:0.0000}", _CommandedPosition["C"]) + ")");
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
                _text["msg"].AppendLine("Trajectory is Active");
            }
            else
            {
                _text["msg"].AppendLine("Robot is NOT Commanded");
                _text["msg"].AppendLine("Trajectory is NOT Active");
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
        public void updateRobotPosition(double x, double y, double z, double a, double b, double c)
        {
            //updateError(x.ToString() + " : " + y.ToString() + " : " + z.ToString() + " : " + a.ToString() + " : " + b.ToString() + " : " + c.ToString());
            _KukaCycleTime.Stop();
            if (_KukaCycleTime.ElapsedTicks == 0)
            {
                _loopTime = 1.0 / 73;
            }
            else
            {
                if (Math.Abs(_loopTime - 1.0 * _KukaCycleTime.ElapsedTicks / TimeSpan.TicksPerSecond) > 5.0 / 1000)
                {
                    _loopTime = 1.0 / 73;
                }
                else
                {
                    _loopTime = 1.0 * _KukaCycleTime.ElapsedTicks / TimeSpan.TicksPerSecond;
                }

            }
            _KukaCycleTime.Restart();

            _LastPosition["X"] = _ReadPosition["X"];
            _LastPosition["Y"] = _ReadPosition["Y"];
            _LastPosition["Z"] = _ReadPosition["Z"];
            _LastPosition["A"] = _ReadPosition["A"];
            _LastPosition["B"] = _ReadPosition["B"];
            _LastPosition["C"] = _ReadPosition["C"];

            _ReadPosition["X"] = x;
            _ReadPosition["Y"] = y;
            _ReadPosition["Z"] = z;
            _ReadPosition["A"] = a;
            _ReadPosition["B"] = b;
            _ReadPosition["C"] = c;

            _LastVelocity["X"] = _Velocity["X"];
            _LastVelocity["Y"] = _Velocity["Y"];
            _LastVelocity["Z"] = _Velocity["Z"];
            _LastVelocity["A"] = _Velocity["A"];
            _LastVelocity["B"] = _Velocity["B"];
            _LastVelocity["C"] = _Velocity["C"];

            _Velocity["X"] = 1.0 * (_ReadPosition["X"] - _LastPosition["X"]) / _loopTime;
            _Velocity["Y"] = 1.0 * (_ReadPosition["Y"] - _LastPosition["Y"]) / _loopTime;
            _Velocity["Z"] = 1.0 * (_ReadPosition["Z"] - _LastPosition["Z"]) / _loopTime;
            _Velocity["A"] = 1.0 * (_ReadPosition["A"] - _LastPosition["A"]) / _loopTime;
            _Velocity["B"] = 1.0 * (_ReadPosition["B"] - _LastPosition["B"]) / _loopTime;
            _Velocity["C"] = 1.0 * (_ReadPosition["C"] - _LastPosition["C"]) / _loopTime;

            _acceleration["X"] = 1.0 * (_Velocity["X"] - _LastVelocity["X"]) / _loopTime;
            _acceleration["Y"] = 1.0 * (_Velocity["Y"] - _LastVelocity["Y"]) / _loopTime;
            _acceleration["Z"] = 1.0 * (_Velocity["Z"] - _LastVelocity["Z"]) / _loopTime;
            _acceleration["A"] = 1.0 * (_Velocity["A"] - _LastVelocity["A"]) / _loopTime;
            _acceleration["B"] = 1.0 * (_Velocity["B"] - _LastVelocity["B"]) / _loopTime;
            _acceleration["C"] = 1.0 * (_Velocity["C"] - _LastVelocity["C"]) / _loopTime;

        }

        public void updateRobotTorque(double a1, double a2, double a3, double a4, double a5, double a6)
        {
            _Torque["A1"] = a1;
            _Torque["A2"] = a2;
            _Torque["A3"] = a3;
            _Torque["A4"] = a4;
            _Torque["A5"] = a5;
            _Torque["A6"] = a6;
        }

        public void LoadedCommand()
        {
            lock (trajectoryLock)
            {
                _newCommandLoaded = true;
            }
        }

        /// <summary>
        /// Loads the _desiredPosition data in Base coordinates and the _desiredRotation in local SartPose coordinates
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
                            Vector3 axis = Vector3.Zero;
                            float angle = 0;
                            StaticFunctions.getAxisAngle(_DesiredRotation, ref axis, ref angle);
                            long orientationDuration = (long)(TimeSpan.TicksPerSecond * (angle / (MaxOrientationDisplacement*10)));
                            updateError("Orientation Time in seconds: " + (1.0f * orientationDuration / TimeSpan.TicksPerSecond).ToString());
                            updateError("vectot out 2: " + Vector3.Transform(currentPose.Backward, _DesiredRotation));
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

        public void newPosition(double x, double y, double z)
        {
            lock (trajectoryLock)
            {
                _DesiredPosition["X"] = x;
                _DesiredPosition["Y"] = y;
                _DesiredPosition["Z"] = z;
                _newPositionLoaded = true;
            }
            /*
            lock (trajectoryLock)
            {
                _CurrentTrajectory = new Trajectory(x, y, z, this);
                startMovement();
            }
             */
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
        /// Updates the desired rotation with a quaternion representing a change from origin to the final orientation, EEVector
        /// </summary>
        /// <param name="EEvector"></param>
        /// <param name="DesiredRotationOut"></param>
        /// <returns></returns>
        bool setupController(Vector3 EEvector, ref Quaternion DesiredRotationOut)
        {
            updateError("Desired vector: " + EEvector.ToString());
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
                updateError("vectot out: " + Vector3.Transform(_currentPose.Backward, DesiredRotationOut));
                updateError("Angle of rotation: " + angle.ToString());
                updateError("Axis of rotation: " + Vector3.Normalize(axis).ToString());
                updateError("Matrix of rotation: " + Matrix.CreateFromQuaternion(DesiredRotationOut).ToString());
                return true;
            }
            Vector3 xAxis = Vector3.Zero;
            Vector3 yAxis = Vector3.Zero;
            Vector3 zAxis = EEvector;
            updateError("current Pose: " + _currentPose.ToString());
            updateError("Forwards: " + _currentPose.Forward.ToString() + "Down: " + _currentPose.Down.ToString() + "Left: " + _currentPose.Left.ToString());
            updateError("Y comp: " + Vector3.Dot(EEvector, Vector3.Normalize(_currentPose.Up)).ToString());
            updateError("X comp: " + Vector3.Dot(EEvector, Vector3.Normalize(_currentPose.Right)).ToString());

            if (Math.Abs(Vector3.Dot(EEvector, Vector3.Normalize(_currentPose.Right))) > Math.Abs(Vector3.Dot(EEvector, Vector3.Normalize(_currentPose.Up))))
            {
                yAxis = Vector3.Normalize(Vector3.Cross(EEvector, _currentPose.Backward));
                xAxis = Vector3.Normalize(Vector3.Cross(yAxis, zAxis));
            }
            else
            {
                xAxis = Vector3.Normalize(Vector3.Cross(EEvector, _currentPose.Backward));
                yAxis = Vector3.Normalize(Vector3.Cross(zAxis, xAxis));
            }

            updateError("Setup xAxis: " + xAxis.ToString() + "\nSetup yAxis: " + yAxis.ToString() + "\nSetup zAxis: " + zAxis.ToString());
            DesiredRotationOut = Quaternion.CreateFromRotationMatrix(new Matrix(xAxis.X, xAxis.Y, xAxis.Z, 0,
                                                                                       yAxis.X, yAxis.Y, yAxis.Z, 0,
                                                                                       zAxis.X, zAxis.Y, zAxis.Z, 0,
                                                                                       0, 0, 0, 1));
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
                    if (comandPos.Equals(Vector3.Zero))
                    {
                        
                    }

                    _CommandedPosition["X"] = comandPos.X;
                    _CommandedPosition["Y"] = comandPos.Y;
                    _CommandedPosition["Z"] = comandPos.Z;
                    _CommandedPosition["A"] = commandOri.X;
                    _CommandedPosition["B"] = commandOri.Y;
                    _CommandedPosition["C"] = commandOri.Z;
                    //double[] newComandPos = getKukaDisplacement();

                    //_CommandedPosition["X"] = newComandPos[0];
                    // _CommandedPosition["Y"] = newComandPos[1];
                    //_CommandedPosition["Z"] = newComandPos[2];
                    // _CommandedPosition["A"] = newComandPos[3];
                    // _CommandedPosition["B"] = newComandPos[4];
                    // _CommandedPosition["C"] = newComandPos[5];
                }
                else
                {
                    // End condition, or disconnected half way command position is zero
                    flushCommands();
                }

            }

            /*
            try
            {
                goTo.updateSpeed();
                //double[] commandArray = getKukaDisplacement();
                for (int i = 0; i < 3; i++)
                {
                    CommandedPosition[goTo.axisKey[i]] = goTo.normalVector[i];
                }

            }
            catch (Exception)
            {

            }
             * */
        }

        double[] getKukaDisplacement()
        {
            Matrix tempRefPose = _CurrentTrajectory.getReferencePose();
            Matrix _currentPose = currentPose;
            Matrix tempDisplacementPose = Matrix.Invert(_currentPose) * tempRefPose;
            double[] KukaUpdate = new double[6];
            StaticFunctions.getKukaAngles(tempDisplacementPose, ref KukaUpdate);
            _text["Controller"].AppendLine(_CurrentTrajectory.ElapsedMilliseconds + "," + tempRefPose.Translation.X + "," + tempRefPose.Translation.Y + "," + tempRefPose.Translation.Z + "," +
                _currentPose.Translation.X + "," + _currentPose.Translation.Y + "," + _currentPose.Translation.Z + "," +
                tempDisplacementPose.Translation.X + "," + tempDisplacementPose.Translation.Y + "," + tempDisplacementPose.Translation.Z + "," +
                _acceleration["X"].ToString() + "," + _acceleration["Y"].ToString() + "," + _acceleration["Z"].ToString() + "," +
                _Velocity["X"].ToString() + "," + _Velocity["Y"].ToString() + "," + _Velocity["Z"].ToString());
            // For each axis of movement get displacement of current position and trajectory position
            for (int i = 0; i < 6; i++)
            {
                if (Math.Abs(KukaUpdate[i]) > MaxDisplacement)
                {
                    updateError("At " + _CurrentTrajectory.ElapsedMilliseconds.ToString() + "ms, Error, " + StaticFunctions.getCardinalKey(i) + " Axis sent a huge distance, at " + KukaUpdate[i].ToString() + "mm");
                    KukaUpdate[i] = MaxDisplacement * Math.Sign(KukaUpdate[i]);
                }
            }


            // Are we within 0.05mm of goal stop motion
            // NOTE TODO: angular coordinate end condition
            if (_isCommanded && _CurrentTrajectory.hasFinished(_currentPose))
            {
                _isCommanded = false;
                _CurrentTrajectory.Stop();
            }
            return KukaUpdate;
        }

        public void flushCommands()
        {
            for (int i = 0; i < 6; i++)
            {
                _CommandedPosition[StaticFunctions.getCardinalKey(i)] = 0.0;
            }
        }

        #endregion

        #region Dictionary Setup

        void setupCardinalDictionaries(ConcurrentDictionary<string, double> dic)
        {
            for (int i = 0; i < 6; i++)
            {
                if (!dic.TryAdd(StaticFunctions.getCardinalKey(i), 0))
                {
                    dic[StaticFunctions.getCardinalKey(i)] = 0;
                }
            }
        }
        void setupCardinalDictionaries(ConcurrentDictionary<string, double> dic, double[] Values)
        {
            for (int i = 0; i < Values.Length; i++)
            {
                if (!dic.TryAdd(StaticFunctions.getCardinalKey(i), Values[i]))
                {
                    dic[StaticFunctions.getCardinalKey(i)] = Values[i];
                }
            }
        }

        void setupAxisDictionaries(ConcurrentDictionary<string, double> dic)
        {
            for (int i = 0; i < 6; i++)
            {
                if (!dic.TryAdd(StaticFunctions.getAxisKey(i), 0))
                {
                    dic[StaticFunctions.getAxisKey(i)] = 0;
                }
            }
        }
        #endregion


    }
}
