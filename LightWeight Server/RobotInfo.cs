using Microsoft.Xna.Framework;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace LightWeight_Server
{
    class RobotInfo
    {
        public object trajectoryLock = new object();
        object gripperLock = new object();
        object maxSpeedLock = new object();
        object maxDisplacementLock = new object();

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
        // Thread safe lists for updating and storing of robot information.
        // public ConcurrentStack<StateObject> DataHistory;

        ConcurrentDictionary<String, StringBuilder> _text = new ConcurrentDictionary<string, StringBuilder>();

        Trajectory _CurrentTrajectory;

        bool _gripperIsOpen = true;
        double _maxSpeed = 10;
        double _maxDisplacement = 0.5;

        bool _isConnected = false;
        bool _isCommanded = false;
        bool _newCommandLoaded = false;


        StringBuilder _PrintMsg = new StringBuilder();
        String _errorMsg = String.Empty;

        public readonly double[] homePosition = new double[] { 540.5, -18.1, 833.3, 0.0, 0.0, 0.0 };

        public double ProcessDataTimer
        {
            get { return _processDataTimer; }
            set
            {
                if (_maxProcessDataTimer <= value)
                {
                    _maxProcessDataTimer = value;
                }
                _processDataTimer = value;
            }
        }

        // Thread safe getter which blocks till value is given
        public double CommandedPosition(int index)
        {
            return StaticFunctions.Getvalue(_CommandedPosition, StaticFunctions.getCardinalKey(index));
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

        public double[] currentDoublePose { get { return StaticFunctions.getCardinalDoubleArray(_ReadPosition); } }

        public Matrix currentPose
        {
            get
            {
                double[] doublePose = currentDoublePose;
                Matrix pose = Matrix.CreateFromQuaternion(StaticFunctions.MakeQuaternion(currentDoublePose));
                pose.Translation = new Vector3((float)doublePose[0], (float)doublePose[1], (float)doublePose[2]);
                return pose;
            }
        }

        public Quaternion currentRotation { get { return StaticFunctions.MakeQuaternion(currentDoublePose); } }

        public double MaxSpeed
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

        public RobotInfo()
        {
            _text.TryAdd("msg", new StringBuilder());
            _text.TryAdd("Error", new StringBuilder());

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

        public void Disconnected()
        {
            _isConnected = false;
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
                    Console.WriteLine(_PrintMsg.ToString());
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
                    if (_KukaCycleTime.ElapsedMilliseconds > 500)
                    {
                        _isConnected = false;
                    }
                    System.Threading.Thread.Sleep(100);
                }
                catch (Exception)
                {
                    Console.WriteLine("Error printing to Screen");
                }

            }
        }

        void updateMsg()
        {
            _text["msg"].Clear();
            _text["msg"].AppendLine("---------------------------------\n              Info:\n");
            _text["msg"].AppendLine("Current Position:     (" + String.Format("{0:0.00}", _ReadPosition["X"]) + " , " +
                                                                    String.Format("{0:0.00}", _ReadPosition["Y"]) + " , " +
                                                                    String.Format("{0:0.00}", _ReadPosition["Z"]) + ")");
            _text["msg"].AppendLine("Desired Position:     (" + String.Format("{0:0.00}", _DesiredPosition["X"]) + " , " +
                                                                    String.Format("{0:0.00}", _DesiredPosition["Y"]) + " , " +
                                                                    String.Format("{0:0.00}", _DesiredPosition["Z"]) + ")");
            _text["msg"].AppendLine("Command Position:     (" + String.Format("{0:0.00}", _CommandedPosition["X"]) + " , " +
                                                                    String.Format("{0:0.00}", _CommandedPosition["Y"]) + " , " +
                                                                    String.Format("{0:0.00}", _CommandedPosition["Z"]) + ")");

            _text["msg"].AppendLine("Max Speed: " + MaxSpeed.ToString() + "mm per cycle");
            if (gripperIsOpen)
            {
                _text["msg"].AppendLine("Gripper is OPEN.");
            }
            else
            {
                _text["msg"].AppendLine("Gripper is CLOSED");
            }
            if (_CurrentTrajectory.IsActive)
            {
                _text["msg"].AppendLine("Trajectory is Active");
            }
            else
            {
                _text["msg"].AppendLine("Trajectory is NOT Active");
            }
            if (_isCommanded)
            {
                _text["msg"].AppendLine("Robot is Commanded");
            }
            else
            {
                _text["msg"].AppendLine("Robot is NOT Commanded");
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
            _loopTime = 1.0 * _KukaCycleTime.ElapsedTicks / TimeSpan.TicksPerSecond;
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

        public void LoadTrajectory()
        {
            lock (trajectoryLock)
            {
                if (_newCommandLoaded)
                {
                    _CurrentTrajectory = new Trajectory(StaticFunctions.Getvalue(_DesiredPosition, "X"), StaticFunctions.Getvalue(_DesiredPosition, "Y"), StaticFunctions.Getvalue(_DesiredPosition, "Z"), this);
                    _CurrentTrajectory.Start();
                    _newCommandLoaded = false;
                    _isCommanded = true;
                }
            }
        }

        /// <summary>
        /// Resets timer and sets is active to true. Must occure in a trajectory lock.
        /// </summary>
        void startMovement()
        {
            // Start timer
            _KukaCycleTime.Restart();
            _CurrentTrajectory.IsActive = true;
        }

        public void newPosition(double x, double y, double z)
        {
            _DesiredPosition["X"] = x;
            _DesiredPosition["Y"] = y;
            _DesiredPosition["Z"] = z;
            lock (trajectoryLock)
            {
                _CurrentTrajectory = new Trajectory(x, y, z, this);
                startMovement();
            }
        }



        public void updateComandPosition()
        {
            if (_isConnected && _isCommanded && _CurrentTrajectory.IsActive)
            {
                // Update the command position all lights green
                double[] newComandPos = getKukaDisplacement();

                _CommandedPosition["X"] = newComandPos[0];
                _CommandedPosition["Y"] = newComandPos[1];
                _CommandedPosition["Z"] = newComandPos[2];
            }
            else
            {
                // End condition, or disconnected half way command position is zero
                flushCommands();
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
            double[] displacement = new double[6];
            double sumDisplacement = 0;
            // For each axis of movement get displacement of current position and trajectory position
            for (int i = 0; i < 3; i++)
            {
                displacement[i] = _CurrentTrajectory.RefPos(i) - StaticFunctions.Getvalue(_ReadPosition, StaticFunctions.getCardinalKey(i));
                if (Math.Abs(displacement[i]) > MaxDisplacement)
                {
                    updateError("Error, " + StaticFunctions.getCardinalKey(i) + " Axis sent a huge distance, at " + displacement[i].ToString() + "mm");
                    displacement[i] = 0.5 * Math.Sign(displacement[i]);
                }
                sumDisplacement += displacement[i];
            }
            // Are we within 0.05mm of goal stop motion
            // NOTE TODO: angular coordinate end condition
            if (Math.Abs(sumDisplacement) < 0.05)
            {
                _isCommanded = false;
                _CurrentTrajectory.Stop();
            }
            return displacement;
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
