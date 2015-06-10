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
        public readonly double[] startPosition = new double[] { 540.5, -18.1, 833.3, 0.0, 0.0, 0.0 };
        public object trajectoryLock = new object();

        Stopwatch KukaUpdateTime = new Stopwatch();

        // Time of loop in SECONDS
        double loopTime = 0;
        double processDataTimer = 0;
        double maxProcessDataTimer = 0;

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

        public bool GripperIsOpen = true;

        bool _isConnected = false;

        double _maxSpeed = 0.5;

        StringBuilder _PrintMsg = new StringBuilder();
        String errorMsg = String.Empty;

        public double ProcessDataTimer
        {
            get { return _processDataTimer; }
            set
            {
                if (maxProcessDataTimer <= value)
                {
                    maxProcessDataTimer = value;
                }
                processDataTimer = value;
            }
        }

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


        public double CurrentMotionMaxSpeed
        {
            get
            {
                lock (trajectoryLock)
                {
                    return _CurrentTrajectory.maxSpeed;
                }
            }
            set
            {
                lock (trajectoryLock)
                {
                    _CurrentTrajectory.maxSpeed = value;
                }
            }
        }
        public RobotInfo()
        {
            _text.TryAdd("msg", new StringBuilder());
            _text.TryAdd("Error", new StringBuilder());

            setupCardinalDictionaries(_ReadPosition, startPosition);
            setupCardinalDictionaries(_DesiredPosition, startPosition);
            setupCardinalDictionaries(_LastPosition);
            setupCardinalDictionaries(_Velocity);
            setupCardinalDictionaries(_LastVelocity);
            setupCardinalDictionaries(_acceleration);
            setupCardinalDictionaries(_CommandedPosition);

            setupAxisDictionaries(_Torque);

            _text["Error"].Append("---------------------------------\n             Errors:\n");


            newPosition(540.5, -18.1, 833.3);

        }

        public void Connect()
        {
            if (!_isConnected)
            {
                newPosition(540.5, -18.1, 833.3);
                _isConnected = true;
            }
        }

        public void reset()
        {
            GripperIsOpen = true;

            _isConnected = false;

            setupCardinalDictionaries(_ReadPosition, startPosition);
            setupCardinalDictionaries(_DesiredPosition, startPosition);

            _maxSpeed = 0.5;
            loopTime = 0;
            processDataTimer = 0;
            maxProcessDataTimer = 0;

            newPosition(540.5, -18.1, 833.3);

        }


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
                    updateMsg();
                    Console.Clear();
                    bool hasgotMsg = false;
                    while (!hasgotMsg)
                    {
                        hasgotMsg = _text.TryGetValue("Error", out _PrintMsg);
                    }
                    Console.WriteLine(_PrintMsg.ToString());
                    if (_isConnected)
                    {
                        _text.TryGetValue("msg", out _PrintMsg);
                        Console.WriteLine(_PrintMsg.ToString());
                    }
                    else
                    {
                        Console.WriteLine("---------------------------------\n   Not Connected to Kuka Robot");
                        Console.WriteLine("{0} : {1} : {2} : {3} : {4} : {5}", _ReadPosition["X"], _ReadPosition["Y"], _ReadPosition["Z"], _ReadPosition["A"], _ReadPosition["B"], _ReadPosition["C"]);
                    }
                    if (KukaUpdateTime.ElapsedMilliseconds > 1000)
                    {
                        _isConnected = false;
                        reset();
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

            _text["msg"].AppendLine("Max Speed: " + CurrentMotionMaxSpeed.ToString() + "mm per cycle");
            if (GripperIsOpen)
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
            _text["msg"].AppendLine("Process data time: " + processDataTimer.ToString() + "ms.");
            _text["msg"].AppendLine("Kuka cycle time: " + loopTime.ToString() + "ms.");


        }
        #endregion

        #region Movement
        public void updateRobotPosition(double x, double y, double z, double a, double b, double c)
        {
            //updateError(x.ToString() + " : " + y.ToString() + " : " + z.ToString() + " : " + a.ToString() + " : " + b.ToString() + " : " + c.ToString());
            KukaUpdateTime.Stop();
            loopTime = 1.0 * KukaUpdateTime.ElapsedTicks / TimeSpan.TicksPerSecond;
            KukaUpdateTime.Restart();

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

            _Velocity["X"] = 1.0 * (_ReadPosition["X"] - _LastPosition["X"]) / loopTime;
            _Velocity["Y"] = 1.0 * (_ReadPosition["Y"] - _LastPosition["Y"]) / loopTime;
            _Velocity["Z"] = 1.0 * (_ReadPosition["Z"] - _LastPosition["Z"]) / loopTime;
            _Velocity["A"] = 1.0 * (_ReadPosition["A"] - _LastPosition["A"]) / loopTime;
            _Velocity["B"] = 1.0 * (_ReadPosition["B"] - _LastPosition["B"]) / loopTime;
            _Velocity["C"] = 1.0 * (_ReadPosition["C"] - _LastPosition["C"]) / loopTime;

            _acceleration["X"] = 1.0 * (_Velocity["X"] - _LastVelocity["X"]) / loopTime;
            _acceleration["Y"] = 1.0 * (_Velocity["Y"] - _LastVelocity["Y"]) / loopTime;
            _acceleration["Z"] = 1.0 * (_Velocity["Z"] - _LastVelocity["Z"]) / loopTime;
            _acceleration["A"] = 1.0 * (_Velocity["A"] - _LastVelocity["A"]) / loopTime;
            _acceleration["B"] = 1.0 * (_Velocity["B"] - _LastVelocity["B"]) / loopTime;
            _acceleration["C"] = 1.0 * (_Velocity["C"] - _LastVelocity["C"]) / loopTime;

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

        /// <summary>
        /// Resets timer and sets is active to true. Must occure in a trajectory lock.
        /// </summary>
        void startMovement()
        {
            // Start timer
            KukaUpdateTime.Restart();
            _CurrentTrajectory.IsActive = true;
        }

        public void newPosition(double x, double y, double z)
        {
            _DesiredPosition["X"] = x;
            _DesiredPosition["Y"] = y;
            _DesiredPosition["Z"] = z;
            lock (trajectoryLock)
            {
                _CurrentTrajectory = new Trajectory( x, y, z, this);
                startMovement();
            }
        }



        double[] GetCommandPosition()
        {
            lock (trajectoryLock)
            {
                if (_CurrentTrajectory.IsActive )
                {
                    return getKukaDisplacement();
                }
                else
                {
                    return new double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
                }

            }

        }

        public void updateComandPosition()
        {
            double[] newComandPos = GetCommandPosition();

            _CommandedPosition["X"] = newComandPos[0];
            _CommandedPosition["Y"] = newComandPos[1];
            _CommandedPosition["Z"] = newComandPos[2];

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
                displacement[i] = _CurrentTrajectory.RefPos(i) - _ReadPosition[StaticFunctions.getCardinalKey(i)];
                if (Math.Abs(displacement[i]) > 0.5)
                {
                    updateError("Error, " + StaticFunctions.getCardinalKey(i) + " Axis sent a huge distance, at " + displacement[i].ToString() + "mm");
                    displacement[i] = 0.5 * Math.Sign(displacement[i]);
                }
                sumDisplacement += displacement[i];
            }
            // Are we within 0.05mm of goal?
            if (Math.Abs(sumDisplacement) < 0.05)
            {
                _CurrentTrajectory.IsActive = false;
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


        public double _processDataTimer { get; set; }
    }
}
