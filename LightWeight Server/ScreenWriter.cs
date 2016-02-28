using Microsoft.Xna.Framework;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Net;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using CustomExtensions;

namespace LightWeight_Server
{
    
    class ScreenWriter
    {
        object dataWriteLock = new object();
        object ErrorWriteLock = new object();
        object telemetryLock = new object();
        object ScreenLock = new object();
        object debuggerLock = new object();
        object CSVdebuggerLock = new object();
        object controlDAtaLock = new object();

        readonly string DirectoryName = "DataInfo" + Guid.NewGuid().ToString("");

        bool _isConnected;
        public bool closing = false;

        public int _nConnectedRobots, _nConnectedExternalServers;

        ConcurrentDictionary<string, double[]> _angles=new ConcurrentDictionary<string,double[]>();
        ConcurrentDictionary<string, Pose> _Position = new ConcurrentDictionary<string, Pose>(), 
                                            _Velocity = new ConcurrentDictionary<string, Pose>(), 
                                            _Acceleration = new ConcurrentDictionary<string, Pose>(),
                                            _DesiredPosition = new ConcurrentDictionary<string, Pose>(),
                                            _ReferencePosition = new ConcurrentDictionary<string, Pose>(), 
                                            _ReferenceVelcoity = new ConcurrentDictionary<string, Pose>();
        ConcurrentDictionary<string, Vector3> _EndEffector = new ConcurrentDictionary<string,Vector3>(), 
                                            _currentZaxis= new ConcurrentDictionary<string,Vector3>(), 
                                            _desiredZaxis= new ConcurrentDictionary<string,Vector3>();
        ConcurrentDictionary<string, StringBuilder> _ControllerData = new ConcurrentDictionary<string,StringBuilder>();
        ConcurrentDictionary<string, string> _RobotName = new ConcurrentDictionary<string, string>();
        RobotInfo[] _ConnectedRobots;

        StringBuilder _ErrorWriter = new StringBuilder(), _Debugger = new StringBuilder(), _DisplayMsg = new StringBuilder(), _ControlData = new StringBuilder(), _CSVdebugData = new StringBuilder();

        //ConcurrentDictionary<string, Stopwatch> _Timers = new ConcurrentDictionary<string,Stopwatch>();

        TimeSpan A3 = TimeSpan.Zero, A4 = TimeSpan.Zero, A5 = TimeSpan.Zero, A6 = TimeSpan.Zero;

        ConcurrentQueue<IPEndPoint> ClientList = new ConcurrentQueue<IPEndPoint>();
        
        
        Stopwatch _ErrorTimer = new Stopwatch();
        Stopwatch _FileWriter = new Stopwatch();

        long lastLog;

        public bool IsConnected
        {
            set
            {
                lock (ScreenLock)
                {
                    _isConnected = value;
                }
            }
        }

        public void disconnect()
        {
            IsConnected = false;
            IPEndPoint garbage = null;
            while (ClientList.TryDequeue(out garbage)) ;
        }

        public ScreenWriter()
        {
            Console.WriteLine("Welcome to Kuka RSI server.");
            /*
            Console.WriteLine("Type the number of connected robots...");
            // Selects the first IP in the list and writes it to screen
            while (true)
            {
                string numbers = Console.ReadLine();
                if (int.TryParse(numbers, out _nConnectedRobots))
                {
                    Console.WriteLine("number of robots: {0}", _nConnectedRobots);
                    break;
                }
                else
                {
                    Console.WriteLine("Type numbers only followed by enter.");
                }
            }
             */
            _nConnectedRobots = 1;
            _ConnectedRobots = new RobotInfo[_nConnectedRobots];
            // Initialises the screen variables.
            lastLog = DateTime.Now.Ticks;
        }

        public void ConnectRobot(RobotInfo newRobot, int number)
        {
            _ErrorTimer.Start();
            _ConnectedRobots[number] = newRobot;
            _RobotName.TryAdd(newRobot._RobotID.ToString(), "Robot " + number.ToString());
            SetupTelemetryInfo(newRobot);
        }

        public void ConnectExternal(IPEndPoint newClient)
        {
            ClientList.Enqueue(newClient);
        }


        public void updateError(string newError, Exception Error)
        {
            lock (ErrorWriteLock)
            {
                _ErrorWriter.AppendLine(string.Format("Error Log: {0}", System.DateTime.Now.ToShortTimeString()));
                _ErrorWriter.AppendLine(string.Format("Local IPOC: {0:0.0}", _ErrorTimer.Elapsed.TotalMilliseconds));
                _ErrorWriter.AppendLine(newError);
                _ErrorWriter.AppendLine(Error.Message);
                _ErrorWriter.AppendLine(Error.StackTrace);
            }
        }
        
        // Dedicated loop thread
        public void UpdateScreen()
        {
            _FileWriter.Start();
            while (true)
            {
                if (_FileWriter.Elapsed.TotalSeconds > 2)
                {
                    _FileWriter.Restart();
                    WriteToFile();
                }

                try
                {
                    lock (ScreenLock)
                    {
                        if (closing)
                        {
                            Console.Clear();
                            Console.WriteLine("Saving progress and errors...");
                        }
                        else
                        {
                            if (_isConnected)
                            {
                                if (!ClientList.IsEmpty)
                                {
                                    IPEndPoint[] clintIPList = ClientList.ToArray();
                                    int n = 1;
                                    foreach (IPEndPoint IP in clintIPList)
                                    {
                                        try
                                        {
                                            _DisplayMsg.AppendLine("Client " + n.ToString() + " info:");
                                            _DisplayMsg.AppendLine(string.Format("Connected Port: {0}\nConnected IP: {1}\n", IP.Port.ToString(), IP.Address.ToString()));
                                        }
                                        catch (Exception e)
                                        {
                                            updateError("Error writing to screen", e);
                                        }
                                    }
                                }
                                foreach (var robot in _ConnectedRobots)
                                {
                                    try
                                    {
                                        _DisplayMsg.AppendLine(_RobotName[robot._RobotID.ToString()] + " info:");
                                        _DisplayMsg.AppendLine(string.Format("Connected Port: {0}\nConnected IP: {1}\n", robot.EndPoint.Port.ToString(), robot.EndPoint.Address.ToString()));
                                        updateTelemetryInfo(robot);
                                        updateMsg(robot);
                                    }
                                    catch (Exception e)
                                    {
                                        updateError("Error writing to screen", e);
                                    }
                                }
                                Console.Clear();
                                Console.WriteLine(_DisplayMsg);
                                _DisplayMsg.Clear();
                            }
                            else
                            {
                                Console.Clear();
                                Console.WriteLine("---------------------------------\n   Not Connected to Kuka Robot.....Start RSI");
                            }
                        }
                    }
                    System.Threading.Thread.Sleep(100);
                }
                catch (Exception e)
                {
                    updateError("Error printing to Screen", e);
                }
            }
        }
        
        public void WriteToFile()
        {
            if (!Directory.Exists(DirectoryName))
            {
                Directory.CreateDirectory(DirectoryName);
            }
            using (StreamWriter file = new StreamWriter(DirectoryName + "/ErrorMsg" + ".txt", true))
            {
                lock (ErrorWriteLock)
                {
                    if (_ErrorWriter.Length != 0)
                    {
                        file.WriteLine(_ErrorWriter);
                        _ErrorWriter.Clear();
                    }
                }
            }
            using (StreamWriter file = new StreamWriter(DirectoryName + "/DebuggerLog" + ".txt", true))
            {
                lock (debuggerLock)
                {
                    if (_Debugger.Length != 0)
                    {

                        file.WriteLine(_Debugger);
                        _Debugger.Clear();
                    }
                }
            }
            using (StreamWriter file = new StreamWriter(DirectoryName + "/Control" + ".csv", true))
            {
                lock (controlDAtaLock)
                {
                    if (_ControlData.Length != 0)
                    {

                        file.WriteLine(_ControlData);
                        _ControlData.Clear();
                    }
                }
            }
            using (StreamWriter file = new StreamWriter(DirectoryName + "/CSVDebug" + ".csv", true))
            {
                lock (CSVdebuggerLock)
                {
                    if (_CSVdebugData.Length != 0)
                    {
                        file.WriteLine(_CSVdebugData);
                        _CSVdebugData.Clear();
                    }
                }
            }
        }


        void SetupTelemetryInfo(RobotInfo robot)
        {
            _angles.TryAdd(robot._RobotID.ToString(),robot.currentAxisAngle);
            _Position.TryAdd(robot._RobotID.ToString(),robot.currentPose);
            _Velocity.TryAdd(robot._RobotID.ToString(),robot.currentVelocity);
            _Acceleration.TryAdd(robot._RobotID.ToString(),robot.currentAcceleration);
            _DesiredPosition.TryAdd(robot._RobotID.ToString(),robot.currentDesiredPositon);
            _ReferencePosition.TryAdd(robot._RobotID.ToString(),robot.currentReferencePosition);
            _ReferenceVelcoity.TryAdd(robot._RobotID.ToString(), robot.currentReferenceVelocity);
            _EndEffector.TryAdd(robot._RobotID.ToString(), robot.EndEffector);
            _currentZaxis.TryAdd(robot._RobotID.ToString(), robot.currentPose.zAxis);
            _desiredZaxis.TryAdd(robot._RobotID.ToString(), robot.currentDesiredPositon.zAxis);
            _ControllerData.TryAdd(robot._RobotID.ToString(), new StringBuilder());

        }

        public void addControllerData(string msg)
        {
            lock (controlDAtaLock)
            {
                _ControlData.AppendLine(msg);
            }
        }

        void updateTelemetryInfo(RobotInfo robot)
        {
            _angles[robot._RobotID.ToString()] = robot.currentAxisAngle.getDegree();
            _Position[robot._RobotID.ToString()] = robot.currentPose;
            _Velocity[robot._RobotID.ToString()] = robot.currentVelocity;
            _Acceleration[robot._RobotID.ToString()] = robot.currentAcceleration;
            _DesiredPosition[robot._RobotID.ToString()] = robot.currentDesiredPositon;
            _ReferencePosition[robot._RobotID.ToString()] = robot.currentReferencePosition;
            _ReferenceVelcoity[robot._RobotID.ToString()] = robot.currentReferenceVelocity;
            _EndEffector[robot._RobotID.ToString()] = robot.EndEffector;
        }

        void updateMsg(RobotInfo robot)
        {
            _DisplayMsg.AppendLine("---------------------------------\n");
            plotDoubleQue(robot.serverTimer, "Server loop Time", _DisplayMsg);
            plotDoubleQue(robot.MaxserverTimer, "Max Server Loop Time", _DisplayMsg);
            plotDoubleQue(robot._processDataTimer, "Process Data Time", _DisplayMsg);
            plotDoubleQue(robot._maxProcessDataTimer, "Max Process Time", _DisplayMsg);
            plotVector(_EndEffector[robot._RobotID.ToString()], "End Effector", _DisplayMsg);
            plotPose(_DesiredPosition[robot._RobotID.ToString()], "Desired Position", _DisplayMsg);
            plotPose(_Position[robot._RobotID.ToString()], "Position", _DisplayMsg);
            plotPose(_ReferencePosition[robot._RobotID.ToString()], "Reference Position", _DisplayMsg);
            plotPose(_Velocity[robot._RobotID.ToString()], "Velocity", _DisplayMsg);
            plotPose(_ReferenceVelcoity[robot._RobotID.ToString()], "Reference Velocity", _DisplayMsg);
            plotDoubles(_angles[robot._RobotID.ToString()], "Axis Angles", _DisplayMsg);
            plotVector(_Position[robot._RobotID.ToString()].zAxis, "Current Z-Axis", _DisplayMsg);
            plotVector(_DesiredPosition[robot._RobotID.ToString()].zAxis, "Desired Z-Axis", _DisplayMsg);
            plotVector(_Position[robot._RobotID.ToString()].xAxis, "Current X-Axis", _DisplayMsg);
            plotVector(_DesiredPosition[robot._RobotID.ToString()].xAxis, "Desired X-Axis", _DisplayMsg);
            plotVector(_Position[robot._RobotID.ToString()].yAxis, "Current Y-Axis", _DisplayMsg);
            plotVector(_DesiredPosition[robot._RobotID.ToString()].yAxis, "Desired Y-Axis", _DisplayMsg);
            plotDoubleQue(robot.JocTimer, "FK / IK Time", _DisplayMsg);
            plotDoubleQue(robot.MaxJocTimer, "Max FK / IK Time", _DisplayMsg);
            plotDoubleQue(robot._trajectoryLoaderTime, "Trajectory Load Time", _DisplayMsg);
            /*
        public FixedSizedQueue<double> _maxProcessDataTimer = new FixedSizedQueue<double>(10);
        public FixedSizedQueue<double> _trajectoryLoaderTime = new FixedSizedQueue<double>(10);
        public FixedSizedQueue<double> _processDataTimer = new FixedSizedQueue<double>(10);
        public FixedSizedQueue<double> JocTimer = new FixedSizedQueue<double>(10);
        public FixedSizedQueue<double> MaxJocTimer = new FixedSizedQueue<double>(10);
        public FixedSizedQueue<double> serverTimer = new FixedSizedQueue<double>(10);
        public FixedSizedQueue<double> MaxserverTimer = new FixedSizedQueue<double>(10);
            
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
            foreach (var item in MaxJocTimer.ThreadSafeToArray)
            {
                Console.Write(" {0} ", item.ToString());
            }
            Console.WriteLine(")");
            Console.WriteLine("server time:   \n(");
            foreach (var item in serverTimer.ThreadSafeToArray)
            {
                Console.Write(" {0} ", item.ToString());
            }
            Console.WriteLine(")");
            Console.WriteLine("Maximum server time:   \n(");
            foreach (var item in MaxserverTimer.ThreadSafeToArray)
            {
                Console.Write(" {0} ", item.ToString());
            }
            Console.WriteLine(")");

             * 
             * 
             */

        }
        void plotDoubleQue(FixedSizedQueue<double> que, string Heading, StringBuilder collection)
        {
            if (que.Count > 0)
            {
                double[] array = que.ThreadSafeToArray;
                int length = array.Length;
                if (Heading.Length < 24)
                {
                    string newheading = string.Concat(Heading, ":", new string(' ', 24 - Heading.Length));
                }
                collection.Append(String.Format("{0,-25}\n (", Heading));
                for (int i = 0; i < length; i++)
                {
                    collection.Append(String.Format("{0:0.00},", array[i]));
                }
                collection.AppendLine(")");
            }
        }

        void plotDoubles(double[] array, string Heading, StringBuilder collection)
        {
            if (Heading.Length < 24)
            {
                string newheading = string.Concat(Heading, ":", new string(' ', 24 - Heading.Length));
            }
            collection.AppendLine(String.Format("{0,-25}\t ({1:0.000},{2:0.000},{3:0.000},{4:0.000},{5:0.000},{6:0.000})", Heading,
                                                                    array[0],
                                                                    array[1],
                                                                    array[2],
                                                                    array[3],
                                                                    array[4],
                                                                    array[5]));
        }

        void plotVector(Vector3 Vector, string Heading, StringBuilder collection)
        {
            if (Heading.Length < 24)
            {
                string newheading = string.Concat(Heading, ":", new string(' ', 24 - Heading.Length));
            }
            collection.AppendLine(String.Format("{0:-20}\t ({1:0.0},{2:0.0},{3:0.0})", Heading, Vector.X, Vector.Y, Vector.Z));
        }

        void plotPose(Pose pose, string Heading, StringBuilder collection)
        {
            if (Heading.Length < 24)
            {
                string newheading = string.Concat(Heading, ":", new string(' ', 24 - Heading.Length));
            }
            collection.AppendLine(String.Format("{0:-20}\t {1:Display}", Heading,pose));
        }


        void plotTrajectories(TaskTrajectory[] Trajectories)
        {
            try
            {
                StringBuilder msg = new StringBuilder();
                msg.AppendLine("  Time  |     Start Pos     |     Final Pos     |     Start Vel     |    Final Vel      |   Angle   ");
                msg.AppendLine("  [s]   |       [mm]        |       [mm]        |       [mm]        |       [mm]        |   [Deg]   ");
                for (int i = 0; i < Trajectories.Length; i++)
                {
                    msg.AppendLine(String.Format("{0,-8:0.00}|{1,9:G}|{2,9:G}|{3,10:G}|{4,10:G}|{5,-7:0.00}", Trajectories[i].trajectoryTime.TotalSeconds,
                                                                                                            Trajectories[i].startPose,
                                                                                                            Trajectories[i].finalPose,
                                                                                                            Trajectories[i].startVelocity,
                                                                                                            Trajectories[i].finalVelocity,
                                                                                                            Trajectories[i].finalAngle * 180 / Math.PI));
                }

            }
            catch (FormatException fe)
            {
                updateError(fe.Message,fe);
            }
            catch (ArgumentNullException ne)
            {
                updateError(ne.Message,ne);
            }
            catch (Exception e)
            {
                updateError(e.Message,e);
            }
        }


        /// <summary>
        /// Constructs the information to be updated giving unique ID used to populate information later.
        /// </summary>
        /// <param name="data"></param> Data type which is overlaoded for correct display settings
        /// <param name="message"></param> String message to display
        /// <returns></returns>
        public Guid displayInformation(Object data, string message)
        {
            // Sets up data to display with unique ID and populates a dictionary
            return Guid.Empty;
        }

        /// <summary>
        /// Updates information to display to screen. This must be inforamtion which is regerested at compilation
        /// </summary>
        /// <param name="data"></param>
        /// <param name="ID"></param>
        /// <returns></returns>
        public bool updateInfo(Object data, Guid ID)
        {
            // Updates data using an ID tag to know what to do with it.
            return false;
        }


        /// <summary>
        /// Public funtion to update Debug log when catch statement are triggered.
        /// </summary>
        /// <param name="Error"></param>Error trigger
        /// <param name="msg"></param>Custom string to give specific information such as values which failed and why.
        public void Debug(Exception Error, string msg)
        {
            // Writes errors with time stamp to log file.
            // Notify error occured to server and external program if required
        }

        /// <summary>
        /// Public funtion to update Error log when catch statement are triggered. 
        /// Will link important errors to server display or external server such as collisions or automatic trajectory changes.
        /// </summary>
        /// <param name="Error"></param>Error trigger
        /// <param name="msg"></param>Custom string to give specific information such as values which failed and why.
        public void Error(Exception Error, string msg)
        {
            // Writes errors with time stamp to log file.
            // Notify error occured to server and external program if required
        }

        /// <summary>
        /// Creates a stream/file writer to save data in csv format. Returns the index of the data and links with columns of the csv document.
        /// </summary>
        /// <param name="DataName"></param>String of the name of the file to be created.
        /// <param name="data"></param>Object of data to be written, TODO: overload with known types.
        /// <returns></returns>
        public int CreateData(string DataName, object data)
        {
            // Dummy method to create data writer.
            // Overload with different inputs, this will set the array and link index with columns required.
            // RETURN int of index for WriteData reference
            // TODO: Don't use -1 but use errors and event handeler.
            return -1;
        }



        /// <summary>
        /// Adds information to be printed to csv file. Returns false if data file was not created.
        /// </summary>
        /// <param name="DataName"></param>String of the name of the file to be created.
        /// <param name="time"></param>Time of the data to be added / captured. Expressed in ms and will save in bins of 4ms
        /// <param name="index"></param>ID of the data to be written as per CreateData result.
        /// <param name="data"></param>Object of data to be written, TODO: overload with known types.
        /// <returns></returns>
        public bool WriteData(string DataName, double time, int index, object data)
        {
            // Dummy method to write data to file. 
            // Overloaded with variouse inputs such as Pose, double, double[,] ect, 
            // RETURN bool if data was added,
            // TODO: Don't use bool but use errors and event handeler.
            return false;
        }

        internal void updateLog(string Logmsg)
        {
            lock (debuggerLock)
            {
                //long timestamp = DateTime.Now.Ticks;
                //_Debugger.AppendLine(string.Format("{0:hh:mm:ss:fff} | {1:0.0}ms ago.", timestamp, 1.0 * (lastLog - timestamp) / TimeSpan.TicksPerMillisecond));
                _Debugger.AppendLine(Logmsg);
                //lastLog = timestamp;
            }
        }

        internal void updateCSVLog(string Logmsg)
        {
            lock (CSVdebuggerLock)
            {
                //long timestamp = DateTime.Now.Ticks;
                //_Debugger.AppendLine(string.Format("{0:hh:mm:ss:fff} | {1:0.0}ms ago.", timestamp, 1.0 * (lastLog - timestamp) / TimeSpan.TicksPerMillisecond));
                _CSVdebugData.AppendLine(Logmsg);
                //lastLog = timestamp;
            }
        }
    }
}
