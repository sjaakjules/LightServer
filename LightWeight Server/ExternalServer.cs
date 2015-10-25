using Microsoft.Xna.Framework;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Xml;

namespace LightWeight_Server
{
    class ExternalServer
    {
        // Thread signals to pause until data has been received
        ManualResetEvent haveReceived = new ManualResetEvent(false);
        object errorMsgLock = new object();

        int _BufferSize = 20*1024;
        byte[] _buffer;
        Socket _UdpSocket;
        IPEndPoint _localEP;
        int _Port;
        XmlDocument _SendXML;
        bool _loadedPosition = false;
        bool _loadedRotation = false;
        bool _loadedPoses = false;
        string[] splitter = new string[] { "," };
        FixedSizedQueue<IPEndPoint> ClientIEP;
        int _SendRefreshRate = 10; // Refresh rate of send data, Hz

        StringBuilder _errorMessage = new StringBuilder();

        Stopwatch _sendTimer = new Stopwatch();

        

        RobotInfo[] _Robot;
        ScreenWriter _GUI;

        #region Constructor:
        /// <summary>
        /// Creates a UDP server with XML read and write via a port with threadSafe shared robot information
        /// </summary>
        /// <param name="port"></param> The port which communication occurs on
        /// <param name="robot"></param> The robot information to be updated and read from
        public ExternalServer(int port, RobotInfo[] robot, ScreenWriter Gui)
        {
            _Robot = robot;
            _GUI = Gui;
            _Port = port;
            ClientIEP = new FixedSizedQueue<IPEndPoint>(1000);
            _sendTimer.Start();
            SetupXML();

            // Create Socket
            string catchStatement = "while trying to create new socket:";
            try
            {
                _UdpSocket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
            }
            catch (SocketException se)
            {
                _GUI.updateError("SocketException " + catchStatement, se);
            }
            catch (ObjectDisposedException ob)
            {
                _GUI.updateError("ObjectDisposedException " + catchStatement,ob);
            }
            catch (Exception e)
            {
                _GUI.updateError("Generic error " + catchStatement,e);
            }
            // Binds the socket to local IP.
            bindSocket();
        }



        void setupDictionaries(ConcurrentDictionary<string, double> dic)
        {
            dic.TryAdd("X", 0);
            dic.TryAdd("Y", 0);
            dic.TryAdd("Z", 0);
            dic.TryAdd("A", 0);
            dic.TryAdd("B", 0);
            dic.TryAdd("C", 0);
        }

        #endregion


        #region General Server methods
        public void Shutdown()
        {

        }

        public void bindSocket()
        {
            // Finds local EP to bind socket
            _localEP = getAvailableIpEP();

            // Bind the local EP
            string catchStatement = " while trying to bind local EP:";
            try
            {
                _UdpSocket.Bind((EndPoint)_localEP);
                _GUI.updateError("External Server IP bound: " + _UdpSocket.LocalEndPoint.ToString(), new Exception("External Server: "));
                Console.WriteLine("External Server IP bound: " + _UdpSocket.LocalEndPoint.ToString());
            }
            catch (SocketException se)
            {
                _GUI.updateError("SocketException " + catchStatement, se);
            }
            catch (ObjectDisposedException ob)
            {
                _GUI.updateError("ObjectDisposedException " + catchStatement, ob);
            }
            catch (Exception e)
            {
                _GUI.updateError("Generic error " + catchStatement, e);
            }
        }

        private IPEndPoint getAvailableIpEP()
        {
            string catchStatement = "while trying to get local IP endpoint:";
            try
            {
                // Finds the DNS name of the computer and prints to screen.
                //IPHostEntry ipHostInfo = Dns.GetHostEntry(Dns.GetHostName());
                IPHostEntry ipHostInfo = Dns.Resolve(Dns.GetHostName());

                Console.WriteLine("Select IP to bind External server...");

                // Selects the first IP in the list and writes it to screen
                int addressSelction = Convert.ToInt32(Console.ReadKey(true).KeyChar) - 48;
                while (addressSelction >= ipHostInfo.AddressList.Length)
                {

                    if (addressSelction == 28 || addressSelction == 60)
                    {
                        return new IPEndPoint(IPAddress.Parse("127.0.0.1"), _Port);
                    }
                    Console.WriteLine("Please use numbers only or Press L for local host.");
                    addressSelction = Convert.ToInt32(Console.ReadKey(true).KeyChar) - 48;
                }
                return new IPEndPoint(ipHostInfo.AddressList[addressSelction], _Port);
            }
            catch (SocketException se)
            {
                _GUI.updateError("SocketException " + catchStatement, se);
                return new IPEndPoint(IPAddress.Parse("127.0.0.1"), _Port);
            }
            catch (ObjectDisposedException ob)
            {
                _GUI.updateError("ObjectDisposedException " + catchStatement, ob);
                return new IPEndPoint(IPAddress.Parse("127.0.0.1"), _Port);
            }
            catch (Exception e)
            {
                _GUI.updateError("Generic error " + catchStatement, e);
                return new IPEndPoint(IPAddress.Parse("127.0.0.1"), _Port);
            }
        }

        public void ConstantReceive()
        {
            while (true)
            {
                // resets the event to nonsignaled state.
                haveReceived.Reset();


                // Resets buffer and other variables loosing any unsaved data
                _buffer = new byte[_BufferSize];

                // Creates new State object which will have information for this dataGram communication.
                StateObject newState = new StateObject();
                newState.socket = _UdpSocket;

                string catchStatement = "while trying to begin receiving data:";
                try
                {
                    newState.clientEP = (EndPoint)new IPEndPoint(IPAddress.Any, _Port); ;
                    _UdpSocket.BeginReceiveFrom(_buffer, 0, _BufferSize, SocketFlags.None, ref newState.clientEP, new AsyncCallback(FinishReceiveFrom), newState);
                }
                catch (SocketException se)
                {
                    _GUI.updateError("SocketException " + catchStatement, se);
                }
                catch (ObjectDisposedException ob)
                {
                    _GUI.updateError("ObjectDisposedException " + catchStatement, ob);
                }
                catch (Exception e)
                {
                    _GUI.updateError("Generic error " + catchStatement, e);
                }

                // pause This thread until a packet has been returned.
                haveReceived.WaitOne();
            }

        }

        public void FinishReceiveFrom(IAsyncResult ar)
        {
            string catchStatement = "while receive data is active, reading data:";
            try
            {

                // get the object passed into the asyc function
                StateObject connectedState = (StateObject)ar.AsyncState;

                // end the receive and storing size of data in state
                connectedState.PacketInSize = connectedState.socket.EndReceiveFrom(ar, ref connectedState.clientEP);
                // Initialize the state buffer with the received data size and copy buffer to state
                connectedState.PacketIn = new byte[connectedState.PacketInSize];
                Array.Copy(_buffer, connectedState.PacketIn, connectedState.PacketInSize);
                // Retrieve EP information and store in state
                connectedState.clientIpEP = (IPEndPoint)connectedState.clientEP;
                bool hasAdded = false;
                IPEndPoint[] ClintList = ClientIEP.ThreadSafeToArray;
                foreach (var Client in ClintList)
                {
                    if (Client == connectedState.clientIpEP)
                    {
                        hasAdded = true;
                        break;
                    }
                }
                if (!hasAdded)
                {
                    ClientIEP.Enqueue(new IPEndPoint(connectedState.clientIpEP.Address, connectedState.clientIpEP.Port));
                }

                // Reset the global buffer to null ready to be initialised in next receive loop once packet as been sent.
                _buffer = null;

                // Process byte information on state object
                processData(connectedState);

                haveReceived.Set();
                // Send return message to same connection that the data was received.
                //SendData(connectedState);
            }
            catch (SocketException se)
            {
                _GUI.updateError("SocketException " + catchStatement, se);
            }
            catch (ObjectDisposedException ob)
            {
                _GUI.updateError("ObjectDisposedException " + catchStatement, ob);
            }
            catch (Exception e)
            {
                _GUI.updateError("Generic error " + catchStatement, e);
            }
        }

        private void FinishSendTo(IAsyncResult ar)
        {
            string catchStatement = "while trying to finsh sendTo:";
            try
            {
                //Console.WriteLine("Take that!");
                StateObject state = (StateObject)ar.AsyncState;
                int bytesSent = state.socket.EndSendTo(ar);
            }
            catch (SocketException se)
            {
                _GUI.updateError("SocketException " + catchStatement, se);
            }
            catch (ObjectDisposedException ob)
            {
                _GUI.updateError("ObjectDisposedException " + catchStatement, ob);
            }
            catch (Exception e)
            {
                _GUI.updateError("Generic error " + catchStatement, e);
            }
        }

        public void ConstantSendData()
        {
            while (true)
            {
                if (ClientIEP.Count > 0)
                {
                    IPEndPoint[] clientList = ClientIEP.ThreadSafeToArray;
                    foreach (var Client in clientList)
                    {
                        StateObject state = new StateObject();
                        state.socket = _UdpSocket;
                        state.clientEP = (EndPoint)Client;
                        state.clientIpEP = Client;
                        UpdateXML(state);
                        string catchStatement = "while trying to send the data:";
                        try
                        {
                            if (state.hasLoadedMessageOut)
                            {
                                state.socket.BeginSendTo(state.PacketOut, 0, state.PacketOut.Length, SocketFlags.None, state.clientEP, new AsyncCallback(FinishSendTo), state);
                            }
                            else
                            {
                                _GUI.updateError("Couldn't write message", new Exception("External Server:"));
                            }
                        }
                        catch (SocketException se)
                        {
                            _GUI.updateError("SocketException " + catchStatement, se);
                        }
                        catch (ObjectDisposedException ob)
                        {
                            _GUI.updateError("ObjectDisposedException " + catchStatement, ob);
                        }
                        catch (Exception e)
                        {
                            _GUI.updateError("Generic error " + catchStatement, e);
                        }
                    }
                    Thread.Sleep(1000 / _SendRefreshRate);
                }
            }

        }

        void AddError(string msg)
        {
            lock (errorMsgLock)
            {
                _errorMessage.AppendLine(msg);
            }
            _GUI.updateError(msg,new KukaException("Oh no, What the fuck happened?\n"));
        }

        bool getPoseInfo(XmlNode pose,Pose lastPose, out Pose endPose, out double EndVelocity, out double AveVelocity, out TrajectoryTypes type)
        {
            type = TrajectoryTypes.Quintic;
            EndVelocity = -1;
            AveVelocity = -1;
            string[] PositionStrings = null;
            string[] orientationStrings = null;
            bool loaded = false;
            foreach (XmlAttribute atribute in pose.Attributes)
            {
                switch (atribute.Name)
                {
                    case "Type":
                        if (atribute.Value.ToUpper().Equals("QUINTIC"))
                        {
                            type = TrajectoryTypes.Quintic;
                        }
                        else if (atribute.Value.ToUpper().Equals("SPLINE"))
                        {
                            type = TrajectoryTypes.Spline;
                        }
                        else if (atribute.Value.ToUpper().Equals("LINEAR"))
                        {
                            type = TrajectoryTypes.Linear;
                        }
                        break;
                    case "Position":
                        string Position = atribute.Value;
                        PositionStrings = Position.Split(splitter, StringSplitOptions.RemoveEmptyEntries);
                        loaded = true;
                        break;
                    case "Orientation":
                        string orientation = atribute.Value;
                        orientationStrings = orientation.Split(splitter, StringSplitOptions.RemoveEmptyEntries);
                        loaded = true;
                        break;
                    case "Velocity":
                        if (!double.TryParse(atribute.Value, out AveVelocity))
                        {
                            AveVelocity = -1;
                        }
                        break;
                    case "FinalVelocity":
                        if (!double.TryParse(atribute.Value, out EndVelocity))
                        {
                            EndVelocity = -1;
                        }
                        break;
                    default:
                        break;
                }
            }
            if (loaded)
            {
                endPose = new Pose(PositionStrings, orientationStrings, lastPose);
                return true;
            }
            endPose = lastPose;
            return false;
        }

        private void processData(StateObject State)
        {
            string catchStatement = "while trying to process Data:";
            try
            {

                // Encode msg from state object
                State.MessageIn = Encoding.UTF8.GetString(State.PacketIn, 0, State.PacketInSize);
                // create xml document from state message in.
                XmlDocument xmlIn = new XmlDocument();
                xmlIn.LoadXml(State.MessageIn);

                // Update desired position from xml document.
                XmlNodeList parentNode = xmlIn.ChildNodes;
                int nRobot = -1;
                if (parentNode.Item(0).Name.ToUpper().Equals("ROBOTA"))
                {
                    nRobot = 0;
                }
                else if (parentNode.Item(0).Name.ToUpper().Equals("ROBOTB"))
                {
                    nRobot = 1;
                }
                else
                {
                    nRobot = 0;
                }
                if (nRobot >= _Robot.Length)
                {
                    AddError("Specified robot is higher than connected robots.");
                }
                else
                {
                    XmlNodeList ExternalInfoNodes = parentNode.Item(0).ChildNodes;
                    foreach (XmlNode Node in ExternalInfoNodes)
                    {
                        switch (Node.Name)
                        {
                            case "PoseList":
                                bool failedUpdate = false;
                                int N = -1;
                                double defaultVelocity = -1;
                                Pose[] FinalPoseList = null;
                                double[] EndVelocityList = null;
                                double[] AveVelocityList = null;
                                TrajectoryTypes[] Trajectorys = null;
                                if (Node.Attributes != null)
                                {
                                    foreach (XmlAttribute attribute in Node.Attributes)
                                    {
                                        switch (attribute.Name)
                                        {
                                            case "N":
                                                if (int.TryParse(attribute.Value, out N))
                                                {
                                                    FinalPoseList = new Pose[N];
                                                    EndVelocityList = new double[N];
                                                    AveVelocityList = new double[N];
                                                    Trajectorys = new TrajectoryTypes[N];
                                                }
                                                else
                                                {
                                                    AddError("Could not read N within PoseList.\nValue should be an int and read \"" + attribute.Value + "\"");
                                                    failedUpdate = true;
                                                }
                                                break;
                                            case "Velocity":
                                                if (!double.TryParse(attribute.Value, out defaultVelocity))
                                                {
                                                    AddError("Could not read default velocity within PoseList.\nUsing T1 default velocity\nValue should be a double and read \"" + attribute.Value + "\"");
                                                }
                                                break;
                                            default:
                                                break;
                                        }
                                    }
                                    if (Node.HasChildNodes && Node.FirstChild.Name != "#text")
                                    {
                                        XmlNodeList PoseList = Node.ChildNodes;
                                        if (PoseList.Count == N)
                                        {
                                            foreach (XmlNode newPose in PoseList)
                                            {
                                                int nPose = -1;
                                                if (int.TryParse(newPose.Attributes["N"].Value, out nPose))
                                                {
                                                    if (nPose > 0 && nPose <= N)
                                                    {
                                                        if (nPose == 1)
                                                        {
                                                            if (!getPoseInfo(newPose, _Robot[nRobot].currentPose, out FinalPoseList[nPose - 1], out EndVelocityList[nPose - 1], out AveVelocityList[nPose - 1], out Trajectorys[nPose - 1]))
                                                            {
                                                                AddError("Failed to update Pose {0}." + nPose.ToString());
                                                                failedUpdate = true;
                                                            }
                                                        }
                                                        else if (!getPoseInfo(newPose, FinalPoseList[nPose - 2], out FinalPoseList[nPose - 1], out EndVelocityList[nPose - 1], out AveVelocityList[nPose - 1], out Trajectorys[nPose - 1]))
                                                        {
                                                            AddError("Failed to update Pose {0}." + nPose.ToString());
                                                            failedUpdate = true;
                                                        }
                                                    }
                                                    else
                                                    {
                                                        AddError(string.Format("Pose number, N, is too high or too low\n{0} poses expected and read N={1}", N, nPose));
                                                        failedUpdate = true;
                                                    }
                                                }
                                                else
                                                {
                                                    AddError("Could not read N within Pose.\nValue should be an int and read \"" + newPose.Attributes["N"].Value + "\"");
                                                    failedUpdate = true;
                                                }
                                            }
                                        }
                                        else
                                        {
                                            AddError(string.Format("More poses detected than expected.\n{0} expected, {1} detected", N, PoseList.Count));
                                            failedUpdate = true;
                                        }
                                    }
                                    else
                                    {
                                        AddError("No poses found. List is empty.");
                                        failedUpdate = true;
                                    }
                                }
                                else
                                {
                                    AddError("No attributes detected in PoseList. Unknown amount of poses.");
                                    failedUpdate = true;
                                }

                                if (!failedUpdate)
                                {
                                    // Loaded all poses and velocities associated with the trajectory of each new pose.
                                    // If errors are encounted during the load it uses last pose as default values
                                    // TODO: if poses are the same they MUST BE REMOVED! this can be handled when creating trajectories.
                                    _loadedPoses = _Robot[nRobot].newPoses(N, FinalPoseList,AveVelocityList, EndVelocityList );
                                }

                                /*
                                int N = 0;
                                if (int.TryParse(Node.Attributes["N"].Value,out N))
                                {
                                    Pose[] poseList = new Pose[N];
                                    double[] velocityList = new double[N];
                                    string Pose_i = Node.Attributes["N1"].Value;
                                    string[] result = Pose_i.Split(splitter, StringSplitOptions.RemoveEmptyEntries);
                                    poseList[0] = new Pose(result, _Robot[0].currentPose);
                                    // Velocity of -1 means use last known velocity.
                                    if (result.Length % 3 == 0)
                                    {
                                        velocityList[0] = -1;
                                    }
                                    else if (!double.TryParse(result[0], out velocityList[0]))
                                    {
                                        velocityList[0] = -1;
                                    }
                                    for (int i = 1; i < N; i++)
                                    {
                                        Pose_i = Node.Attributes["N" + (i+1).ToString()].Value;
                                        result = Pose_i.Split(splitter, StringSplitOptions.RemoveEmptyEntries);
                                        poseList[i] = new Pose(result, poseList[i - 1]);
                                        if (result.Length % 3 == 0)
                                        {
                                            velocityList[i] = -1;
                                        }
                                        else if (!double.TryParse(result[0], out velocityList[i]))
                                        {
                                            velocityList[i] = -1;
                                        }
                                    }
                                    // Loaded all poses and velocities associated with the trajectory of each new pose.
                                    // If errors are encounted during the load it uses last pose as default values
                                    // TODO: if poses are the same they MUST BE REMOVED! this can be handled when creating trajectories.
                                    _loadedPoses = _Robot[0].newPoses(N, poseList, velocityList);
                                }
                                 * 
                                 */
                                break;
                            case "Pose":
                                bool UpdatedPose = false;
                                Pose newFinalPoseList = Pose.Zero;
                                double newEndVelocityList = -1;
                                double newAveVelocityList = -1;
                                TrajectoryTypes trajType = TrajectoryTypes.Quintic;
                                UpdatedPose = getPoseInfo(Node, _Robot[nRobot].currentPose, out newFinalPoseList, out newEndVelocityList, out newAveVelocityList, out trajType);
                                /*
                                int dataPoints = 3;
                                double[] newOrientation = new double[dataPoints + 1];
                                for (int i = 0; i < dataPoints; i++)
                                {
                                    double result;
                                    if (double.TryParse(Node.Attributes[SF.cardinalKeys[i]].Value, out result))
                                    {
                                        newOrientation[i] = result;
                                        newOrientation[dataPoints] += 1;
                                    }
                                    else
                                    {
                                        break;
                                    }
                                }
                                if (newOrientation[dataPoints] == dataPoints)
                                {
                                    _Robot[0].newConOrientation((float)newOrientation[0], (float)newOrientation[1], (float)newOrientation[2]);
                                    _loadedRotation = true;
                                    _GUI.updateError("Rotation loaded", new Exception("External server: "));
                                }
                                 */
                                if (UpdatedPose)
                                {
                                    // Loaded all poses and velocities associated with the trajectory of each new pose.
                                    // If errors are encounted during the load it uses last pose as default values
                                    // TODO: if poses are the same they MUST BE REMOVED! this can be handled when creating trajectories.
                                    _loadedPoses = _Robot[nRobot].newPoses(1, new Pose[] {newFinalPoseList}, new double[] {newEndVelocityList}, new double[] {newAveVelocityList});
                                }
                                else
                                {
                                    AddError("Failed to update individual Pose.");
                                }
                                break;
                            case "Velocty":
                                double newSpeed = 0;
                                if (double.TryParse(Node.InnerText, out newSpeed))
                                {
                                    _Robot[nRobot].LinearVelocity = newSpeed;
                                }
                                else
                                {
                                    AddError("New velocity not read.\nValue should be double but read \"" + Node.InnerText + "\"");
                                }
                                break;
                            case "Gripper":
                                if (int.Parse(Node.InnerText) == 0)
                                {
                                    _Robot[nRobot].gripperIsOpen = false;
                                }
                                else
                                {
                                    _Robot[nRobot].gripperIsOpen = true;
                                }
                                break;
                            /*
                        case "Position":
                            double[] newPosition = new double[4];
                            for (int i = 0; i < 3; i++)
                            {
                                double result;
                                if (double.TryParse(Node.Attributes[SF.cardinalKeys[i]].Value, out result))
                                {
                                    newPosition[i] = result;
                                    newPosition[3] += 1;
                                }
                                else
                                {
                                    break;
                                }
                            }
                            if (newPosition[3] == 3)
                            {
                                _Robot[0].newPosition(newPosition[0], newPosition[1], newPosition[2]);
                                _loadedPosition = true;
                                _GUI.updateError("Position loaded", new Exception("External server: "));
                            }
                            break;

                        case "Rotation":
                            double[] newRotation = new double[10];
                            for (int i = 0; i < 9; i++)
                            {
                                double result;
                                if (double.TryParse(Node.Attributes[SF.rotationKeys[i]].Value, out result))
                                {
                                    newRotation[i] = result;
                                    newRotation[9] += 1;
                                }
                                else
                                {
                                    break;
                                }
                            }
                            if (newRotation[9] == 9)
                            {
                                _Robot[0].newRotation((float)newRotation[0], (float)newRotation[1], (float)newRotation[2],
                                                   (float)newRotation[3], (float)newRotation[4], (float)newRotation[5],
                                                   (float)newRotation[6], (float)newRotation[7], (float)newRotation[8]);
                                _loadedRotation = true;
                                _GUI.updateError("Rotation loaded", new Exception("External server: "));
                            }
                            break;


                        case "PoseXZ":

                            int dataPoint = 6;
                            double[] newXZOrientation = new double[dataPoint + 1];
                            for (int i = 0; i < dataPoint; i++)
                            {
                                double result;
                                if (double.TryParse(Node.Attributes[SF.axisVecotrKeys[i]].Value, out result))
                                {
                                    newXZOrientation[i] = result;
                                    newXZOrientation[dataPoint] += 1;
                                }
                                else
                                {
                                    break;
                                }
                            }
                            if (newXZOrientation[dataPoint] == dataPoint)
                            {
                                _Robot[0].newConOrientation((float)newXZOrientation[0], (float)newXZOrientation[1], (float)newXZOrientation[2], (float)newXZOrientation[3], (float)newXZOrientation[4], (float)newXZOrientation[5]);
                                _loadedRotation = true;
                                _GUI.updateError("Rotation loaded", new Exception("External server: "));
                            }
                            break;

                        case "Via":
                            bool isVia;
                            if (bool.TryParse(Node.InnerText, out isVia))
                            {
                                _Robot[0].isVia = isVia;
                            }
                            break;

                        case "LinearVelocity":
                            double newSpeed = 0;
                            if (double.TryParse(Node.InnerText, out newSpeed))
                            {
                                _Robot[0].LinearVelocity = newSpeed;
                            }
                            break;

                        case "AngularVelocity":
                            double newVelocity = 0;
                            if (double.TryParse(Node.InnerText, out newVelocity))
                            {
                                _Robot[0].AngularVelocity = newVelocity;
                            }
                            break;

                             */
                            default:
                                break;
                        }
                    }
                    if (_loadedPoses)
                    {
                        _loadedPoses = false;
                        _GUI.updateError("Loaded both rotation and position", new Exception("external server:"));
                    }
                }
            }
            catch (SocketException se)
            {
                _GUI.updateError("SocketException " + catchStatement, se);
            }
            catch (ObjectDisposedException ob)
            {
                _GUI.updateError("ObjectDisposedException " + catchStatement, ob);
            }
            catch (Exception e)
            {
                _GUI.updateError("Generic error " + catchStatement, e);
            }
        }

        private void SetupXML()
        {
            _SendXML = new XmlDocument();
            XmlNode rootNode = _SendXML.CreateElement("Robot");
            _SendXML.AppendChild(rootNode);

            XmlNode currentPosition = _SendXML.CreateElement("Position");
            XmlAttribute attribute = _SendXML.CreateAttribute("X");
            attribute.Value = "0.0000";
            currentPosition.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("Y");
            attribute.Value = "0.0000";
            currentPosition.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("Z");
            attribute.Value = "0.0000";
            currentPosition.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("A");
            attribute.Value = "0.0000";
            currentPosition.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("B");
            attribute.Value = "0.0000";
            currentPosition.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("C");
            attribute.Value = "0.0000";
            currentPosition.Attributes.Append(attribute);
            rootNode.AppendChild(currentPosition);

            XmlNode currentVelocity = _SendXML.CreateElement("Velocity");
            attribute = _SendXML.CreateAttribute("X");
            attribute.Value = "0.0000";
            currentVelocity.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("Y");
            attribute.Value = "0.0000";
            currentVelocity.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("Z");
            attribute.Value = "0.0000";
            currentVelocity.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("A");
            attribute.Value = "0.0000";
            currentVelocity.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("B");
            attribute.Value = "0.0000";
            currentVelocity.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("C");
            attribute.Value = "0.0000";
            currentVelocity.Attributes.Append(attribute);
            rootNode.AppendChild(currentVelocity);

            XmlNode currentAcceleration = _SendXML.CreateElement("Acceleration");
            attribute = _SendXML.CreateAttribute("X");
            attribute.Value = "0.0000";
            currentAcceleration.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("Y");
            attribute.Value = "0.0000";
            currentAcceleration.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("Z");
            attribute.Value = "0.0000";
            currentAcceleration.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("A");
            attribute.Value = "0.0000";
            currentAcceleration.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("B");
            attribute.Value = "0.0000";
            currentAcceleration.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("C");
            attribute.Value = "0.0000";
            currentAcceleration.Attributes.Append(attribute);
            rootNode.AppendChild(currentAcceleration);
        }

        public string Beautify(XmlDocument doc)
        {
            StringBuilder sb = new StringBuilder();
            XmlWriterSettings settings = new XmlWriterSettings
            {
                OmitXmlDeclaration = true,
                Indent = true,
                IndentChars = "",
                NewLineChars = "\r\n",
                NewLineHandling = NewLineHandling.Replace
            };
            using (XmlWriter writer = XmlWriter.Create(sb, settings))
            {
                doc.Save(writer);
            }
            return sb.ToString();
        }

        
        /// <summary>
        /// updates the _sendXML XDocument object with position, velocity and acceleration then copies the updated xml document to the state object.
        /// </summary>
        /// <param name="state"></param> State oobject holding the information sent and rceived over the udp server
        void UpdateXML(StateObject state)
        {
            XmlNode currentPosition = _SendXML.SelectSingleNode("//Robot/Position");
            if (currentPosition != null)
            {
                for (int i = 0; i < 6; i++)
                {
                    currentPosition.Attributes[SF.cardinalKeys[i]].Value = String.Format("{0:0.0000}", _Robot[0].currentPose.kukaValues[i]);
                }
            }

            XmlNode currentVelocity = _SendXML.SelectSingleNode("//Robot/Velocity");
            if (currentVelocity != null)
            {
                for (int i = 0; i < 6; i++)
                {
                    currentVelocity.Attributes[SF.cardinalKeys[i]].Value = String.Format("{0:0.0000}", _Robot[0].currentVelocity.kukaValues[i]);
                }
            }

            XmlNode currentAcceleration = _SendXML.SelectSingleNode("//Robot/Acceleration");
            if (currentAcceleration != null)
            {
                for (int i = 0; i < 6; i++)
                {
                    currentAcceleration.Attributes[SF.cardinalKeys[i]].Value = String.Format("{0:0.0000}", _Robot[0].currentAcceleration.kukaValues[i]);
                }
            }

            state.XMLout = (XmlDocument)_SendXML.Clone();

            state.PacketOut = Encoding.UTF8.GetBytes(Beautify(state.XMLout));

            state.hasLoadedMessageOut = true;
        }
        #endregion

    }
}
