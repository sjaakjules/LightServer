using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Net;
using System.Net.Sockets;
using System.Collections.Concurrent;
using System.Threading;
using System.Xml;
using System.IO;
using System.Diagnostics;

namespace LightWeight_Server
{
    // State object for reading client data asynchronously
    public class StateObject
    {
        // Client  socket.
        public Socket socket = null;
        // Client EP.
        public EndPoint clientEP = null;
        // Client IPEP.
        public IPEndPoint clientIpEP = null;
        // Size of receive buffer.
        public int PacketInSize;
        // Receive buffer.
        public byte[] PacketIn;
        // Received data string.
        public string MessageIn;
        // Size of sending buffer.
        public int PacketOutSize;
        // Sending buffer.
        public byte[] PacketOut;
        // Sending XML document
        public XmlDocument XMLout;
        // Sending data string.
        public string MessageOut;
        // Received IPOC.
        public long IPOC = 0;
        // Received timestamp
        public long timstamp = 0;
        // Received motor angles.
        public double[] Angles = new double[6];
        // Received cartesian pose
        public Pose EEPose = Pose.Zero;
        // trigger that the message is loaded into the state holder
        public bool hasLoadedMessageOut;

    }


    class KukaServer
    {

        // Thread signals to pause until data has been received
        ManualResetEventSlim haveReceived = new ManualResetEventSlim(false);
        ManualResetEventSlim haveUpdatedPositions = new ManualResetEventSlim(false);

        StateObject lastPacket = null;
        ConcurrentQueue<StateObject> FreshPackets = new ConcurrentQueue<StateObject>();

        int _BufferSize = 1024;
        byte[] _buffer;
        Socket _UdpSocket;
        IPEndPoint _localEP;
        int _Port = 6008;
        XmlDocument _SendXML;
        long _IPOC, _LIPOC;

        Stopwatch processDataTimer = new Stopwatch();
        Stopwatch serverTime = new Stopwatch();
        Stopwatch sendTimer = new Stopwatch();

        RobotInfo _Robot;

        Thread constantSender;

        #region Constructor:
        /// <summary>
        /// Creates a UDP server with XML read and write via a port with threadSafe shared robot information
        /// </summary>
        /// <param name="port"></param> The port which communication occurs on
        /// <param name="robot"></param> The robot information to be updated and read from
        public KukaServer(int port, RobotInfo robot)
        {
            _Robot = robot;
            constantSender = new Thread(ConstantSend);
            _Port = port;
            _IPOC = 0;
            _LIPOC = 0;

            SetupXML();


            // Create Socket
            string catchStatement = "while trying to create new socket:";
            try
            {
                _UdpSocket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
            }
            catch (SocketException se)
            {
                _Robot.updateError("SocketException " + catchStatement,se);
            }
            catch (ObjectDisposedException ob)
            {
                _Robot.updateError("ObjectDisposedException " + catchStatement,ob);
            }
            catch (Exception e)
            {
                _Robot.updateError("Generic error " + catchStatement,e);
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
            _Robot.EndPoint = _localEP;
            // Bind the local EP
            string catchStatement = " while trying to bind local EP:";
            try
            {
                _UdpSocket.Bind((EndPoint)_localEP);
                _Robot.updateError("Kuka Server IP bound: " + _UdpSocket.LocalEndPoint.ToString(), new Exception("Kuka server:"));
                Console.WriteLine("Kuka server IP bound: " + _UdpSocket.LocalEndPoint.ToString());
            }
            catch (SocketException se)
            {
                _Robot.updateError("SocketException " + catchStatement, se);
            }
            catch (ObjectDisposedException ob)
            {
                _Robot.updateError("ObjectDisposedException " + catchStatement,ob);
            }
            catch (Exception e)
            {
                _Robot.updateError("Generic error " + catchStatement,e);
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
                Console.WriteLine(Dns.GetHostName().ToString());

                for (int i = 0; i < ipHostInfo.AddressList.Length; i++)
                {
                    Console.WriteLine("[{0}] : {1}", i, ipHostInfo.AddressList[i]);
                }
                Console.WriteLine("[L] : Local Host");
                Console.WriteLine("Select IP to bind Kuka server...");

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
                _Robot.updateError("SocketException " + catchStatement,se);
                return new IPEndPoint(IPAddress.Parse("127.0.0.1"), _Port);
            }
            catch (ObjectDisposedException ob)
            {
                _Robot.updateError("ObjectDisposedException " + catchStatement,ob);
                return new IPEndPoint(IPAddress.Parse("127.0.0.1"), _Port);
            }
            catch (Exception e)
            {
                _Robot.updateError("Generic error " + catchStatement,e);
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
                    //      _Robot.updateError("Listening for data on port: " + _Port);
                }
                catch (SocketException se)
                {
                    _Robot.updateError("Socket Exception " + catchStatement,se);
                    //_Robot.Disconnect();
                    haveReceived.Set();
                }
                catch (ObjectDisposedException ob)
                {
                    _Robot.updateError("ObjectDisposedException " + catchStatement,ob);
                }
                catch (Exception e)
                {
                    _Robot.updateError("Generic error " + catchStatement,e);
                }
                // pause This thread until a packet has been returned.

                haveReceived.Wait();
            }

        }

        public void FinishReceiveFrom(IAsyncResult ar)
        {
           // _Robot.Connect();
            string catchStatement = "while receive data is active, reading data:";
            try
            {
                Stopwatch serverTimer = new Stopwatch();
                serverTimer.Start();
                // get the object passed into the asyc function
                StateObject connectedState = (StateObject)ar.AsyncState;

                // end the receive and storing size of data in state
                connectedState.PacketInSize = connectedState.socket.EndReceiveFrom(ar, ref connectedState.clientEP);
                // Initialize the state buffer with the received data size and copy buffer to state
                connectedState.PacketIn = new byte[connectedState.PacketInSize];
                Array.Copy(_buffer, connectedState.PacketIn, connectedState.PacketInSize);
                // Retrieve EP information and store in state
                connectedState.clientIpEP = (IPEndPoint)connectedState.clientEP;

                // Reset the global buffer to null ready to be initialised in next receive loop once packet as been sent.
                _buffer = null;

                haveReceived.Set();
                sendTimer.Restart();
                // Process byte information on state object
                processData(connectedState);
                // _Robot.addMsg(connectedState.MessageIn);

                // Send return message to same connection that the data was received.
               // SendData(connectedState);
                serverTime.Stop();
                _Robot.updateServerTime(serverTimer.Elapsed.TotalMilliseconds);
                serverTimer.Reset();

            }
            catch (SocketException se)
            {
                _Robot.updateError("SocketException " + catchStatement,se);
            }
            catch (ObjectDisposedException ob)
            {
                _Robot.updateError("ObjectDisposedException " + catchStatement,ob);
            }
            catch (Exception e)
            {
                _Robot.updateError("Generic error " + catchStatement,e);
            }


        }

        private void processData(StateObject State)
        {
            string catchStatement = "while trying to process Data:";
            try
            {
                processDataTimer.Restart();
                // Encode msg from state object
                State.MessageIn = Encoding.UTF8.GetString(State.PacketIn, 0, State.PacketInSize);

                // create xml document from state message in.
                XmlDocument xmlIn = new XmlDocument();
                xmlIn.LoadXml(State.MessageIn);
                XmlNode IpocNode = xmlIn.SelectSingleNode("//Rob/IPOC");
                _LIPOC = _IPOC;
                if (long.TryParse(IpocNode.InnerText, out _IPOC))
                {
                    // Was succsessful, check if order is correct
                    if (_LIPOC > _IPOC)
                    {
                        _Robot.updateError("Error, packet order incorrect: New IPOC: " + _IPOC + " Old IPOC: " + _LIPOC, new Exception("Kuka server:"));
                    }
                    State.IPOC = _IPOC;
                }
                else
                {
                    _Robot.updateError("Error not reading IPOC: ", new Exception("Kuka server:"));
                }

                FreshPackets.Enqueue(State);
                haveUpdatedPositions.Set();

                XmlNodeList parentNode = xmlIn.ChildNodes;
                XmlNodeList KukaInfoNodes = parentNode.Item(0).ChildNodes;
                Pose newPose = Pose.Zero;
                double[] newAngles = new double[6];
                bool updatedPosition = false;
                bool updatedAngles = false;
                foreach (XmlNode Node in KukaInfoNodes)
                {
                    switch (Node.Name)
                    {
                        case "RIst":
                            newPose = new Pose(new double[] {double.Parse(Node.Attributes["X"].Value), double.Parse(Node.Attributes["Y"].Value),
                                            double.Parse(Node.Attributes["Z"].Value), double.Parse(Node.Attributes["A"].Value),
                                            double.Parse(Node.Attributes["B"].Value), double.Parse(Node.Attributes["C"].Value)});
                            State.EEPose = newPose;
                            updatedPosition = true;
                            break;

                        case "AIPos":
                            newAngles[0] = double.Parse(Node.Attributes["A1"].Value) * 1.0 * Math.PI / 180;
                            newAngles[1] = double.Parse(Node.Attributes["A2"].Value) * 1.0 * Math.PI / 180;
                            newAngles[2] = double.Parse(Node.Attributes["A3"].Value) * 1.0 * Math.PI / 180;
                            newAngles[3] = double.Parse(Node.Attributes["A4"].Value) * 1.0 * Math.PI / 180;
                            newAngles[4] = double.Parse(Node.Attributes["A5"].Value) * 1.0 * Math.PI / 180;
                            newAngles[5] = double.Parse(Node.Attributes["A6"].Value) * 1.0 * Math.PI / 180;
                            State.Angles = newAngles;
                            updatedAngles = true;
                            break;

                        case "Torque":
                            break;
                        case "Robot":
                            _Robot.updateSignal(double.Parse(Node.Attributes["Active1"].Value), double.Parse(Node.Attributes["Active2"].Value));
                            break;
                        default:
                            break;
                    }
                }
                State.timstamp = DateTime.Now.Ticks;

                if (updatedAngles)
                {
                    _Robot.UpdateAngles(newAngles);
                }
                if (updatedPosition)
                {
                    _Robot.updateRobotPose(newPose);
                }


                if (_Robot.Connect())
                {
                    constantSender.Start();
                }

                // As the robot positions have been updated, calculate change in position and update command dictionary

                _Robot.updateComandPosition(newPose, newAngles);

                //_Robot.UpdateAngles(newAngles);
                _Robot.updateRobotPosition(newPose, _IPOC);

                processDataTimer.Stop();
                _Robot._processDataTimer.Enqueue(processDataTimer.Elapsed.TotalMilliseconds);
                if (processDataTimer.Elapsed.TotalMilliseconds > 4)
                {
                    _Robot._maxProcessDataTimer.Enqueue(processDataTimer.Elapsed.TotalMilliseconds);
                }
            }
            catch (SocketException se)
            {
                _Robot.updateError("SocketException " + catchStatement,se);
            }
            catch (ObjectDisposedException ob)
            {
                _Robot.updateError("ObjectDisposedException " + catchStatement,ob);
            }
            catch (Exception e)
            {
                _Robot.updateError("Generic error " + catchStatement,e);
            }
        }

        private void SendData(StateObject state)
        {
            string catchStatement = "while trying to send the data:";
            try
            {
                if (state.hasLoadedMessageOut)
                {
                    state.socket.BeginSendTo(state.PacketOut, 0, state.PacketOut.Length, SocketFlags.None, state.clientEP, new AsyncCallback(FinishSendTo), state);
                }
                else
                {
                    _Robot.updateError("Couldn't write message", new Exception("Kuka server:"));
                }
            }
            catch (SocketException se)
            {
                _Robot.updateError("SocketException " + catchStatement,se);
            }
            catch (ObjectDisposedException ob)
            {
                _Robot.updateError("ObjectDisposedException " + catchStatement,ob);
            }
            catch (Exception e)
            {
                _Robot.updateError("Generic error " + catchStatement,e);
            }
            // Save state of the kuka server
            //_Robot.DataHistory.Push(state);
           // haveReceived.Set();
        }

        private void FinishSendTo(IAsyncResult ar)
        {
            string catchStatement = "while trying to finsh sendTo:";
            try
            {
                //_Robot.updateError("Take that!");
                StateObject state = (StateObject)ar.AsyncState;
                int bytesSent = state.socket.EndSendTo(ar);
            }
            catch (SocketException se)
            {
                _Robot.updateError("SocketException " + catchStatement,se);
            }
            catch (ObjectDisposedException ob)
            {
                _Robot.updateError("ObjectDisposedException " + catchStatement,ob);
            }
            catch (Exception e)
            {
                _Robot.updateError("Generic error " + catchStatement,e);
            }
        }

        public void ConstantSend()
        {
            sendTimer.Start();
            while (true)
            {
                haveUpdatedPositions.Reset();
                double[] newCommand;
                if (!_Robot._Commands.TryDequeue(out newCommand))
                {
                    newCommand = new double[] { 0, 0, 0, 0, 0, 0 };
                }
                StateObject lastlastpacket = lastPacket;
                if (FreshPackets.TryDequeue(out lastPacket))
                {
                    UpdateXML(lastPacket, newCommand);
                    SendData(lastPacket);
                    if (lastlastpacket != null && lastPacket.IPOC < lastlastpacket.IPOC)
                    {
                    _Robot.updateError(string.Format("send Ipoc was out of order with Ipoc{0} sent and {1} sent last time",lastPacket.IPOC,lastlastpacket.IPOC), new KukaException("Constrant sender error"));
                    }
                }
                else if (lastPacket != null)
                {
                    _Robot.updateError("Last packet was not updated by the time to send!!", new KukaException("Constrant sender error"));
                    UpdateXML(lastPacket, newCommand);
                    SendData(lastPacket);
                }
                else
                {
                    _Robot.updateError("Last packet was not updated by the time to send ever!!", new KukaException("Constrant sender error"));
                }
                if (sendTimer.Elapsed.TotalMilliseconds > 4)
                {
                    _Robot.updateError(string.Format("Send took more than 4ms!!!, was {0}ms",sendTimer.Elapsed.TotalMilliseconds), new KukaException("Constrant sender error"));
                }
                using (StreamWriter Datafile = new StreamWriter("SendInfo" + ".txt", true))
                {
                    Datafile.WriteLine("{0:mm:ss:ffff}", DateTime.Now);
                    Datafile.WriteLine("{0:0.00}", sendTimer.Elapsed.TotalMilliseconds);
                    Datafile.WriteLine(Encoding.ASCII.GetChars(lastPacket.PacketOut));
                }
                haveUpdatedPositions.Wait();
            }
        }

        private void SetupXML()
        {
            _SendXML = new XmlDocument();
            XmlNode rootNode = _SendXML.CreateElement("Sen");
            XmlAttribute attribute = _SendXML.CreateAttribute("Type");
            attribute.Value = "ImFree";
            rootNode.Attributes.Append(attribute);
            _SendXML.AppendChild(rootNode);

            XmlNode errorNode = _SendXML.CreateElement("EStr");
            errorNode.InnerText = "";
            rootNode.AppendChild(errorNode);

            XmlNode techNode = _SendXML.CreateElement("Tech");
            attribute = _SendXML.CreateAttribute("T21");
            attribute.Value = "1.09";
            techNode.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("T22");
            attribute.Value = "2.08";
            techNode.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("T23");
            attribute.Value = "3.07";
            techNode.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("T24");
            attribute.Value = "4.06";
            techNode.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("T25");
            attribute.Value = "5.05";
            techNode.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("T26");
            attribute.Value = "6.04";
            techNode.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("T27");
            attribute.Value = "7.03";
            techNode.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("T28");
            attribute.Value = "8.02";
            techNode.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("T29");
            attribute.Value = "9.01";
            techNode.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("T210");
            attribute.Value = "10.00";
            techNode.Attributes.Append(attribute);
            rootNode.AppendChild(techNode);
            /*
            XmlNode comPosNode = _SendXML.CreateElement("RKorr");
            attribute = _SendXML.CreateAttribute("X");
            attribute.Value = "0.0000";
            comPosNode.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("Y");
            attribute.Value = "0.0000";
            comPosNode.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("Z");
            attribute.Value = "0.0000";
            comPosNode.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("A");
            attribute.Value = "0.0000";
            comPosNode.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("B");
            attribute.Value = "0.0000";
            comPosNode.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("C");
            attribute.Value = "0.0000";
            comPosNode.Attributes.Append(attribute);
            rootNode.AppendChild(comPosNode);
            */
            
            XmlNode comPosNode = _SendXML.CreateElement("AKorr");
            attribute = _SendXML.CreateAttribute("A1");
            attribute.Value = "0.0000";
            comPosNode.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("A2");
            attribute.Value = "0.0000";
            comPosNode.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("A3");
            attribute.Value = "0.0000";
            comPosNode.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("A4");
            attribute.Value = "0.0000";
            comPosNode.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("A5");
            attribute.Value = "0.0000";
            comPosNode.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("A6");
            attribute.Value = "0.0000";
            comPosNode.Attributes.Append(attribute);
            rootNode.AppendChild(comPosNode);
             

            XmlNode gripper = _SendXML.CreateElement("GRIPPER");
            attribute = _SendXML.CreateAttribute("A");
            attribute.Value = "0";
            gripper.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("B");
            attribute.Value = "0";
            gripper.Attributes.Append(attribute);
            rootNode.AppendChild(gripper);

            XmlNode IpocNode = _SendXML.CreateElement("IPOC");
            IpocNode.InnerText = "0";
            rootNode.AppendChild(IpocNode);

        }

        public void UpdateXML(StateObject state, double[] newCommand)
        {
            XmlNode IpocNode = _SendXML.SelectSingleNode("//Sen/IPOC");
            IpocNode.InnerText = state.IPOC.ToString();

            XmlNode comAxisNode = _SendXML.SelectSingleNode("//Sen/AKorr");
            for (int i = 0; i < 6; i++)
            {
                comAxisNode.Attributes[SF.axisKeys[i]].Value = String.Format("{0:0.0000000}", newCommand[i] * 4.0 * 180.0 / Math.PI); //* 180.0 / Math.PI
            }
            XmlNode gripperNode = _SendXML.SelectSingleNode("//Sen/GRIPPER");
            if (false)
            {
                gripperNode.Attributes["A"].Value = "1";
                gripperNode.Attributes["B"].Value = "0";
            }
            else
            {
                gripperNode.Attributes["A"].Value = "0";
                gripperNode.Attributes["B"].Value = "1";
            }

            state.XMLout = (XmlDocument)_SendXML.Clone();

            state.PacketOut = Encoding.UTF8.GetBytes(Beautify(state.XMLout));
            state.hasLoadedMessageOut = true;
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


        #endregion
    }


}
