using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
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

        int _BufferSize = 1024;
        byte[] _buffer;
        Socket _UdpSocket;
        IPEndPoint _localEP;
        int _Port;
        XmlDocument _SendXML;
        
        RobotInfo _Robot;
        
        #region Constructor:
        /// <summary>
        /// Creates a UDP server with XML read and write via a port with threadSafe shared robot information
        /// </summary>
        /// <param name="port"></param> The port which communication occurs on
        /// <param name="robot"></param> The robot information to be updated and read from
        public ExternalServer(int port, RobotInfo robot)
        {
            _Robot = robot;
            _Port = port;
            
            SetupXML();
            
            // Create Socket
            string catchStatement = "while trying to create new socket:";
            try
            {
                _UdpSocket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
            }
            catch (SocketException se)
            {
                _Robot.updateError("SocketException " + catchStatement);
                _Robot.updateError(se.Message);
            }
            catch (ObjectDisposedException ob)
            {
                _Robot.updateError("ObjectDisposedException " + catchStatement);
                _Robot.updateError(ob.Message);
            }
            catch (Exception e)
            {
                _Robot.updateError("Generic error " + catchStatement);
                _Robot.updateError(e.Message);
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
                _Robot.updateError("External Server IP bound: " + _UdpSocket.LocalEndPoint.ToString());
                Console.WriteLine("External Server IP bound: " + _UdpSocket.LocalEndPoint.ToString());
            }
            catch (SocketException se)
            {
                _Robot.updateError("SocketException " + catchStatement);
                _Robot.updateError(se.Message);
            }
            catch (ObjectDisposedException ob)
            {
                _Robot.updateError("ObjectDisposedException " + catchStatement);
                _Robot.updateError(ob.Message);
            }
            catch (Exception e)
            {
                _Robot.updateError("Generic error " + catchStatement);
                _Robot.updateError(e.Message);
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
                _Robot.updateError(Dns.GetHostName().ToString());

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
                _Robot.updateError("SocketException " + catchStatement);
                _Robot.updateError(se.Message);
                return new IPEndPoint(IPAddress.Parse("127.0.0.1"), _Port);
            }
            catch (ObjectDisposedException ob)
            {
                _Robot.updateError("ObjectDisposedException " + catchStatement);
                _Robot.updateError(ob.Message);
                return new IPEndPoint(IPAddress.Parse("127.0.0.1"), _Port);
            }
            catch (Exception e)
            {
                _Robot.updateError("Generic error " + catchStatement);
                _Robot.updateError(e.Message);
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
                newState.Socket = _UdpSocket;

                string catchStatement = "while trying to begin receiving data:";
                try
                {
                    newState.clientEP = (EndPoint)new IPEndPoint(IPAddress.Any, _Port); ;
                    _UdpSocket.BeginReceiveFrom(_buffer, 0, _BufferSize, SocketFlags.None, ref newState.clientEP, new AsyncCallback(FinishReceiveFrom), newState);
                }
                catch (SocketException se)
                {
                    _Robot.updateError("SocketException " + catchStatement);
                    _Robot.updateError(se.Message);
                }
                catch (ObjectDisposedException ob)
                {
                    _Robot.updateError("ObjectDisposedException " + catchStatement);
                    _Robot.updateError(ob.Message);
                }
                catch (Exception e)
                {
                    _Robot.updateError("Generic error " + catchStatement);
                    _Robot.updateError(e.Message);
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
                connectedState.PacketInSize = connectedState.Socket.EndReceiveFrom(ar, ref connectedState.clientEP);
                // Initialize the state buffer with the received data size and copy buffer to state
                connectedState.PacketIn = new byte[connectedState.PacketInSize];
                Array.Copy(_buffer, connectedState.PacketIn, connectedState.PacketInSize);
                // Retrieve EP information and store in state
                connectedState.clientIpEP = (IPEndPoint)connectedState.clientEP;

                // Reset the global buffer to null ready to be initialised in next receive loop once packet as been sent.
                _buffer = null;

                // Process byte information on state object
                processData(connectedState);
                
                haveReceived.Set();
                // Send return message to same connection that the data was received.
                SendData(connectedState);
            }
            catch (SocketException se)
            {
                _Robot.updateError("SocketException " + catchStatement);
                _Robot.updateError(se.Message);
            }
            catch (ObjectDisposedException ob)
            {
                _Robot.updateError("ObjectDisposedException " + catchStatement);
                _Robot.updateError(ob.Message);
            }
            catch (Exception e)
            {
                _Robot.updateError("Generic error " + catchStatement);
                _Robot.updateError(e.Message);
            }
        }

        private void FinishSendTo(IAsyncResult ar)
        {
            string catchStatement = "while trying to finsh sendTo:";
            try
            {
                //Console.WriteLine("Take that!");
                StateObject state = (StateObject)ar.AsyncState;
                int bytesSent = state.Socket.EndSendTo(ar);
            }
            catch (SocketException se)
            {
                _Robot.updateError("SocketException " + catchStatement);
                _Robot.updateError(se.Message);
            }
            catch (ObjectDisposedException ob)
            {
                _Robot.updateError("ObjectDisposedException " + catchStatement);
                _Robot.updateError(ob.Message);
            }
            catch (Exception e)
            {
                _Robot.updateError("Generic error " + catchStatement);
                _Robot.updateError(e.Message);
            }
        }

        private void SendData(StateObject state)
        {
            string catchStatement = "while trying to send the data:";
            try
            {
                if (state.hasLoadedMessageOut)
                {
                    state.Socket.BeginSendTo(state.PacketOut, 0, state.PacketOut.Length, SocketFlags.None, state.clientEP, new AsyncCallback(FinishSendTo), state);
                }
                else
                {
                    _Robot.updateError("Couldn't write message");
                }
            }
            catch (SocketException se)
            {
                _Robot.updateError("SocketException " + catchStatement);
                _Robot.updateError(se.Message);
            }
            catch (ObjectDisposedException ob)
            {
                _Robot.updateError("ObjectDisposedException " + catchStatement);
                _Robot.updateError(ob.Message);
            }
            catch (Exception e)
            {
                _Robot.updateError("Generic error " + catchStatement);
                _Robot.updateError(e.Message);
            }

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
                //_Robot.updateError(State.MessageIn);
                // Update desired position from xml document.
               // XmlNode parentNode = xmlIn.ChildNodes("//Robot");

                XmlNodeList parentNode = xmlIn.ChildNodes;
                XmlNodeList ExternalInfoNodes = parentNode.Item(0).ChildNodes;
                foreach (XmlNode Node in ExternalInfoNodes)
                {
                    switch (Node.Name)
                    {
                        case "Position":
                            _Robot.DesiredPosition["X"] = double.Parse(Node.Attributes["X"].Value);
                            _Robot.DesiredPosition["Y"] = double.Parse(Node.Attributes["Y"].Value);
                            _Robot.DesiredPosition["Z"] = double.Parse(Node.Attributes["Z"].Value);    
                            break;

                        case "Velocity":
                            //updateVelocity(double.Parse(Node.Attributes["X"].Value), double.Parse(Node.Attributes["Y"].Value), double.Parse(Node.Attributes["Z"].Value));
                            break;
                        case "Gripper":
                            if (int.Parse(Node.InnerText) == 0)
                            {
                                _Robot.GripperIsOpen = false;
                            }
                            else
                            {
                                _Robot.GripperIsOpen = true;
                            }
                            break;
                        default:
                            break;
                    }
                }

                _Robot.newPosition();

                UpdateXML(State);

            }
            catch (SocketException se)
            {
                _Robot.updateError("SocketException " + catchStatement);
                _Robot.updateError(se.Message);
            }
            catch (ObjectDisposedException ob)
            {
                _Robot.updateError("ObjectDisposedException " + catchStatement);
                _Robot.updateError(ob.Message);
            }
            catch (Exception e)
            {
                _Robot.updateError("Generic error " + catchStatement);
                _Robot.updateError(e.Message);
                _Robot.updateError(e.StackTrace);
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
                currentPosition.Attributes["X"].Value = String.Format("{0:0.0000}", _Robot.ReadPosition["X"]);
                currentPosition.Attributes["Y"].Value = String.Format("{0:0.0000}", _Robot.ReadPosition["Y"]);
                currentPosition.Attributes["Z"].Value = String.Format("{0:0.0000}", _Robot.ReadPosition["Z"]);
                currentPosition.Attributes["A"].Value = String.Format("{0:0.0000}", _Robot.ReadPosition["A"]);
                currentPosition.Attributes["B"].Value = String.Format("{0:0.0000}", _Robot.ReadPosition["B"]);
                currentPosition.Attributes["C"].Value = String.Format("{0:0.0000}", _Robot.ReadPosition["C"]);
            }

            XmlNode currentVelocity = _SendXML.SelectSingleNode("//Robot/Velocity");
            if (currentVelocity != null)
            {
                currentVelocity.Attributes["X"].Value = String.Format("{0:0.0000}", _Robot.Velocity["X"]);
                currentVelocity.Attributes["Y"].Value = String.Format("{0:0.0000}", _Robot.Velocity["Y"]);
                currentVelocity.Attributes["Z"].Value = String.Format("{0:0.0000}", _Robot.Velocity["Z"]);
                currentVelocity.Attributes["A"].Value = String.Format("{0:0.0000}", _Robot.Velocity["A"]);
                currentVelocity.Attributes["B"].Value = String.Format("{0:0.0000}", _Robot.Velocity["B"]);
                currentVelocity.Attributes["C"].Value = String.Format("{0:0.0000}", _Robot.Velocity["C"]);
            }

            XmlNode currentAcceleration = _SendXML.SelectSingleNode("//Robot/Acceleration");
            if (currentAcceleration != null)
            {
                currentAcceleration.Attributes["X"].Value = String.Format("{0:0.0000}", _Robot.acceleration["X"]);
                currentAcceleration.Attributes["Y"].Value = String.Format("{0:0.0000}", _Robot.acceleration["Y"]);
                currentAcceleration.Attributes["Z"].Value = String.Format("{0:0.0000}", _Robot.acceleration["Z"]);
                currentAcceleration.Attributes["A"].Value = String.Format("{0:0.0000}", _Robot.acceleration["A"]);
                currentAcceleration.Attributes["B"].Value = String.Format("{0:0.0000}", _Robot.acceleration["B"]);
                currentAcceleration.Attributes["C"].Value = String.Format("{0:0.0000}", _Robot.acceleration["C"]);
            }

            state.XMLout = (XmlDocument)_SendXML.Clone();

            state.PacketOut = Encoding.UTF8.GetBytes(Beautify(state.XMLout));

            state.hasLoadedMessageOut = true;
        }
        #endregion

    }
}
