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

namespace TestBot
{

    // State object for reading client data asynchronously
    public class StateObject
    {
        // Client  socket.
        public Socket Socket = null;
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
        // Received cartesian coordinates.
        public double[] cartPos = new double[6];
        // trigger that the message is loaded into the state holder
        public bool hasLoadedMessageOut;
    }

    class UDP_Client
    {
        // Thread signals to pause until data has been received
        ManualResetEvent haveReceived = new ManualResetEvent(false);
        public ManualResetEvent isReadyToSend = new ManualResetEvent(false);

        object positionLock = new object();

        int _BufferSize = 1024;
        byte[] _buffer;
        byte[] _PacketOut;
        Socket _UdpSocket;
        IPEndPoint _kukaServerIPEP;
        IPEndPoint _localEP;
        int _Port = 6009;
        XmlDocument _SendXML;

        double[] _kukaPosition;

        Stopwatch IPOCTimer = new Stopwatch(); 
        public Stopwatch _loopTimer = new Stopwatch();

        string[] CardinalKey = new string[] { "X", "Y", "Z", "A", "B", "C" };

        bool isConnected = false;

        #region Constructor:
        /// <summary>
        /// Creates a UDP server with XML read and write via a port with threadSafe shared robot information
        /// </summary>
        /// <param name="port"></param> The port which communication occurs on
        /// <param name="robot"></param> The robot information to be updated and read from
        public UDP_Client()
        {
            _kukaPosition = new double[] { 540.5, -18.1, 833.3, 180, 0, 180 };
            _kukaServerIPEP = new IPEndPoint(IPAddress.Parse("127.0.0.1"), 6008);

            SetupXML();

            // Create Socket
            string catchStatement = "while trying to create new socket:";
            try
            {
                _UdpSocket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
            }
            catch (SocketException se)
            {
                Console.WriteLine("SocketException " + catchStatement);
                Console.WriteLine(se.Message);
            }
            catch (ObjectDisposedException ob)
            {
                Console.WriteLine("ObjectDisposedException " + catchStatement);
                Console.WriteLine(ob.Message);
            }
            catch (Exception e)
            {
                Console.WriteLine("Generic error " + catchStatement);
                Console.WriteLine(e.Message);
            }
            // Binds the socket to local IP.
            bindSocket();
            Console.WriteLine("Connected to server: " + _kukaServerIPEP.ToString());
        }
        #endregion


        #region General Server methods
        void Shutdown()
        {

        }

        void bindSocket()
        {
            // Finds local EP to bind socket
            _localEP = getAvailableIpEP();

            // Bind the local EP
            string catchStatement = " while trying to bind local EP:";
            try
            {
                _UdpSocket.Bind((EndPoint)_localEP);
                Console.WriteLine("Kuka robot IP bound: " + _UdpSocket.LocalEndPoint.ToString());
            }
            catch (SocketException se)
            {
                Console.WriteLine("SocketException " + catchStatement);
                Console.WriteLine(se.Message);
            }
            catch (ObjectDisposedException ob)
            {
                Console.WriteLine("ObjectDisposedException " + catchStatement);
                Console.WriteLine(ob.Message);
            }
            catch (Exception e)
            {
                Console.WriteLine("Generic error " + catchStatement);
                Console.WriteLine(e.Message);
            }
        }

        IPEndPoint getAvailableIpEP()
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
                Console.WriteLine("Select IP to bind Kuka robot...");

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
                Console.WriteLine("SocketException " + catchStatement);
                Console.WriteLine(se.Message);
                return new IPEndPoint(IPAddress.Parse("127.0.0.1"), _Port);
            }
            catch (ObjectDisposedException ob)
            {
                Console.WriteLine("ObjectDisposedException " + catchStatement);
                Console.WriteLine(ob.Message);
                return new IPEndPoint(IPAddress.Parse("127.0.0.1"), _Port);
            }
            catch (Exception e)
            {
                Console.WriteLine("Generic error " + catchStatement);
                Console.WriteLine(e.Message);
                return new IPEndPoint(IPAddress.Parse("127.0.0.1"), _Port);
            }
        }

        public void ConstantReceive()
        {
            while (true)
            {
                // resets the event to nonsignaled state.
                haveReceived.Reset();
                if (!isConnected)
                {
                    IPOCTimer.Restart();
                }
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
                    haveReceived.Set();
                }
                catch (ObjectDisposedException ob)
                {
                    Console.WriteLine("ObjectDisposedException " + catchStatement);
                    Console.WriteLine(ob.Message);
                }
                catch (Exception e)
                {
                    Console.WriteLine("Generic error " + catchStatement);
                    Console.WriteLine(e.Message);
                }
                // pause This thread until a packet has been returned.
                haveReceived.WaitOne();
            }

        }

        void FinishReceiveFrom(IAsyncResult ar)
        {
            string catchStatement = "while receive data is active, reading data:";
            try
            {
                isConnected = true;
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
                //Console.WriteLine("Loud and clear");
                haveReceived.Set();
            }
            catch (SocketException se)
            {
                Console.WriteLine("SocketException " + catchStatement);
                Console.WriteLine(se.Message);
            }
            catch (ObjectDisposedException ob)
            {
                Console.WriteLine("ObjectDisposedException " + catchStatement);
                Console.WriteLine(ob.Message);
            }
            catch (Exception e)
            {
                Console.WriteLine("Generic error " + catchStatement);
                Console.WriteLine(e.Message);
            }


        }

        void processData(StateObject State)
        {
            string catchStatement = "while trying to process Data:";
            try
            {
                // Encode msg from state object
                State.MessageIn = Encoding.UTF8.GetString(State.PacketIn, 0, State.PacketInSize);

                // create xml document from state message in.
                XmlDocument xmlIn = new XmlDocument();
                xmlIn.LoadXml(State.MessageIn);

                XmlNodeList parentNode = xmlIn.ChildNodes;
                XmlNodeList KukaInfoNodes = parentNode.Item(0).ChildNodes;
                foreach (XmlNode Node in KukaInfoNodes)
                {
                    switch (Node.Name)
                    {
                        case "RKorr":
                            double[] newCommand = new double[7];
                            for (int i = 0; i < 6; i++)
                            {
                                double result;
                                if (double.TryParse(Node.Attributes[CardinalKey[i]].Value, out result))
                                {
                                    newCommand[i] = result;
                                    newCommand[6] += 1;
                                }
                                else
                                {
                                    break;
                                }
                            }
                            if (newCommand[6] == 6)
                            {
                                updateRobotPosition(newCommand);
                            }
                            break;
                        default:
                            break;
                    }
                }
            }
            catch (SocketException se)
            {
                Console.WriteLine("SocketException " + catchStatement);
                Console.WriteLine(se.Message);
            }
            catch (ObjectDisposedException ob)
            {
                Console.WriteLine("ObjectDisposedException " + catchStatement);
                Console.WriteLine(ob.Message);
            }
            catch (Exception e)
            {
                Console.WriteLine("Generic error " + catchStatement);
                Console.WriteLine(e.Message);
                Console.WriteLine(e.StackTrace);
            }
        }

        void updateRobotPosition(double[] newCommand)
        {
            // TODO: update robot position when data has come in.
            lock (positionLock)
            {
                //Console.WriteLine("In in the lock in update robot");
                for (int i = 0; i < 6; i++)
                {
                    _kukaPosition[i] += newCommand[i];
                }
            }
        }

        public void ConstantSend()
        {
            while (true)
            {
                isReadyToSend.Reset();
                lock (positionLock)
                {
                    //Console.WriteLine("in the lock in the sender");
                    UpdateXML();
                    SendData();
                }
                isReadyToSend.WaitOne();
            }
        }

        void SendData()
        {
            string catchStatement = "while trying to send the data:";
            try
            {
                _UdpSocket.BeginSendTo(_PacketOut, 0, _PacketOut.Length, SocketFlags.None, _kukaServerIPEP, new AsyncCallback(FinishSendTo), _UdpSocket);
            }
            catch (SocketException se)
            {
                Console.WriteLine("SocketException " + catchStatement);
                Console.WriteLine(se.Message);
            }
            catch (ObjectDisposedException ob)
            {
                Console.WriteLine("ObjectDisposedException " + catchStatement);
                Console.WriteLine(ob.Message);
            }
            catch (Exception e)
            {
                Console.WriteLine("Generic error " + catchStatement);
                Console.WriteLine(e.Message);
            }
        }

        void UpdateXML()
        {
            XmlNode IpocNode = _SendXML.SelectSingleNode("//Rob/IPOC");
            IpocNode.InnerText = IPOCTimer.ElapsedTicks.ToString();

            XmlNode comPosNode = _SendXML.SelectSingleNode("//Rob/RIst");
            for (int i = 0; i < 6; i++)
            {
                comPosNode.Attributes[CardinalKey[i]].Value = String.Format("{0:0.0000}", _kukaPosition[i]);
            }
            _PacketOut = Encoding.UTF8.GetBytes(Beautify(_SendXML));

        }

        void FinishSendTo(IAsyncResult ar)
        {
            string catchStatement = "while trying to finsh sendTo:";
            try
            {
                //Console.WriteLine("Take that!");
                Socket state = (Socket)ar.AsyncState;
                int bytesSent = state.EndSendTo(ar);
            }
            catch (SocketException se)
            {
                Console.WriteLine("SocketException " + catchStatement);
                Console.WriteLine(se.Message);
            }
            catch (ObjectDisposedException ob)
            {
                Console.WriteLine("ObjectDisposedException " + catchStatement);
                Console.WriteLine(ob.Message);
            }
            catch (Exception e)
            {
                Console.WriteLine("Generic error " + catchStatement);
                Console.WriteLine(e.Message);
            }
        }

        void SetupXML()
        {
            _SendXML = new XmlDocument();
            XmlNode rootNode = _SendXML.CreateElement("Rob");
            XmlAttribute attribute = _SendXML.CreateAttribute("Type");
            attribute.Value = "KUKA";
            rootNode.Attributes.Append(attribute);
            _SendXML.AppendChild(rootNode);

            XmlNode comPosNode = _SendXML.CreateElement("RIst");
            attribute = _SendXML.CreateAttribute("X");
            attribute.Value = _kukaPosition[0].ToString();
            comPosNode.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("Y");
            attribute.Value = _kukaPosition[1].ToString();
            comPosNode.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("Z");
            attribute.Value = _kukaPosition[2].ToString();
            comPosNode.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("A");
            attribute.Value = _kukaPosition[3].ToString();
            comPosNode.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("B");
            attribute.Value = _kukaPosition[4].ToString();
            comPosNode.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("C");
            attribute.Value = _kukaPosition[5].ToString();
            comPosNode.Attributes.Append(attribute);
            rootNode.AppendChild(comPosNode);

            XmlNode IpocNode = _SendXML.CreateElement("IPOC");
            IpocNode.InnerText = "0";
            rootNode.AppendChild(IpocNode);

        }

        string Beautify(XmlDocument doc)
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


        public void timer()
        {
            while (true)
            {
                if (_loopTimer.ElapsedTicks / TimeSpan.TicksPerMillisecond > 13)
                {
                    _loopTimer.Restart();
                    isReadyToSend.Set();
                }
            }
        }
        #endregion
    }

}
