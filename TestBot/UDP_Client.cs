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
        ManualResetEventSlim haveReceived = new ManualResetEventSlim(false);

        object speedlock = new object();
        object positionLock = new object();

        int _BufferSize = 1024;
        byte[] _buffer;
        byte[] _PacketOut;
        Socket _UdpSocket;
        IPEndPoint _kukaServerIPEP;
        IPEndPoint _localEP;
        int _Port = 6009;
        XmlDocument _SendXML;

        double[] _kukaPosition, _kukaAngles, _AxisSpeed;
      //  Matrix _kukaPose;

        Stopwatch IPOCTimer = new Stopwatch();
        Stopwatch frameRate = new Stopwatch();
        public Stopwatch _loopTimer = new Stopwatch();

        string[] CardinalKey = new string[] { "X", "Y", "Z", "A", "B", "C" };
        string[] AxisKey = new string[] { "A1", "A2", "A3", "A4", "A5", "A6" };

        bool isConnected = false;
        int loopCount = 0;

        Vector3 _EE = new Vector3(50.3f, -10, 102.6f);


        Random rnd = new Random();

        StringBuilder errorString = new StringBuilder(); 

        #region Constructor:
        /// <summary>
        /// Creates a UDP server with XML read and write via a port with threadSafe shared robot information
        /// </summary>
        /// <param name="port"></param> The port which communication occurs on
        /// <param name="robot"></param> The robot information to be updated and read from
        public UDP_Client()
        {
            IPOCTimer.Start();
            // 540, 0, 915 is kuka end effector base hence 39.5 -18.1 81.7 is tip
            //_kukaPosition = new double[] { 500.5, -18.1, 833.3, -180, 0, -180 };
            _kukaAngles = new double[] { 0, -90, 90, 0, 90, 0 };
            _kukaPosition = forwardKinimatics(_kukaAngles, _EE);
            _AxisSpeed = new double[] { 0, 0, 0, 0, 0, 0 };
      //      _kukaPose = MakeMatrixFromKuka(_kukaPosition);
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
                errorString.AppendLine("SocketException " + catchStatement);
                errorString.AppendLine(se.Message);
            }
            catch (ObjectDisposedException ob)
            {
                errorString.AppendLine("ObjectDisposedException " + catchStatement);
                errorString.AppendLine(ob.Message);
            }
            catch (Exception e)
            {
                errorString.AppendLine("Generic error " + catchStatement);
                errorString.AppendLine(e.Message);
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
                errorString.AppendLine("SocketException " + catchStatement);
                errorString.AppendLine(se.Message);
            }
            catch (ObjectDisposedException ob)
            {
                errorString.AppendLine("ObjectDisposedException " + catchStatement);
                errorString.AppendLine(ob.Message);
            }
            catch (Exception e)
            {
                errorString.AppendLine("Generic error " + catchStatement);
                errorString.AppendLine(e.Message);
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
                errorString.AppendLine("SocketException " + catchStatement);
                errorString.AppendLine(se.Message);
                return new IPEndPoint(IPAddress.Parse("127.0.0.1"), _Port);
            }
            catch (ObjectDisposedException ob)
            {
                errorString.AppendLine("ObjectDisposedException " + catchStatement);
                errorString.AppendLine(ob.Message);
                return new IPEndPoint(IPAddress.Parse("127.0.0.1"), _Port);
            }
            catch (Exception e)
            {
                errorString.AppendLine("Generic error " + catchStatement);
                errorString.AppendLine(e.Message);
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
                    errorString.AppendLine("ObjectDisposedException " + catchStatement);
                    errorString.AppendLine(ob.Message);
                }
                catch (Exception e)
                {
                    errorString.AppendLine("Generic error " + catchStatement);
                    errorString.AppendLine(e.Message);
                }
                // pause This thread until a packet has been returned.
                haveReceived.Wait();
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
                errorString.AppendLine("SocketException " + catchStatement);
                errorString.AppendLine(se.Message);
            }
            catch (ObjectDisposedException ob)
            {
                errorString.AppendLine("ObjectDisposedException " + catchStatement);
                errorString.AppendLine(ob.Message);
            }
            catch (Exception e)
            {
                errorString.AppendLine("Generic error " + catchStatement);
                errorString.AppendLine(e.Message);
            }


        }

        void processData(StateObject State)
        {
            string catchStatement = "while trying to process Data:";
            try
            {
                // Encode msg from state object
                State.MessageIn = Encoding.UTF8.GetString(State.PacketIn, 0, State.PacketInSize);
                Console.Clear();
                Console.WriteLine(State.MessageIn);
                Console.WriteLine(errorString);
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
                        case "AKorr":
                            double[] newAngleCommand = new double[7];
                            for (int i = 0; i < 6; i++)
                            {
                                double result;
                                if (double.TryParse(Node.Attributes[AxisKey[i]].Value, out result))
                                {
                                    newAngleCommand[i] = result;
                                    newAngleCommand[6] += 1;
                                }
                                else
                                {
                                    break;
                                }
                            }
                            if (newAngleCommand[6] == 6)
                            {
                                updateRobotAngle(newAngleCommand);
                            }
                            break;
                        default:
                            break;
                    }
                }
            }
            catch (SocketException se)
            {
                errorString.AppendLine("SocketException " + catchStatement);
                errorString.AppendLine(se.Message);
            }
            catch (ObjectDisposedException ob)
            {
                errorString.AppendLine("ObjectDisposedException " + catchStatement);
                errorString.AppendLine(ob.Message);
            }
            catch (Exception e)
            {
                errorString.AppendLine("Generic error " + catchStatement);
                errorString.AppendLine(e.Message);
                errorString.AppendLine(e.StackTrace);
            }
        }

        public void constantSpeedUpdate()
        {
            frameRate.Start();
            while (true)
            {
                double currentFramerate = frameRate.Elapsed.TotalMilliseconds;
                frameRate.Restart();
                lock (positionLock)
                {
                    lock (speedlock)
                    {
                        for (int i = 0; i < 6; i++)
                        {
                            _kukaAngles[i] = _kukaAngles[i] + _AxisSpeed[i] * currentFramerate;
                        }
                    }
                    _kukaPosition = forwardKinimatics(_kukaAngles, _EE);
                }
                Thread.Sleep(1);
            }
        }

        void updateRobotAngle(double[] newAngle)
        {
            lock (speedlock)
            {

                for (int i = 0; i < 6; i++)
                {
                    _AxisSpeed[i] = newAngle[i] / 4;
                    //_kukaAngles[i] += newAngle[i];
                }
            }
            //_kukaPosition = forwardKinimatics(_kukaAngles, _EE);
           // _kukaPose = MakeMatrixFromKuka(_kukaPosition);
            
        }

        void updateRobotPosition(double[] newCommand)
        {
            // TODO: update robot position when data has come in.
            lock (positionLock)
            {
                //Console.WriteLine("In in the lock in update robot");
                Matrix changeTransform = MakeMatrixFromKuka(newCommand);
           //     _kukaPose = M(changeTransform,_kukaPose);
            //    getKukaAngles(_kukaPose, out _kukaPosition);
                
                /*
                 * 
                 * 
                 * 
                for (int i = 0; i < 6; i++)
                {
                    _kukaPosition[i] += newCommand[i];
                }
                for (int i = 3; i < 6; i++)
                {
                    _kukaPosition[i] = (_kukaPosition[i] > 180) ? _kukaPosition[i] - 360 : _kukaPosition[i];
                    _kukaPosition[i] = (_kukaPosition[i] < -180) ? _kukaPosition[i] + 360 : _kukaPosition[i];
                }
                 * 
                 * 
                 */
            }
        }

        public void ConstantSend()
        {
            //_loopTimer.Start();
            while (true)
            {

                //  isReadyToSend.Reset();
                //  if (_loopTimer.Elapsed.TotalMilliseconds > serverSpeed)
                //    {
                //Console.WriteLine("in the lock in the sender");
                UpdateXML();
                SendData();
                //_loopTimer.Restart();
                //   }
                // isReadyToSend.WaitOne(4);
                Thread.Sleep(4);
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
                errorString.AppendLine("SocketException " + catchStatement);
                errorString.AppendLine(se.Message);
            }
            catch (ObjectDisposedException ob)
            {
                errorString.AppendLine("ObjectDisposedException " + catchStatement);
                errorString.AppendLine(ob.Message);
            }
            catch (Exception e)
            {
                errorString.AppendLine("Generic error " + catchStatement);
                errorString.AppendLine(e.Message);
            }
        }

        void UpdateXML()
        {
            XmlNode IpocNode = _SendXML.SelectSingleNode("//Rob/IPOC");
            IpocNode.InnerText = ((long)IPOCTimer.Elapsed.TotalMilliseconds).ToString();
            lock (positionLock)
            {

                XmlNode comPosNode = _SendXML.SelectSingleNode("//Rob/RIst");
                for (int i = 0; i < 6; i++)
                {
                    comPosNode.Attributes[CardinalKey[i]].Value = String.Format("{0:0.000000}", _kukaPosition[i]);
                }

                XmlNode comAngNode = _SendXML.SelectSingleNode("//Rob/AIPos");
                for (int i = 0; i < 6; i++)
                {
                    comAngNode.Attributes[AxisKey[i]].Value = String.Format("{0:0.000000}", _kukaAngles[i]);
                }
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
                errorString.AppendLine("SocketException " + catchStatement);
                errorString.AppendLine(se.Message);
            }
            catch (ObjectDisposedException ob)
            {
                errorString.AppendLine("ObjectDisposedException " + catchStatement);
                errorString.AppendLine(ob.Message);
            }
            catch (Exception e)
            {
                errorString.AppendLine("Generic error " + catchStatement);
                errorString.AppendLine(e.Message);
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

            XmlNode comAngNode = _SendXML.CreateElement("AIPos");
            attribute = _SendXML.CreateAttribute("A1");
            attribute.Value = _kukaAngles[0].ToString();
            comAngNode.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("A2");
            attribute.Value = _kukaAngles[1].ToString();
            comAngNode.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("A3");
            attribute.Value = _kukaAngles[2].ToString();
            comAngNode.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("A4");
            attribute.Value = _kukaAngles[3].ToString();
            comAngNode.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("A5");
            attribute.Value = _kukaAngles[4].ToString();
            comAngNode.Attributes.Append(attribute);
            attribute = _SendXML.CreateAttribute("A6");
            attribute.Value = _kukaAngles[5].ToString();
            comAngNode.Attributes.Append(attribute);
            rootNode.AppendChild(comAngNode);

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



        public void getKukaAngles(Matrix pose, out double[] kukaOut)
        {
            float A = 0;
            float B = 0;
            float C = 0;
            Matrix rotationMat = Matrix.Transpose(pose);

            B = (float)Math.Atan2(-rotationMat.M31, Math.Sqrt(rotationMat.M32 * rotationMat.M32 + rotationMat.M33 * rotationMat.M33));

            if (Math.Abs(Math.Abs(B) - Math.PI / 2) < 1e-6)
            {
                // Gimbal lock situation! A and C form a line of infinate solutions.
                C = 0;// (float)Math.PI / 5f;
                A = (float)Math.Atan2(Math.Sign(B) * rotationMat.M23, Math.Sign(B) * rotationMat.M13) + Math.Sign(B) * C;
            }
            else
            {
                A = (float)Math.Atan2(rotationMat.M21, rotationMat.M11);
                C = (float)Math.Atan2(rotationMat.M32, rotationMat.M33);
            }
            kukaOut = new double[] {(double)pose.Translation.X,(double)pose.Translation.Y,(double)pose.Translation.Z,MathHelper.ToDegrees(A),MathHelper.ToDegrees(B),MathHelper.ToDegrees(C),};

        }


        Matrix M(Matrix mat1, Matrix mat2)
        {
            return Matrix.Transpose(Matrix.Transpose(mat1) * Matrix.Transpose(mat2));
        }

        Matrix MakeMatrixFromKuka(double[] pose)
        {
            Matrix Rz = Matrix.CreateRotationZ((float)(pose[3] * Math.PI / 180));
            Matrix Ry = Matrix.CreateRotationY((float)(pose[4] * Math.PI / 180));
            Matrix Rx = Matrix.CreateRotationX((float)(pose[5] * Math.PI / 180));
            Matrix poseout = M(M(Rz, Ry), Rx);
            poseout.Translation = new Vector3((float)pose[0], (float)pose[1], (float)pose[2]);
            return poseout;
        }


        /// <summary>
        /// Computers the forwards kinimatics using a known EndEffector displacement aligned with T67. The angles are in DEGREES, EE is in mm
        /// </summary>
        /// <param name="angles"></param>
        /// <param name="EE"></param>
        /// <returns></returns>
        public double[] forwardKinimatics(double[] angles, Vector3 EE)
        {
            return forwardKinimatics(angles[0] * Math.PI / 180, angles[1] * Math.PI / 180, angles[2] * Math.PI / 180, angles[3] * Math.PI / 180, angles[4] * Math.PI / 180, angles[5] * Math.PI / 180, EE);
        }

        /// <summary>
        /// Computers the forwards kinimatics using a known EndEffector displacement aligned with T67. The angles are in radians EE is in mm
        /// </summary>
        /// <param name="a1"></param>
        /// <param name="a2"></param>
        /// <param name="a3"></param>
        /// <param name="a4"></param>
        /// <param name="a5"></param>
        /// <param name="a6"></param>
        /// <param name="EE"></param>
        /// <returns></returns>
        public double[] forwardKinimatics(double a1, double a2, double a3, double a4, double a5, double a6, Vector3 EE)
        {
            double s1 = Math.Sin(a1);
            double c1 = Math.Cos(a1);
            double s2 = Math.Sin(a2);
            double c2 = Math.Cos(a2);
            double s3p = Math.Sin(a3 - Math.PI / 2);
            double c3p = Math.Cos(a3 - Math.PI / 2);
            double s4 = Math.Sin(a4);
            double c4 = Math.Cos(a4);
            double s5 = Math.Sin(a5);
            double c5 = Math.Cos(a5);
            double s6 = Math.Sin(a6);
            double c6 = Math.Cos(a6);

            double a = (c4 * s1 + s4 * (c1 * s2 * s3p - c1 * c2 * c3p));
            double b1 = (s1 * s4 - c4 * (c1 * s2 * s3p - c1 * c2 * c3p));
            double b2 = (c1 * c2 * s3p + c1 * c3p * s2);
            double b = (c5 * b1 - s5 * b2);

            double m11 = -s6 * a - c6 * b;
            double m12 = c6 * a - s6 * b;
            double m13 = -s5 * b1 - c5 * b2;
            double m14 = 25 * c1 + 560 * c1 * c2 - EE.X * (s6 * a + c6 * b) + EE.Y * (c6 * (c4 * s1 + s4 * (c1 * s2 * s3p - c1 * c2 * c3p)) - s6 * b) - (s5 * b1 + c5 * b2) * (EE.Z + 80) - 515 * c1 * c2 * s3p - 515 * c1 * c3p * s2 - 35 * c1 * s2 * s3p + 35 * c1 * c2 * c3p;

            a = (c1 * c4 + s4 * (c2 * c3p * s1 - s1 * s2 * s3p));
            b1 = (c1 * s4 - c4 * (c2 * c3p * s1 - s1 * s2 * s3p));
            b2 = (c2 * s1 * s3p + c3p * s1 * s2);
            b = (c5 * b1 + s5 * b2);
            double m21 = -s6 * a - c6 * b;
            double m22 = c6 * a - s6 * b;
            double m23 = c5 * b2 - s5 * b1;
            double m24 = EE.Y * (c6 * a - s6 * b) - 560 * c2 * s1 - EE.X * (s6 * a + c6 * b) - 25 * s1 - (s5 * b1 - c5 * b2) * (EE.Z + 80) - 35 * c2 * c3p * s1 + 515 * c2 * s1 * s3p + 515 * c3p * s1 * s2 + 35 * s1 * s2 * s3p;

            a = (c2 * s3p + c3p * s2);
            b1 = (c2 * c3p - s2 * s3p);
            b = (s5 * b1 + c4 * c5 * a);
            double m31 = c6 * b - s4 * s6 * a;
            double m32 = s6 * b + c6 * s4 * a;
            double m33 = c4 * s5 * a - c5 * b1;
            double m34 = 515 * s2 * s3p - 515 * c2 * c3p - 35 * c2 * s3p - 35 * c3p * s2 - 560 * s2 - (c5 * b1 - c4 * s5 * a) * (EE.Z + 80) + EE.X * (c6 * b - s4 * s6 * a) + EE.Y * (s6 * b + c6 * s4 * a) + 400;

            double m41 = 0;
            double m42 = 0;
            double m43 = 0;
            double m44 = 1;


            Matrix M = new Matrix((float)m11, (float)m12, (float)m13, (float)m14, (float)m21, (float)m22, (float)m23, (float)m24, (float)m31, (float)m32, (float)m33, (float)m34, (float)m41, (float)m42, (float)m43, (float)m44);
            //Matrix orientationShift = Matrix.CreateFromYawPitchRoll((float)Math.PI / 6, 0, 0);
            //Vector3 zAxis = Vector3.Transform(Vector3.Backward, orientationShift);
          //  Console.WriteLine("{0} : {1} : {2}", zAxis.X, zAxis.Y, zAxis.Z);
          //  M = M * Matrix.Transpose(orientationShift);
            M = Matrix.Transpose(M);
            Vector3 pos = M.Translation;

            double[] kukaValues = new double[6];
            getKukaAngles(M, out kukaValues);
            return kukaValues;
        }
            /*
            double a = (c4 * s1 + s4 * (c1 * s2 * s3p - c1 * c2 * c3p));
            double b1 = (s1 * s4 - c4 * (c1 * s2 * s3p - c1 * c2 * c3p));
            double b2 = (c1 * c2 * s3p + c1 * c3p * s2);
            double b = (c5 * b1 - s5 * b2);
            double m11 = -s6 * a - c6 * b;
            double m12 = c6 * a - s6 * b;
            double m13 = -s5 * b1 - c5 * b2;
            //double m14 = 25 * c1 + 560 * c1 * c2 - 515 * c1 * s2 * s3 - 80 * s1 * s4 * s5 + 515 * c1 * c2 * c3 + 35 * c1 * c2 * s3 + 35 * c1 * c3 * s2 + 80 * c1 * c2 * c3 * c5 - 80 * c1 * c5 * s2 * s3 - 80 * c1 * c2 * c4 * s3 * s5 - 80 * c1 * c3 * c4 * s2 * s5;
            double m14 = 25 * c1 + 560 * c1 * c2 - EE.X * (s6 * (c4 * s1 + s4 * (c1 * s2 * s3p - c1 * c2 * c3p)) + c6 * (c5 * (s1 * s4 - c4 * (c1 * s2 * s3p - c1 * c2 * c3p)) - s5 * (c1 * c2 * s3p + c1 * c3p * s2))) + EE.Y * (c6 * (c4 * s1 + s4 * (c1 * s2 * s3p - c1 * c2 * c3p)) - s6 * (c5 * (s1 * s4 - c4 * (c1 * s2 * s3p - c1 * c2 * c3p)) - s5 * (c1 * c2 * s3p + c1 * c3p * s2))) - (s5 * (s1 * s4 - c4 * (c1 * s2 * s3p - c1 * c2 * c3p)) + c5 * (c1 * c2 * s3p + c1 * c3p * s2)) * (EE.Z + 80) - 515 * c1 * c2 * s3p - 515 * c1 * c3p * s2 - 35 * c1 * s2 * s3p + 35 * c1 * c2 * c3p;

            a = (c1 * c4 + s4 * (c2 * c3p * s1 - s1 * s2 * s3p));
            b1 = (c1 * s4 - c4 * (c2 * c3p * s1 - s1 * s2 * s3p));
            b2 = (c2 * s1 * s3p + c3p * s1 * s2);
            b = (c5 * b1 + s5 * b2);
            double m21 = -s6 * a + c6 * b;
            double m22 = -s6 * b + c6 * a;
            double m23 = -s5 * b1 + c5 * b2;
            //double m24 = 515 * s1 * s2 * s3 - 560 * c2 * s1 - 35 * c2 * s1 * s3 - 35 * c3 * s1 * s2 - 80 * c1 * s4 * s5 - 25 * s1 - 515 * c2 * c3 * s1 - 80 * c2 * c3 * c5 * s1 + 80 * c5 * s1 * s2 * s3 + 80 * c2 * c4 * s1 * s3 * s5 + 80 * c3 * c4 * s1 * s2 * s5;
            double m24 = -EE.X * (s6 * (c1 * c4 + s4 * (c2 * c3p * s1 - s1 * s2 * s3p)) + c6 * (c5 * (c1 * s4 - c4 * (c2 * c3p * s1 - s1 * s2 * s3p)) + s5 * (c2 * s1 * s3p + c3p * s1 * s2))) - 560 * c2 * s1 - 25 * s1 + EE.Y * (c6 * (c1 * c4 + s4 * (c2 * c3p * s1 - s1 * s2 * s3p)) - s6 * (c5 * (c1 * s4 - c4 * (c2 * c3p * s1 - s1 * s2 * s3)) + s5 * (c2 * s1 * s3p + c3 * s1 * s2))) - (s5 * (c1 * s4 - c4 * (c2 * c3p * s1 - s1 * s2 * s3p)) - c5 * (c2 * s1 * s3p + c3p * s1 * s2)) * (EE.Z + 80) - 35 * c2 * c3p * s1 + 515 * c2 * s1 * s3p + 515 * c3p * s1 * s2 + 35 * s1 * s2 * s3p;

            a = (c2 * s3p + c3p * s2);
            b1 = (c2 * c3p - s2 * s3p);
            b = (s5 * b1 + c4 * c5 * a);
            double m31 = -s4 * s6 * a + c6 * b;
            double m32 = s6 * b + c6 * s4 * a;
            double m33 = c4 * s5 * a - c5 * b1;
            //double m34 = 40 * s23 * s45 - 35 * s23 - 560 * s2 - 515 * c23 - 80 * c23 * c5 - 40 * s4m5 * s23 + 400;
            double m34 = 515 * s2 * s3p - 515 * c2 * c3p - 35 * c2 * s3p - 35 * c3p * s2 - 560 * s2 - (c5 * (c2 * c3p - s2 * s3p) - c4 * s5 * (c2 * s3p + c3p * s2)) * (EE.Z + 80) + EE.X * (c6 * (s5 * (c2 * c3p - s2 * s3p) + c4 * c5 * (c2 * s3p + c3p * s2)) - s4 * s6 * (c2 * s3p + c3p * s2)) + EE.Y * (s6 * (s5 * (c2 * c3p - s2 * s3p) + c4 * c5 * (c2 * s3p + c3p * s2)) + c6 * s4 * (c2 * s3p + c3p * s2)) + 400;

            double m41 = 0;
            double m42 = 0;
            double m43 = 0;
            double m44 = 1;
             * 
             * 
             */


        #endregion
    }

    

}
