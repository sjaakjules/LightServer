using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Xml;

namespace LightWeight_Server
{
    class Program
    {
        static int Main(string[] args)
        {
            ScreenWriter GUI = new ScreenWriter();
            RobotInfo[] ConnectedRobots = new RobotInfo[GUI._nConnectedRobots];
            KukaServer[] KukaServer = new KukaServer[GUI._nConnectedRobots];
            Thread[] KukaServerThread = new Thread[GUI._nConnectedRobots];
            for (int i = 0; i < GUI._nConnectedRobots; i++)
            {
                ConnectedRobots[i] = new RobotInfo(Guid.NewGuid(), GUI,i);
                KukaServer[i] = new KukaServer(6008, ConnectedRobots[i]);
                KukaServerThread[i] = new Thread(KukaServer[i].ConstantReceive);
            }

            ExternalServer externalServer = new ExternalServer(5008, ConnectedRobots, GUI);
            Thread externalReceiveThread = new Thread(externalServer.ConstantReceive);
            Thread externalSendThread = new Thread(externalServer.ConstantSendData);

            Thread GUI_Writer = new Thread(GUI.UpdateScreen);

            // Start the server listening on its own thread
            for (int i = 0; i < GUI._nConnectedRobots; i++)
            {
                KukaServerThread[i].Start();
            }
            externalReceiveThread.Start();
            externalSendThread.Start();
            GUI_Writer.Start();

            Console.WriteLine("press O to open, P to close");

            while (true)
            {
                    switch (System.Console.ReadKey(true).Key)
                    {
                        case ConsoleKey.P:
                            ConnectedRobots[0].gripperIsOpen = false;
                            break;
                        case ConsoleKey.O:
                            ConnectedRobots[0].gripperIsOpen = true;
                            break;
                        case ConsoleKey.OemPlus:
                            ConnectedRobots[0].LinearVelocity = ConnectedRobots[0].LinearVelocity + 0.01;
                            break;
                        case ConsoleKey.OemMinus:
                            ConnectedRobots[0].LinearVelocity = ConnectedRobots[0].LinearVelocity - 0.01;
                            break;
                        case ConsoleKey.D0:
                            ConnectedRobots[0].AngularVelocity = ConnectedRobots[0].AngularVelocity + 0.001;
                            break;
                        case ConsoleKey.D9:
                            ConnectedRobots[0].AngularVelocity = ConnectedRobots[0].AngularVelocity - 0.001;
                            break;
                        case ConsoleKey.H:
                            ConnectedRobots[0].goHome();
                            break;

                    }
            }

            /*
            Stopwatch timer = new Stopwatch();
            timer.Start();



            bool expectCom = true;
            bool circleMove = false;
            double speed = 0.1;
            double amplitude = 1.0;
            double WaveLength = 10.0;
            while (expectCom)
            {
                if (!circleMove)
                {
                    switch (System.Console.ReadKey(true).Key)
                    {

                        case ConsoleKey.DownArrow:
                            Console.WriteLine("Key pressed, It's ALIVE!!!");
                            KukaServer.CommandedPosition["Z"] = -1.0 * speed;
                            break;
                        case ConsoleKey.UpArrow:
                            Console.WriteLine("Key pressed, It's ALIVE!!!");
                            KukaServer.CommandedPosition["Z"] = speed;
                            break;
                        case ConsoleKey.LeftArrow:
                            Console.WriteLine("Key pressed, It's ALIVE!!!");
                            KukaServer.CommandedPosition["Y"] = -1.0 * speed;
                            break;
                        case ConsoleKey.RightArrow:
                            Console.WriteLine("Key pressed, It's ALIVE!!!");
                            KukaServer.CommandedPosition["Y"] = speed;
                            break;
                        case ConsoleKey.OemPlus:
                            Console.WriteLine("Speeding up, now {0} mm per cycle", speed);
                            speed = speed + 0.1;
                            break;
                        case ConsoleKey.OemMinus:
                            Console.WriteLine("Speeding up, now {0} mm per cycle", speed);
                            speed = speed - 0.1;
                            break;
                        case ConsoleKey.A:
                            amplitude = amplitude + 0.1;
                            Console.WriteLine("Amplitude increased: {0}", amplitude);
                            break;
                        case ConsoleKey.Z:
                            amplitude = amplitude - 0.1;
                            Console.WriteLine("Amplitude decreased: {0}", amplitude);
                            break;
                        case ConsoleKey.S:
                            WaveLength = WaveLength + 0.1;
                            Console.WriteLine("Amplitude increased: {0}", WaveLength);
                            break;
                        case ConsoleKey.X:
                            WaveLength = WaveLength - 0.1;
                            Console.WriteLine("Amplitude decreased: {0}", WaveLength);
                            break;
                        case ConsoleKey.P:
                            Console.WriteLine("You spin me right round baby right round....");
                            circleMove = true;
                            break;

                        case ConsoleKey.Escape:
                            Console.WriteLine("Get me outta here!!");
                            expectCom = false;
                            break;
                    }
                }


                if (circleMove)
                {
                    KukaServer.CommandedPosition["Z"] = amplitude * Math.Sin(2 * Math.PI * timer.ElapsedMilliseconds / (WaveLength * 1000.0));
                    KukaServer.CommandedPosition["Y"] = -1.0 * amplitude * Math.Sin(2 * Math.PI * timer.ElapsedMilliseconds / (WaveLength * 1000.0));
                    Console.WriteLine("Commanded correction, Z: {0} X {1}", KukaServer.CommandedPosition["Z"], KukaServer.CommandedPosition["X"]);
                }
            }
            */
            return 0;
        }


    }
}
