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

            RobotInfo ConnectedKuka = new RobotInfo();

            KukaServer KukaServer = new KukaServer(6008, ConnectedKuka);
            Thread KukaServerThread = new Thread(KukaServer.ConstantReceive);

            ExternalServer externalServer = new ExternalServer(5008, ConnectedKuka);
            Thread externalServerThread = new Thread(externalServer.ConstantReceive);

            Thread errorWriter = new Thread(ConnectedKuka.writeMsgs);

            // Start the server listening on its own thread
            KukaServerThread.Start();
            externalServerThread.Start();
            errorWriter.Start();

            Console.WriteLine("press O to open, P to close");

            while (true)
            {
                    switch (System.Console.ReadKey(true).Key)
                    {
                        case ConsoleKey.P:
                            ConnectedKuka.GripperIsOpen = false;
                            break;
                        case ConsoleKey.O:
                            ConnectedKuka.GripperIsOpen = true;
                            break;
                        case ConsoleKey.OemPlus:
                            ConnectedKuka.CurrentTrajectory.MaxSpeed += 0.1;
                            break;
                        case ConsoleKey.OemMinus:
                            ConnectedKuka.CurrentTrajectory.MaxSpeed -= 0.1;
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
