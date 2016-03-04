using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Xml;
using System.Runtime.InteropServices;

namespace LightWeight_Server
{
    class Program
    {

        public static ScreenWriter GUI = new ScreenWriter();

        static int Main(string[] args)
        {
            //HandlerRoutine hr = new HandlerRoutine(InspectControlType);
            //SetConsoleCtrlHandler(hr, true);

            RobotInfo[] ConnectedRobots = new RobotInfo[GUI._nConnectedRobots];
            KukaServer[] KukaServer = new KukaServer[GUI._nConnectedRobots];
            Thread[] KukaServerThread = new Thread[GUI._nConnectedRobots];
            for (int i = 0; i < GUI._nConnectedRobots; i++)
            {
                ConnectedRobots[i] = new RobotInfo(Guid.NewGuid(), GUI,i);
                KukaServer[i] = new KukaServer(6008, ConnectedRobots[i]);
                KukaServerThread[i] = new Thread(KukaServer[i].ConstantReceive);
            }

            ExternalServer externalServer = new ExternalServer(5005, ConnectedRobots, GUI);
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
            return 0;
        }

        [DllImport("Kernel32")]
        public static extern bool SetConsoleCtrlHandler(HandlerRoutine Handler, bool Add);

        //This delegate type used as handler routine to SetConsoleControlHandler.
        public delegate bool HandlerRoutine(ControlTypes CtrlType);

        //Enumerated control types for handlers
        public enum ControlTypes
        {
            CTRL_C_EVENT = 0,
            CTRL_BREAK_EVENT,
            CTRL_CLOSE_EVENT,
            CTRL_LOGOFF_EVENT = 5,
            CTRL_SHUTDOWN_EVENT
        }

        private static bool InspectControlType(ControlTypes ctrlType)
        {
            GUI.closing = true;
           // Console.ReadLine();
            return true;
        }

    }
}
