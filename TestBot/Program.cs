using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace TestBot
{
    class Program
    {
        static UDP_Client robotClient = new UDP_Client();
        static void Main(string[] args)
        {
            Thread robotClientListener = new Thread(robotClient.ConstantReceive);
            Thread robotClientSender = new Thread(robotClient.ConstantSend);
            Thread timerThread = new Thread(robotClient.timer);
            robotClient._loopTimer.Start();
            robotClientListener.Start();
            robotClientSender.Start();
            timerThread.Start();
            Console.ReadLine();
        }

    }
}
