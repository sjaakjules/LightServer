using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace LightWeight_Server
{
    class RobotInfo
    {

        Stopwatch KukaUpdateTime = new Stopwatch();
        // Time of loop in SECONDS
        double loopTime = 0;

        ConcurrentDictionary<String, double> ReadPosition = new ConcurrentDictionary<string, double>();
        ConcurrentDictionary<String, double> LastPosition = new ConcurrentDictionary<string, double>();
        ConcurrentDictionary<String, double> Velocity = new ConcurrentDictionary<string, double>();
        ConcurrentDictionary<String, double> LastVelocity = new ConcurrentDictionary<string, double>();
        ConcurrentDictionary<String, double> acceleration = new ConcurrentDictionary<string, double>();
        ConcurrentDictionary<String, double> Torque = new ConcurrentDictionary<string, double>();
        ConcurrentDictionary<String, double> DesiredPosition = new ConcurrentDictionary<string, double>();
        ConcurrentDictionary<String, double> CommandedPosition = new ConcurrentDictionary<string, double>();
        // Thread safe lists for updating and storing of robot information.
        // public ConcurrentStack<StateObject> DataHistory;

        ConcurrentDictionary<String, StringBuilder> text = new ConcurrentDictionary<string, StringBuilder>();

        Trajectory CurrentTrajectory;

        bool GripperIsOpen = true;

        bool isConnected = false;

        double maxSpeed = 0.5;

        StringBuilder _PrintMsg = new StringBuilder();

        public RobotInfo()
        {
            text.TryAdd("msg", new StringBuilder());
            text.TryAdd("Error", new StringBuilder());

            setupCardinalDictionaries(ReadPosition, 540.5, -18.1, 833.3);
            setupCardinalDictionaries(LastPosition);
            setupCardinalDictionaries(Velocity);
            setupCardinalDictionaries(LastVelocity);
            setupCardinalDictionaries(acceleration);
            setupCardinalDictionaries(CommandedPosition);
            setupCardinalDictionaries(DesiredPosition, 540.5, -18.1, 833.3);

            setupAxisDictionaries(Torque);

            text["Error"].Append("---------------------------------\n             Errors:\n");

        }

        public void Connect()
        {
            if (!isConnected)
            {
                CurrentTrajectory = new Trajectory(new double[] { 540.5, -18.1, 833.3 }, this);

                // Start timer
                KukaUpdateTime.Start();
                isConnected = true;
            }
        }

        #region ScreenDisplay

        public void updateError(string newError)
        {
            text["Error"].AppendLine(newError);
        }

        // Dedicated loop thread
        public void writeMsgs()
        {
            while (true)
            {
                try
                {
                    updateMsg();
                    Console.Clear();
                    _PrintMsg.Clear();
                    text.TryGetValue("Error", out _PrintMsg);
                    Console.WriteLine(_PrintMsg.ToString());
                    text.TryGetValue("msg", out _PrintMsg);
                    Console.WriteLine(_PrintMsg.ToString());
                    System.Threading.Thread.Sleep(100);

                }
                catch (Exception e)
                {
                }

            }
        }

        void updateMsg()
        {
            text["msg"].Clear();
            text["msg"].AppendLine("---------------------------------\n              Info:\n");
            text["msg"].AppendLine("Current Position:     (" + String.Format("{0:0.00}", ReadPosition["X"]) + " , " +
                                                                    String.Format("{0:0.00}", ReadPosition["Y"]) + " , " +
                                                                    String.Format("{0:0.00}", ReadPosition["Z"]) + ")");
            text["msg"].AppendLine("Desired Position:     (" + String.Format("{0:0.00}", DesiredPosition["X"]) + " , " +
                                                                    String.Format("{0:0.00}", DesiredPosition["Y"]) + " , " +
                                                                    String.Format("{0:0.00}", DesiredPosition["Z"]) + ")");
            text["msg"].AppendLine("Command Position:     (" + String.Format("{0:0.00}", CommandedPosition["X"]) + " , " +
                                                                    String.Format("{0:0.00}", CommandedPosition["Y"]) + " , " +
                                                                    String.Format("{0:0.00}", CommandedPosition["Z"]) + ")");

            text["msg"].AppendLine("Max Speed: " + CurrentTrajectory.MaxSpeed.ToString() + "mm per cycle");
            if (GripperIsOpen)
            {
                text["msg"].AppendLine("Gripper is OPEN.");
            }
            else
            {
                text["msg"].AppendLine("Gripper is CLOSED");
            }
            if (CurrentTrajectory.IsActive)
            {
                text["msg"].AppendLine("Trajectory is Active");
            }
            else
            {
                text["msg"].AppendLine("Trajectory is NOT Active");
            }

        }
        #endregion

        #region Movement
        public void updateRobotPosition(double x, double y, double z, double a, double b, double c)
        {

            KukaUpdateTime.Stop();
            loopTime = 1.0 * KukaUpdateTime.ElapsedTicks / TimeSpan.TicksPerSecond;
            KukaUpdateTime.Restart();

            LastPosition["X"] = ReadPosition["X"];
            LastPosition["Y"] = ReadPosition["Y"];
            LastPosition["Z"] = ReadPosition["Z"];
            LastPosition["A"] = ReadPosition["A"];
            LastPosition["B"] = ReadPosition["B"];
            LastPosition["C"] = ReadPosition["C"];

            ReadPosition["X"] = x;
            ReadPosition["Y"] = y;
            ReadPosition["Z"] = z;
            ReadPosition["A"] = a;
            ReadPosition["B"] = b;
            ReadPosition["C"] = c;

            LastVelocity["X"] = Velocity["X"];
            LastVelocity["Y"] = Velocity["Y"];
            LastVelocity["Z"] = Velocity["Z"];
            LastVelocity["A"] = Velocity["A"];
            LastVelocity["B"] = Velocity["B"];
            LastVelocity["C"] = Velocity["C"];

            Velocity["X"] = 1.0 * (ReadPosition["X"] - LastPosition["X"]) / loopTime;
            Velocity["Y"] = 1.0 * (ReadPosition["Y"] - LastPosition["Y"]) / loopTime;
            Velocity["Z"] = 1.0 * (ReadPosition["Z"] - LastPosition["Z"]) / loopTime;
            Velocity["A"] = 1.0 * (ReadPosition["A"] - LastPosition["A"]) / loopTime;
            Velocity["B"] = 1.0 * (ReadPosition["B"] - LastPosition["B"]) / loopTime;
            Velocity["C"] = 1.0 * (ReadPosition["C"] - LastPosition["C"]) / loopTime;

            acceleration["X"] = 1.0 * (Velocity["X"] - LastVelocity["X"]) / loopTime;
            acceleration["Y"] = 1.0 * (Velocity["Y"] - LastVelocity["Y"]) / loopTime;
            acceleration["Z"] = 1.0 * (Velocity["Z"] - LastVelocity["Z"]) / loopTime;
            acceleration["A"] = 1.0 * (Velocity["A"] - LastVelocity["A"]) / loopTime;
            acceleration["B"] = 1.0 * (Velocity["B"] - LastVelocity["B"]) / loopTime;
            acceleration["C"] = 1.0 * (Velocity["C"] - LastVelocity["C"]) / loopTime;

        }

        public void updateRobotTorque(double a1, double a2, double a3, double a4, double a5, double a6)
        {
            Torque["A1"] = a1;
            Torque["A2"] = a2;
            Torque["A3"] = a3;
            Torque["A4"] = a4;
            Torque["A5"] = a5;
            Torque["A6"] = a6;
        }

        void newPosition()
        {
            double[] finalPosition = new double[] { DesiredPosition["X"], DesiredPosition["Y"], DesiredPosition["Z"] };
            CurrentTrajectory = new Trajectory(finalPosition, this);
        }

        public double GetCommandPosition(int Axis)
        {
            if (CurrentTrajectory.IsActive)
            {
                double[] newComandPos = getKukaDisplacement();

                return newComandPos[Axis];

            }
            else
            {
                return 0.0;
            }
        }

        void updateComandPosition()
        {
            if (CurrentTrajectory.IsActive)
            {
                double[] newComandPos = getKukaDisplacement();

                CommandedPosition["X"] = newComandPos[0];
                CommandedPosition["Y"] = newComandPos[1];
                CommandedPosition["Z"] = newComandPos[2];

            }
            else
            {
                CommandedPosition["X"] = 0.0;
                CommandedPosition["Y"] = 0.0;
                CommandedPosition["Z"] = 0.0;
            }
            /*
            try
            {
                goTo.updateSpeed();
                //double[] commandArray = getKukaDisplacement();
                for (int i = 0; i < 3; i++)
                {
                    CommandedPosition[goTo.axisKey[i]] = goTo.normalVector[i];
                }

            }
            catch (Exception)
            {

            }
             * */

        }

        double[] getKukaDisplacement()
        {
            double[] displacement = new double[3];
            double sumDisplacement = 0;
            // For each axis of movement get isplacement of current position and trajectory position
            for (int i = 0; i < 3; i++)
            {
                displacement[i] = CurrentTrajectory.RefPos(i) - ReadPosition[CurrentTrajectory.axisKey[i]];
                if (Math.Abs(displacement[i]) > 0.5)
                {
                    updateError("Error, " + CurrentTrajectory.axisKey[i] + " Axis sent a huge distance, at " + displacement[i].ToString() + "mm");
                    displacement[i] = 0.5 * Math.Sign(displacement[i]);
                }
                sumDisplacement += displacement[i];
            }
            // Are we within 0.05mm of goal?
            if (Math.Abs(sumDisplacement) < 0.05)
            {
                CurrentTrajectory.IsActive = false;
            }
            return displacement;
        }
        #endregion

        #region Setup

        void setupCardinalDictionaries(ConcurrentDictionary<string, double> dic)
        {
            dic.TryAdd("X", 0);
            dic.TryAdd("Y", 0);
            dic.TryAdd("Z", 0);
            dic.TryAdd("A", 0);
            dic.TryAdd("B", 0);
            dic.TryAdd("C", 0);
        }
        void setupCardinalDictionaries(ConcurrentDictionary<string, double> dic, double x, double y, double z)
        {
            dic.TryAdd("X", x);
            dic.TryAdd("Y", y);
            dic.TryAdd("Z", z);
            dic.TryAdd("A", 0);
            dic.TryAdd("B", 0);
            dic.TryAdd("C", 0);
        }

        void setupAxisDictionaries(ConcurrentDictionary<string, double> dic)
        {

            dic.TryAdd("A1", 0);
            dic.TryAdd("A2", 0);
            dic.TryAdd("A3", 0);
            dic.TryAdd("A4", 0);
            dic.TryAdd("A5", 0);
            dic.TryAdd("A6", 0);
        }
        #endregion

    }
}
