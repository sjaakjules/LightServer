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

        Stopwatch updateTime, ServerTime;
        // Time of loop in SECONDS
        double loopTime;

        public ConcurrentDictionary<String, double> ReadPosition;
        public ConcurrentDictionary<String, double> LastPosition;
        public ConcurrentDictionary<String, double> Velocity;
        public ConcurrentDictionary<String, double> LastVelocity;
        public ConcurrentDictionary<String, double> acceleration;
        public ConcurrentDictionary<String, double> Torque;
        public ConcurrentDictionary<String, double> DesiredPosition;
        public ConcurrentDictionary<String, double> CommandedPosition;
        // Thread safe lists for updating and storing of robot information.
        // public ConcurrentStack<StateObject> DataHistory;

        public ConcurrentDictionary<String, StringBuilder> text;

        public Trajectory CurrentTrajectory;

        public bool GripperIsOpen;

        private double maxSpeed = 0.5;

        //StateObject latestState;


        public RobotInfo()
        {
            ReadPosition = new ConcurrentDictionary<string, double>();
            LastPosition = new ConcurrentDictionary<string, double>();
            Velocity = new ConcurrentDictionary<string, double>();
            LastVelocity = new ConcurrentDictionary<string, double>();
            acceleration = new ConcurrentDictionary<string, double>();
            Torque = new ConcurrentDictionary<string, double>();
            CommandedPosition = new ConcurrentDictionary<string, double>();
            DesiredPosition = new ConcurrentDictionary<string, double>();

            // initialise external objects.
            // DataHistory = new ConcurrentStack<StateObject>();

            text = new ConcurrentDictionary<string, StringBuilder>();

            text.TryAdd("msg", new StringBuilder());
            text.TryAdd("Error", new StringBuilder());


            Torque.TryAdd("A1", 0);
            Torque.TryAdd("A2", 0);
            Torque.TryAdd("A3", 0);
            Torque.TryAdd("A4", 0);
            Torque.TryAdd("A5", 0);
            Torque.TryAdd("A6", 0);

            setupDictionaries(ReadPosition);
            setupDictionaries(LastPosition);
            setupDictionaries(Velocity);
            setupDictionaries(LastVelocity);
            setupDictionaries(acceleration);
            setupDictionaries(CommandedPosition);
            setupDictionaries(DesiredPosition);

            ReadPosition["X"] = 540.5;
            ReadPosition["Y"] = -18.1;
            ReadPosition["Z"] = 833.3;

            DesiredPosition["X"] = 540.5;
            DesiredPosition["Y"] = -18.1;
            DesiredPosition["Z"] = 833.3;

            text["Error"].Append("---------------------------------\n             Errors:\n");
            GripperIsOpen = true;


            CurrentTrajectory = new Trajectory(new double[] { 540.5, -18.1, 833.3 }, this);

            updateTime = new Stopwatch();
            ServerTime = new Stopwatch();
            loopTime = 0;

            // Start timer
            ServerTime.Start();
            updateTime.Start();
        }

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
                    string tempText = text["Error"].ToString();
                    Console.WriteLine(tempText);
                    tempText = text["msg"].ToString();
                    Console.WriteLine(tempText);
                    System.Threading.Thread.Sleep(100);

                }
                catch (Exception e)
                {
                }

            }
        }

        public void updateMsg()
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


        void setupDictionaries(ConcurrentDictionary<string, double> dic)
        {
            dic.TryAdd("X", 0);
            dic.TryAdd("Y", 0);
            dic.TryAdd("Z", 0);
            dic.TryAdd("A", 0);
            dic.TryAdd("B", 0);
            dic.TryAdd("C", 0);
        }

        public void updateRobotInfo(double x, double y, double z, double a, double b, double c)
        {

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

            updateTime.Stop();
            loopTime = 1.0 * updateTime.ElapsedTicks / TimeSpan.TicksPerSecond;

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

            updateTime.Restart();
        }

        public void newPosition()
        {
            double[] finalPosition = new double[] { DesiredPosition["X"], DesiredPosition["Y"], DesiredPosition["Z"] };
            CurrentTrajectory = new Trajectory(finalPosition, this);
        }


        public void updateComandPosition()
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

        public double[] getKukaDisplacement()
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
            if (Math.Abs(sumDisplacement)<0.05)
            {
                CurrentTrajectory.IsActive = false;
            }
            return displacement;
        }
    }
}
