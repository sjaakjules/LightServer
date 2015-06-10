using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
//using Microsoft.Xna.Framework;
//using Microsoft.Xna.Framework.Audio;
//using Microsoft.Xna.Framework.Content;
//using Microsoft.Xna.Framework.GamerServices;
//using Microsoft.Xna.Framework.Graphics;
//using Microsoft.Xna.Framework.Input;
//using Microsoft.Xna.Framework.Media;
//using Microsoft.Xna.Framework;
using MathNet.Numerics.LinearAlgebra;

namespace LightWeight_Server
{
    class Trajectory
    {

        public double maxSpeed = 0.5;
        double[,] quinticParameters;
        //private readonly Quaternion finalOrientation;

        public double AVE_SPEED = 10; // Average velocity used to calculate trajectories (mm/s)

        TimeSpan trajectoryTime;
        bool isActive = false;
        Stopwatch elapsedTime;
        double[] _finalPosition;
        double[] normalVector, displacemnet;

        RobotInfo _robot;


        /// <summary>
        /// Overloaded constructor for various poses
        /// 
        /// Set up variables, quaternion for slerp and time from cardinal distance
        /// Populate quintinParameters with x y z quintic curves where each entry in the list is the next point
        /// </summary>
        public Trajectory(double finalX, double finalY, double finalZ, RobotInfo robot)
        {
            _finalPosition = new double[] { finalX, finalY, finalZ };
            elapsedTime = new Stopwatch();

            _robot = robot;
            double distance = Math.Sqrt(Math.Pow(finalX - robot.CurrentPosition(0), 2) + Math.Pow(finalY - robot.CurrentPosition(1), 2) + Math.Pow(finalZ - robot.CurrentPosition(2), 2));

            trajectoryTime = new TimeSpan(0, 0, 0, 0, Convert.ToInt32(1000 * (distance / AVE_SPEED)));
            _robot.updateError("Current Distance = " + distance.ToString() + " Time: " + (trajectoryTime.Milliseconds).ToString());
            quinticParameters = new double[6, 3];
            FindQuintic(_finalPosition);

            IsActive = true;
        }


        public void updateSpeed()
        {

            displacemnet = new double[3];
            for (int i = 0; i < 3; i++)
            {
                displacemnet[i] = _finalPosition[i] - _robot.CurrentPosition(i);
            }
            normalVector = new double[3];
            double magnatude = Math.Sqrt(Math.Pow(displacemnet[0], 2) + Math.Pow(displacemnet[1], 2) + Math.Pow(displacemnet[2], 2));
            double sf = 0;
            if (magnatude > 1) sf = (magnatude > 20) ? 1 : magnatude / 20;

            for (int i = 0; i < 3; i++)
            {
                normalVector[i] = maxSpeed * sf * displacemnet[i] / magnatude;

            }
        }


        /// <summary>
        /// Controls when the trajectory begins moving
        /// </summary>
        public Boolean IsActive
        {
            get
            {
                return isActive;

            }
            set
            {
                if (!isActive & value)
                {
                    isActive = value;
                    elapsedTime.Start();
                }
                else if (isActive & !value)
                {
                    isActive = value;
                    elapsedTime.Reset();
                }

            }
        }


        /// <summary>
        /// Gets the Position at a specified time, returns -1 if out of time boundaries
        /// </summary>
        public double RefPos(int Axis)
        {
            if (elapsedTime.ElapsedMilliseconds > trajectoryTime.Milliseconds)
            {
                _robot.updateError("elapse time is exceeded : " + elapsedTime.ElapsedMilliseconds + "ms Of " + trajectoryTime.Milliseconds + "ms");
                return _finalPosition[Axis];
            }
            else
                return quinticParameters[0, Axis] +
                    quinticParameters[1, Axis] * Math.Pow(elapsedTime.ElapsedMilliseconds, 1) +
                    quinticParameters[2, Axis] * Math.Pow(elapsedTime.ElapsedMilliseconds, 2) +
                    quinticParameters[3, Axis] * Math.Pow(elapsedTime.ElapsedMilliseconds, 3) +
                    quinticParameters[4, Axis] * Math.Pow(elapsedTime.ElapsedMilliseconds, 4) +
                    quinticParameters[5, Axis] * Math.Pow(elapsedTime.ElapsedMilliseconds, 5);
        }

        /// <summary>
        /// Gets the velocity at a specified time, returns -1 if out of time boundaries
        /// </summary>
        public double RefVel(int Axis)
        {
            if (elapsedTime.ElapsedMilliseconds > trajectoryTime.Milliseconds)
            {
                return 0;
            }
            else
                return quinticParameters[1, Axis] +
                    2 * quinticParameters[2, Axis] * Math.Pow(elapsedTime.ElapsedMilliseconds, 1) +
                    3 * quinticParameters[3, Axis] * Math.Pow(elapsedTime.ElapsedMilliseconds, 2) +
                    4 * quinticParameters[4, Axis] * Math.Pow(elapsedTime.ElapsedMilliseconds, 3) +
                    5 * quinticParameters[5, Axis] * Math.Pow(elapsedTime.ElapsedMilliseconds, 4);
        }

        /// <summary>
        /// Gets the acceleration at a specified time, returns -1 if out of time boundaries
        /// </summary>
        public double RefAcc(int Axis)
        {
            if (elapsedTime.ElapsedMilliseconds > trajectoryTime.Milliseconds)
            {
                return 0;
            }
            else
                return 2 * quinticParameters[2, Axis] +
                    6 * quinticParameters[3, Axis] * Math.Pow(elapsedTime.ElapsedMilliseconds, 1) +
                    12 * quinticParameters[4, Axis] * Math.Pow(elapsedTime.ElapsedMilliseconds, 2) +
                    20 * quinticParameters[5, Axis] * Math.Pow(elapsedTime.ElapsedMilliseconds, 3);
        }

        /// <summary>
        /// Given a pose it calculates the quintic coefficients conserving the current position and velocity
        /// It assumes zero initial time, zero final velocity and when half the time has elapsed it is hafway  with the average velocity
        /// </summary>
        private void FindQuintic(double[] poses)
        {
            Matrix<double> tempQuinticParam = Matrix<double>.Build.Dense(6, 3);
            for (int i = 0; i < 3; i++)
            {
                tempQuinticParam.SetColumn(i, FindCurve(0, 1.0 * trajectoryTime.Milliseconds, _robot.CurrentPosition(i), poses[i], _robot.CurrentVelocity(i), 0)); // magic numbers are zero start time and zero final velocity

            }
            quinticParameters = tempQuinticParam.ToArray();
        }

        /// <summary>
        /// Finds the coefficients to describe a quintic path using start and final time, position and velocity
        /// It assumes when half the time has elapsed it is hafway and with the average velocity
        /// </summary>
        /// <param name="t0"></param> Start Time seconds
        /// <param name="tf"></param> Final Time seconds
        /// <param name="x0"></param> Start Position
        /// <param name="xf"></param> Final Position
        /// <param name="v0"></param> Start Velocity
        /// <param name="vf"></param> Final Velocity
        /// <returns></returns>
        private Vector<double> FindCurve(double t0, double tf, double x0, double xf, double v0, double vf)
        {
            double tm = (tf - t0) / 2;
            Matrix<Double> A = Matrix<Double>.Build.DenseOfArray(new double[,] {{1,  t0  ,Math.Pow(t0,2),Math.Pow(t0,3)  , Math.Pow(t0,4)  , Math.Pow(t0,5)  },  // start position
                                                                                {0,  1   , 2*t0         ,3*Math.Pow(t0,2), 4*Math.Pow(t0,3), 5*Math.Pow(t0,4)},  // start velocity 
                                                                                {1,  tm  ,Math.Pow(tm,2),Math.Pow(tm,3)  ,  Math.Pow(tm,4) , Math.Pow(tm,5)  },  // mid position 
                                                                                {1,  tf  ,Math.Pow(tf,2),Math.Pow(tf,3)  ,  Math.Pow(tf,4) , Math.Pow(tf,5)  },  // final position
                                                                                {0,  1   , 2*tf         ,3*Math.Pow(tf,2), 4*Math.Pow(tf,3), 5*Math.Pow(tf,4)},  // final velocity
                                                                                {0,  0   ,    2         ,6*Math.Pow(tm,1), 12*Math.Pow(tm,2), 20*Math.Pow(tm,3)}}); // mid acceleration 
            Matrix<Double> Y = Matrix<Double>.Build.DenseOfArray(new double[,] {{x0},                           // start position
                                                                                {v0},                           // start velocity 
                                                                                {x0+(xf-x0)/2},                 // mid position (half way)
                                                                                {xf},                           // final position
                                                                                {vf},                           // final velocity
                                                                                {0}});                   // mid acceleration 
            Matrix<Double> X = A.Solve(Y);
            return X.Column(0);
        }
        /*
        // Bool to trigger that the trajectory is in motion
        bool IsActive;
        // Average and max velocities used for trajectory generation when no time is given
        double _maxVelocity, _averageVelocity;
        // Timers used while trajectory is activated
        Stopwatch _trajectoryTime;
        // 6 Coefficients for trajectory in end effector space [parameter, x y z alpha][points][6 vars]
        double[] _QuinticCurves;
        // list of times to get to each point including origin
        TimeSpan[] _trajectoryPoints;
        // Matrix of position and velocity that match with _trajectoryTime, including origin [parameters, x y z alpha][points]
        double[][] _xi, _vi;
        // Number of parameters, will be 4 in end effector space, x,y,z,alpha
        int _parameters;

        #region Constructors
        public Trajectory(double[] EndPos)
        {
            // TODO: Constructor for endpoint only

        }

        public Trajectory(double X, double Y, double Z)
            : this(new double[] { X, Y, Z, 0, 0, 0 })
        {
            // TODO: Constructor for endpoint only
        }

        #region Constructor methods

        void getXiVi(double[] poses, TimeSpan timeStamps)
        {
            // TODO: write method to populate the _xi and _vi with values
        }

        void populateCurves(TimeSpan timestamps, double[][] _xi, double[][] _vi)
        {
            // TODO: write method to populate quinticCurves with values
        }

        #endregion

        #endregion

        public double[] getCardinalPosition()
        {
            //TODO: write method to get the desired position using stopwatch for timer./
        }

        public bool startTrajectory()
        {
            // TODO: wirte code which triggers the stopwatch 
        }
        */
    }
}
