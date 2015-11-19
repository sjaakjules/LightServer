using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Audio;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.GamerServices;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using Microsoft.Xna.Framework.Media;
using System.Diagnostics;

namespace LightWeight_Server
{
    enum TrajectoryTypes { Joint, Quintic, Linear, Spline, Cubic };

    abstract class Trajectory
    {
        TrajectoryTypes _type;

        public TrajectoryTypes type { get { return _type; } }
        public TimeSpan trajectoryTime { get; protected set; }
        public Guid segmentID { get; set; }

        public Trajectory(TrajectoryTypes TrajectoryType) 
        {
            _type = TrajectoryType;
        }

    }

    abstract class JointTrajectory : Trajectory
    {
        public double[] startJoint { get; protected set; }
        public double[] finalJoint { get; protected set; }
        public double averageVelocity { get; protected set; }

        public JointTrajectory() : base(TrajectoryTypes.Joint) { }

        abstract public double[] getReferenceVelocity(double t);
        abstract public double[] getReferencePosition(double t);
    }

    abstract class TaskTrajectory : Trajectory
    {

        protected Vector3 _TrajectoryAxis;
        protected float _finalAngle;


        public Pose startPose { get; protected set; }
        public Pose startVelocity { get; protected set; }
        public Pose finalPose { get; protected set; }
        public Pose finalVelocity { get; protected set; }
        public Pose changePose { get; protected set; }
        public double averageVelocity { get; protected set; }

        public TaskTrajectory(TrajectoryTypes types) : base(types) { }

        public abstract Vector3 trajectoryAxis { get;}
        public abstract float finalAngle { get; }

        abstract public Pose getReferenceVelocity(double t);
        abstract public Pose getReferencePosition(double t);
        abstract public void updateStartPosition(Pose StartPose, Pose StartVelocity);
    }

    class JointLinearTrajectory : JointTrajectory
    {
        public double[,] _LinearPerameters;

        /// <summary>
        /// Creates a new joint space trajectory with start and final angles and a maximum velocity. Velocity in [RADIANS/MS]
        /// </summary>
        /// <param name="StartAngle"></param> start angle
        /// <param name="FinalAngle"></param> final angle
        /// <param name="MaxVelocity"></param>Max Velocity in [RADIANS/MS]
        /// <param name="SegmentID"></param> segment id
        public JointLinearTrajectory(double[] StartAngle, double[] FinalAngle, double MaxVelocity, Guid SegmentID) : base()
        {
            segmentID = SegmentID;
            double[] newStart = new double[StartAngle.Length];
            double[] newFinal = new double[FinalAngle.Length];
            double maxChange = 0;
            for (int i = 0; i < StartAngle.Length; i++)
            {
                newStart[i] = StartAngle[i];
                newFinal[i] = FinalAngle[i];
                if (maxChange < Math.Abs(newFinal[i] - newStart[i]))
                {
                    maxChange = Math.Abs(newFinal[i] - newStart[i]);
                }
            }
            startJoint = newStart;
            finalJoint = newFinal;
            averageVelocity = MaxVelocity;
            trajectoryTime = TimeSpan.FromMilliseconds(1.2 * maxChange / MaxVelocity);
            _LinearPerameters = new double[StartAngle.Length, 2];
            for (int i = 0; i < StartAngle.Length; i++)
            {
                _LinearPerameters[i, 0] = StartAngle[i];
                _LinearPerameters[i, 1] = 1.0*(FinalAngle[i]-StartAngle[i])/trajectoryTime.TotalMilliseconds;
            }

        }


        public override double[] getReferenceVelocity(double t)
        {
            return new double[] { 0, 0, 0, 0, 0, 0 };
            //return (t > trajectoryTime.TotalMilliseconds) ? SF.createDouble(0, finalJoint.Length) : SF.getCol(_LinearPerameters, 1);
        }

        public override double[] getReferencePosition(double t)
        {
            return finalJoint;
            /*
            if (t > trajectoryTime.TotalMilliseconds)
            {
                return finalJoint;
            }
            else
            {
                double[] newPos = new double[finalJoint.Length];
                for (int i = 0; i < finalJoint.Length; i++)
                {
                    newPos[i] = _LinearPerameters[i, 0] + _LinearPerameters[i, 1] * t;
                }
                return newPos;
            }
             */
        }
    }

    class TrajectoryLinear : TaskTrajectory
    {
        public double[][] _LinearPerameters;

        public int nSteps;
        public double[][] nAnglePositions;

        public override float finalAngle { get { return _finalAngle; } }
        public override Vector3 trajectoryAxis { get { return _TrajectoryAxis; } }

        public TrajectoryLinear(Pose EndPose, double AverageVelocity, Pose StartPose, Vector3 StartVelocity, Vector3 FinalVelocity, Guid SegmentID, double[] StartAngle, RobotInfo robot)
            : base(TrajectoryTypes.Linear)
        {
            averageVelocity = AverageVelocity;
            segmentID = SegmentID;
            startPose = StartPose;
            finalPose = EndPose;
            startVelocity = new Pose(Quaternion.Identity, StartVelocity);
            finalVelocity = new Pose(Quaternion.Identity, FinalVelocity);
            Vector3 x0 = StartPose.Translation;
            Vector3 xf = EndPose.Translation;
            trajectoryTime = TimeSpan.FromMilliseconds(1.0f * (xf - x0).Length() / (float)AverageVelocity);
            changePose = new Pose(Quaternion.Inverse(StartPose.Orientation) * EndPose.Orientation, xf - x0);
            SF.getAxisAngle(changePose.Orientation, out _TrajectoryAxis, out _finalAngle);
            _TrajectoryAxis = Vector3.Transform(_TrajectoryAxis, StartPose.Orientation);
            nSteps = (int)trajectoryTime.TotalMilliseconds / 40;
            _LinearPerameters = new double[4][];
            _LinearPerameters[0] = new double[] { x0.X, 1.0 * (xf.X - x0.X) / trajectoryTime.TotalMilliseconds };
            _LinearPerameters[1] = new double[] {x0.Y,1.0 * (xf.Y - x0.Y) / trajectoryTime.TotalMilliseconds};
            _LinearPerameters[2] = new double[] {x0.Z, 1.0 * (xf.Z - x0.Z) / trajectoryTime.TotalMilliseconds};
            _LinearPerameters[3] = new double[] {0, _finalAngle};
            nAnglePositions = new double[nSteps][];
        }


        public override Pose getReferenceVelocity(double t)
        {
            if (t > trajectoryTime.TotalMilliseconds)
            {
                return Pose.Zero;
            }
            return new Pose(Quaternion.CreateFromAxisAngle(_TrajectoryAxis, 1.0f*_finalAngle / (float)trajectoryTime.TotalMilliseconds), Vector3.Multiply(Vector3.Normalize(finalPose.Translation - startPose.Translation), (float)averageVelocity));
        }
        public override Pose getReferencePosition(double t)
        {
            return finalPose;
        }

        public override void updateStartPosition(Pose StartPose, Pose StartVelocity)
        {
            startPose = StartPose;
            Vector3 x0 = StartPose.Translation;
            Vector3 xf = finalPose.Translation;
            trajectoryTime = TimeSpan.FromMilliseconds(1.0f * (xf - x0).Length() / (float)averageVelocity);
            changePose = new Pose(Quaternion.Inverse(StartPose.Orientation) * finalPose.Orientation, xf - x0);
            SF.getAxisAngle(changePose.Orientation, out _TrajectoryAxis, out _finalAngle);
            _TrajectoryAxis = Vector3.Transform(_TrajectoryAxis, StartPose.Orientation);
        }
    }

    class TrajectoryCubic : TaskTrajectory
    {
        public double[][] _CubicPerameters;

        public int nSteps;
        public double[][] nAnglePositions;

        public override float finalAngle { get { return _finalAngle; } }
        public override Vector3 trajectoryAxis { get { return _TrajectoryAxis; } }

        public TrajectoryCubic(Pose EndPose, double AverageVelocty, Pose StartPose, Vector3 StartVelocity, Vector3 FinalVelocity, Guid SegmentID,double[] startAngles, RobotInfo robot)
            : base(TrajectoryTypes.Cubic)
        {
            _CubicPerameters = new double[4][];
            averageVelocity = AverageVelocty;
            segmentID = SegmentID;
            startPose = StartPose;
            finalPose = EndPose;
            startVelocity = new Pose(Quaternion.Identity, StartVelocity);
            finalVelocity = new Pose(Quaternion.Identity, FinalVelocity);
            Vector3 x0 = StartPose.Translation;
            Vector3 xf = EndPose.Translation;
            Vector3 x0d = StartVelocity;
            Vector3 xfd = FinalVelocity;
            trajectoryTime = TimeSpan.FromMilliseconds(1.2f * (xf - x0).Length() / (float)AverageVelocty);
            changePose = new Pose(Quaternion.Inverse(StartPose.Orientation)*EndPose.Orientation, xf-x0);
            SF.getAxisAngle(changePose.Orientation, out _TrajectoryAxis, out _finalAngle);
            _TrajectoryAxis = Vector3.Transform(_TrajectoryAxis, StartPose.Orientation);
            _CubicPerameters[0] = cubic(x0.X, xf.X, x0d.X, xfd.X, trajectoryTime.TotalMilliseconds);
            _CubicPerameters[1] = cubic(x0.Y, xf.Y, x0d.Y, xfd.Y, trajectoryTime.TotalMilliseconds);
            _CubicPerameters[2] = cubic(x0.Z, xf.Z, x0d.Z, xfd.Z, trajectoryTime.TotalMilliseconds);
            _CubicPerameters[3] = cubic(0, finalAngle, 0, 0, trajectoryTime.TotalMilliseconds);
        }

        public override Pose getReferencePosition(double t)
        {
            throw new NotImplementedException();
        }

        public override Pose getReferenceVelocity(double t)
        {
            throw new NotImplementedException();
        }

        public override void updateStartPosition(Pose StartPose, Pose StartVelocity)
        {
            throw new NotImplementedException();
        }

        double[] cubic(double x0, double xf, double x0d, double xfd, double tf)
        {
            double a0 = x0;
            double a1 = x0d;
            double a2 = - (3*x0 - 3*xf)/(tf*tf) - (2*x0d + xfd)/tf;
            double a3 = (2 * x0 - 2 * xf + tf * (x0d + xfd)) / (tf * tf * tf);
            return new double[] { a0, a1, a2, a3 };
        }
    }

    class TrajectoryQuintic : TaskTrajectory
    {
        public double[][] _QuinticPerameters;
        RobotInfo _robot;
      //  public int nSteps;
     //   public double[][] nAnglePositions;
     //   public Pose[] nPose;

        public ElbowPosition elb;
        public BasePosition bas;

        public override float finalAngle { get { return _finalAngle; } }
        public override Vector3 trajectoryAxis { get { return _TrajectoryAxis; } }



        public TrajectoryQuintic(Pose EndPose, double AverageVelocty, Pose StartPose, Vector3 StartVelocity, Vector3 FinalVelocity, Guid SegmentID, double[] startAngles, RobotInfo robot)
            : base(TrajectoryTypes.Quintic)
        {
            bool isStationary = false;
            _robot = robot;
            _QuinticPerameters = new double[4][];
            elb = robot._elbow;
            bas = robot._base;
            segmentID = SegmentID;
            startPose = StartPose;
            finalPose = EndPose;
            startVelocity = new Pose(Quaternion.Identity, StartVelocity);
            finalVelocity = new Pose(Quaternion.Identity, FinalVelocity);
            Vector3 x0 = StartPose.Translation;
            Vector3 xf = EndPose.Translation;
            Vector3 xm = Vector3.Zero;
            Vector3 x0d = Vector3.Zero;
            Vector3 xfd = Vector3.Zero;
            Vector3 xmd = Vector3.Zero;
            averageVelocity = (AverageVelocty == 0) ? 1.0 * robot._MaxCartesianChange / 4 : AverageVelocty;
            if (Vector3.Distance(xf, x0) > 1e-3)
            {
                xm = ((xf - x0) / 2) + x0;
                x0d = StartVelocity;
                xfd = FinalVelocity;
                averageVelocity = (AverageVelocty == 0) ? (Vector3.Distance(xf, x0)) / 0.1 : averageVelocity;
                xmd = (float)averageVelocity * Vector3.Normalize(xf - x0);
            }
            else isStationary = true;
            changePose = new Pose(Quaternion.Inverse(StartPose.Orientation)*EndPose.Orientation, xf-x0);
            SF.getAxisAngle(changePose.Orientation, out _TrajectoryAxis, out _finalAngle);
            _TrajectoryAxis = Vector3.Transform(_TrajectoryAxis, StartPose.Orientation);
            TimeSpan LineartrajectoryTime = TimeSpan.FromMilliseconds(1.2f * (xf - x0).Length() / (float)averageVelocity);
            TimeSpan AngularTrajectoryTime = TimeSpan.FromMilliseconds(720.0 * _finalAngle / (Math.PI * robot._MaxAngularChange));
            trajectoryTime = (LineartrajectoryTime.TotalMilliseconds > AngularTrajectoryTime.TotalMilliseconds) ? LineartrajectoryTime : AngularTrajectoryTime;
            _QuinticPerameters[0] = (isStationary) ? new double[] {x0.X, 0, 0, 0, 0, 0} : Quintic(x0.X, xf.X, xm.X, x0d.X, xfd.X, xmd.X, trajectoryTime.TotalMilliseconds);
            _QuinticPerameters[1] = (isStationary) ? new double[] { x0.Y, 0, 0, 0, 0, 0 } : Quintic(x0.Y, xf.Y, xm.Y, x0d.Y, xfd.Y, xmd.Y, trajectoryTime.TotalMilliseconds);
            _QuinticPerameters[2] = (isStationary) ? new double[] { x0.Z, 0, 0, 0, 0, 0 } : Quintic(x0.Z, xf.Z, xm.Z, x0d.Z, xfd.Z, xmd.Z, trajectoryTime.TotalMilliseconds);
            _QuinticPerameters[3] = Quintic(0, finalAngle, finalAngle / 2, 0, 0, trajectoryTime.TotalMilliseconds);

            /*
            nSteps = (int)(trajectoryTime.TotalMilliseconds / 40) +1;
            nAnglePositions = new double[nSteps][];
            nPose = new Pose[nSteps];
            nPose[0] = getReferencePosition(40);
            nAnglePositions[0] = SF.IKSolver(nPose[0], robot.EndEffector, startAngles, ref elb, ref bas);
            for (int i = 1; i < nSteps; i++)
            {
                nPose[i] = getReferencePosition((i + 1) * 40);
                nAnglePositions[i] = SF.IKSolver(nPose[i], robot.EndEffector, nAnglePositions[i - 1], ref elb, ref bas);
            }
             */
        }

        public TrajectoryQuintic(Pose EndPose, Guid SegmentID)
            : base(TrajectoryTypes.Quintic)
        {
            _QuinticPerameters = new double[][] { new double[] { 0, 0, 0, 0, 0, 0 }, new double[] { 0, 0, 0, 0, 0, 0 }, new double[] { 0, 0, 0, 0, 0, 0 }, new double[] { 0, 0, 0, 0, 0, 0 } };
            segmentID = SegmentID;
            startPose = EndPose;
            finalPose = EndPose;
            startVelocity = new Pose(Quaternion.Identity, Vector3.Zero);
            finalVelocity = new Pose(Quaternion.Identity, Vector3.Zero);
            changePose = new Pose(Quaternion.Identity, Vector3.Zero);
            _TrajectoryAxis = Vector3.Up;
            _finalAngle = 0;
            trajectoryTime = TimeSpan.Zero;
        }

        public override void updateStartPosition(Pose StartPose, Pose StartVelocity)
        {
            bool isStationary = false;
            startVelocity = StartVelocity;
            startPose = StartPose;
            Vector3 x0 = StartPose.Translation;
            Vector3 xf = finalPose.Translation;

            Vector3 xm = Vector3.Zero;
            Vector3 x0d = Vector3.Zero;
            Vector3 xfd = Vector3.Zero;
            Vector3 xmd = Vector3.Zero;
            if (Vector3.Distance(xf, x0) > 1e-3)
            {
                xm = ((xf - x0) / 2) + x0;
                x0d = StartVelocity.Translation;
                xfd = finalVelocity.Translation;
                averageVelocity = (averageVelocity == 0) ? (Vector3.Distance(xf, x0)) / 0.1 : averageVelocity;
                xmd = (float)averageVelocity * Vector3.Normalize(xf - x0);
            }
            else isStationary = true;
            changePose = new Pose(Quaternion.Inverse(StartPose.Orientation) * finalPose.Orientation, xf - x0);
            SF.getAxisAngle(changePose.Orientation, out _TrajectoryAxis, out _finalAngle);
            _TrajectoryAxis = Vector3.Transform(_TrajectoryAxis, StartPose.Orientation);
            TimeSpan LineartrajectoryTime = TimeSpan.FromMilliseconds(1.2f * (xf - x0).Length() / (float)averageVelocity);
            TimeSpan AngularTrajectoryTime = TimeSpan.FromMilliseconds(720.0 * _finalAngle /( (_robot==null)? (Math.PI * 0.08) : (Math.PI * _robot._MaxAngularChange)));
            trajectoryTime = (LineartrajectoryTime.TotalMilliseconds > AngularTrajectoryTime.TotalMilliseconds) ? LineartrajectoryTime : AngularTrajectoryTime;
            _QuinticPerameters[0] = (isStationary) ? new double[] { x0.X, 0, 0, 0, 0, 0 } : Quintic(x0.X, xf.X, xm.X, x0d.X, xfd.X, xmd.X, trajectoryTime.TotalMilliseconds);
            _QuinticPerameters[1] = (isStationary) ? new double[] { x0.Y, 0, 0, 0, 0, 0 } : Quintic(x0.Y, xf.Y, xm.Y, x0d.Y, xfd.Y, xmd.Y, trajectoryTime.TotalMilliseconds);
            _QuinticPerameters[2] = (isStationary) ? new double[] { x0.Z, 0, 0, 0, 0, 0 } : Quintic(x0.Z, xf.Z, xm.Z, x0d.Z, xfd.Z, xmd.Z, trajectoryTime.TotalMilliseconds);
            _QuinticPerameters[3] = Quintic(0, finalAngle, finalAngle / 2, 0, 0, trajectoryTime.TotalMilliseconds);
        }

        /// <summary>
        /// Gets a reference pose and will saturate at final time, give the final Pose.
        /// </summary>
        /// <param name="t"></param>Time during trajectory in ms
        /// <returns></returns>
        public override Pose getReferencePosition(double t)
        {
            if (t > trajectoryTime.TotalMilliseconds)
            {
                return finalPose;
            }
            Vector3 translation = new Vector3(getPosition(t, _QuinticPerameters[0]), getPosition(t, _QuinticPerameters[1]), getPosition(t, _QuinticPerameters[2]));
            Pose ReferencePose = new Pose(startPose.Orientation*Quaternion.CreateFromAxisAngle(changePose.axis, getPosition(t, _QuinticPerameters[3])), translation);
            return ReferencePose;
        }

        /// <summary>
        /// Gets a reference pose velocity and will saturate at final time, give the final velocity pose
        /// </summary>
        /// <param name="t"></param>
        /// <returns></returns>
        public override Pose getReferenceVelocity(double t)
        {
            if (t > trajectoryTime.TotalMilliseconds)
            {
                return Pose.Zero;
            }
            Vector3 translation = new Vector3(getVelocity(t, _QuinticPerameters[0]), getVelocity(t, _QuinticPerameters[1]), getVelocity(t, _QuinticPerameters[2]));
            Pose ReferencePose = new Pose(Quaternion.CreateFromAxisAngle(_TrajectoryAxis, getVelocity(t, _QuinticPerameters[3])), translation);
            return ReferencePose;
        }

        float getPosition(double t, double[] a)
        {
            t = (t > trajectoryTime.TotalMilliseconds) ? trajectoryTime.TotalMilliseconds : t;
            return (float)(a[0] + a[1] * t + a[2] * Math.Pow(t, 2) + a[3] * Math.Pow(t, 3) + a[4] * Math.Pow(t, 4) + a[5] * Math.Pow(t, 5));
        }

        float getVelocity(double t, double[] a)
        {
            t = (t > trajectoryTime.TotalMilliseconds) ? trajectoryTime.TotalMilliseconds : t;
            return (float)(a[1] + 2 * a[2] * Math.Pow(t, 1) + 3 * a[3] * Math.Pow(t, 2) + 4 * a[4] * Math.Pow(t, 3) + 5 * a[5] * Math.Pow(t, 4));
        }
        
        /// <summary>
        /// Constructs a quintic with position and velocity specified halfway in the trajectory
        /// </summary>
        /// <param name="x0"></param> Start Position
        /// <param name="xf"></param> Final Position
        /// <param name="xm"></param> Mid Position
        /// <param name="x0d"></param> start Velocity
        /// <param name="xfd"></param> final velocity
        /// <param name="xmd"></param> mid velcoity
        /// <param name="tf"></param> time for trajectory
        /// <returns></returns>
        double[] Quintic(double x0, double xf, double xm, double x0d, double xfd, double xmd, double tf)
        {
            if (tf == 0)
            {
                tf = 1;
            }
            double a0 = x0;
            double a1 = x0d;
            double a2 = (7 * xf - 23 * x0 + 16 * xm) / Math.Pow(tf, 2) - (6 * x0d + xfd + 8 * xmd) / tf;
            double a3 = (13 * x0d + 5 * xfd + 32 * xmd) / Math.Pow(tf, 2) - (34 * xf - 66 * x0 + 32 * xm) / Math.Pow(tf, 3);
            double a4 = (52 * xf - 68 * x0 + 16 * xm) / Math.Pow(tf, 4) - (12 * x0d + 8 * xfd + 40 * xmd) / Math.Pow(tf, 3);
            double a5 = (4 * x0d + 4 * xfd + 16 * xmd) / Math.Pow(tf, 4) + (24 * x0 - 24 * xf) / Math.Pow(tf, 5);
            return new double[] { a0, a1, a2, a3, a4, a5 };
        }

        /// <summary>
        /// Constructs a quintic with position specified halfway in the trajectory where there is zero acceleration
        /// </summary>
        /// <param name="x0"></param>
        /// <param name="xf"></param>
        /// <param name="xm"></param>
        /// <param name="x0d"></param>
        /// <param name="xfd"></param>
        /// <param name="tf"></param>
        /// <returns></returns>
        double[] Quintic(double x0, double xf, double xm, double x0d, double xfd, double tf)
        {
            double a0 = x0;
            double a1 = x0d;
            double a2 = -(16 * x0 + 16 * xf - 32 * xm) / Math.Pow(tf, 2) - (5 * x0d - 5 * xfd) / tf;
            double a3 = (38 * x0 + 58 * xf - 96 * xm) / Math.Pow(tf, 3) + (9 * x0d - 19 * xfd) / Math.Pow(tf, 2);
            double a4 = -(33 * x0 + 63 * xf - 96 * xm) / Math.Pow(tf, 4) - (7 * x0d - 22 * xfd) / Math.Pow(tf, 3);
            double a5 = (10 * x0 + 22 * xf - 32 * xm) / Math.Pow(tf, 5) + (2 * x0d - 8 * xfd) / Math.Pow(tf, 4);
            return new double[] { a0, a1, a2, a3, a4, a5 };
        }
    }


}
