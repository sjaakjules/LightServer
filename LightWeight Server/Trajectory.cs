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
using MathNet.Numerics.LinearAlgebra;

namespace LightWeight_Server
{
    struct Trajectory
    {
        readonly Pose _startPose, _startVelocity, _finalPose, _finalVelocity, _changePose;
        readonly TimeSpan _trajectoryTime;
        readonly double[][] _QuinticPerameters;
        readonly Vector3 _TrajectoryAxis;
        readonly float _finalAngle;
        readonly Guid _segmentID;

        public Trajectory(Pose EndPose, double averageVelocty, Pose StartPose, Vector3 startVelocity, Vector3 FinalVelocity, Guid segmentID)
        {
            this._QuinticPerameters = new double[4][];
            this._segmentID = segmentID;
            this._startPose = StartPose;
            this._finalPose = EndPose;
            this._startVelocity = new Pose(Quaternion.Identity, startVelocity);
            this._finalVelocity = new Pose(Quaternion.Identity, FinalVelocity);
            Vector3 x0 = StartPose.Translation;
            Vector3 xf = EndPose.Translation;
            Vector3 xm = ((xf - x0) / 2) + x0;
            Vector3 x0d = startVelocity;
            Vector3 xfd = FinalVelocity;
            Vector3 xmd = (float)averageVelocty * Vector3.Normalize(xf - x0);
            this._trajectoryTime = TimeSpan.FromMilliseconds(1.2f * (xf - x0).Length() / (float)averageVelocty);
            this._changePose = new Pose(Quaternion.Inverse(StartPose.Orientation)*EndPose.Orientation, xf-x0);
            SF.getAxisAngle(_changePose.Orientation,out _TrajectoryAxis, out _finalAngle);
            _QuinticPerameters[0] = Quintic(x0.X, xf.X, xm.X, x0d.X, xfd.X, xmd.X, _trajectoryTime.TotalMilliseconds);
            _QuinticPerameters[1] = Quintic(x0.Y, xf.Y, xm.Y, x0d.Y, xfd.Y, xmd.Y, _trajectoryTime.TotalMilliseconds);
            _QuinticPerameters[2] = Quintic(x0.Z, xf.Z, xm.Z, x0d.Z, xfd.Z, xmd.Z, _trajectoryTime.TotalMilliseconds);
            _QuinticPerameters[3] = Quintic(0, _finalAngle, _finalAngle / 2, 0, 0, _trajectoryTime.TotalMilliseconds);
        }

        /// <summary>
        /// Gets a reference pose and will saturate at final time, give the final Pose.
        /// </summary>
        /// <param name="currentPose"></param>
        /// <param name="t"></param>Time during trajectory in ms
        /// <returns></returns>
        public Pose getReferencePosition(double t)
        {
            if (t > _trajectoryTime.TotalMilliseconds)
            {
                return _finalPose;
            }
            Vector3 translation = new Vector3(getPosition(t, _QuinticPerameters[0]), getPosition(t, _QuinticPerameters[1]), getPosition(t, _QuinticPerameters[2]));
            Pose ReferencePose = new Pose(Quaternion.CreateFromAxisAngle(_TrajectoryAxis, getPosition(t, _QuinticPerameters[3])), translation);
            return ReferencePose;
        }

        /// <summary>
        /// Gets a reference pose velocity and will saturate at final time, give the final velocity pose
        /// </summary>
        /// <param name="CurrentVelocity"></param>
        /// <param name="t"></param>
        /// <returns></returns>
        public Pose getReferenceVelocity(double t)
        {
            if (t > _trajectoryTime.TotalMilliseconds)
            {
                return _finalVelocity;
            }
            Vector3 translation = new Vector3(getVelocity(t, _QuinticPerameters[0]), getVelocity(t, _QuinticPerameters[1]), getVelocity(t, _QuinticPerameters[2]));
            Pose ReferencePose = new Pose(Quaternion.CreateFromAxisAngle(_TrajectoryAxis, getVelocity(t, _QuinticPerameters[3])), translation);
            return ReferencePose;
        }

        float getPosition(double t, double[] a)
        {
            t = (t > _trajectoryTime.TotalMilliseconds) ? _trajectoryTime.TotalMilliseconds : t;
            return (float)(a[0] + a[1] * t + a[2] * Math.Pow(t, 2) + a[3] * Math.Pow(t, 3) + a[4] * Math.Pow(t, 4) + a[5] * Math.Pow(t, 5));
        }

        float getVelocity(double t, double[] a)
        {
            t = (t > _trajectoryTime.TotalMilliseconds) ? _trajectoryTime.TotalMilliseconds : t;
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
