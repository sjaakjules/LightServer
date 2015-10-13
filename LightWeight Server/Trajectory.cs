﻿using System;
using System.Collections.Generic;
using System.Diagnostics;
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
    class Trajectory
    {
        Stopwatch _elapsedTime = new Stopwatch();
        bool _isActive = false, _isRotating = false, _isTranslating = false, _hasCompleated = false;

        Pose _finalPose, _startPose, _startVelocity, _finalVelocity, _changePose;
        Vector3 x0, xf, xm, x0d, xfd, xmd;
        TimeSpan _trajectoryTime;
        Quaternion _startInverse;

        double[][] _QuinticPerameters = new double[4][];
        Vector3 _TrajectoryAxis;
        float _finalAngle;

        int newSegments;
        double[][] _NewQuinticPerameters = new double[4][];
        Vector3 _NewTrajectoryAxis;

        public bool IsActive { get { return _isActive; } }

        // midVelocity might be -1 meaning take last Velocity
        // next velocity might be -1 meaning take this velocity
        // last velocity must be populated with a corrent velocity
        public Trajectory(Pose EndPose, double midVelocity, Pose LastPose, Vector3 startVelocity, Vector3 FinalVelocity)
        {
            this.x0 = LastPose.Translation;
            this.xf = EndPose.Translation;
            this.xm = ((xf - x0) / 2) + x0;
            this.x0d = startVelocity;
            this.xfd = FinalVelocity;
            this.xmd = (float)midVelocity * Vector3.Normalize(xf - x0);
            this._trajectoryTime = TimeSpan.FromMilliseconds(1.2f * (xf - x0).Length() / (float)midVelocity);
            SF.getAxisAngle(Quaternion.Inverse(LastPose.Orientation)*EndPose.Orientation,out _TrajectoryAxis, out _finalAngle);
            _QuinticPerameters[1] = Quintic(x0.X, xf.X, xm.X, x0d.X, xfd.X, xmd.X, _trajectoryTime.TotalMilliseconds);
            _QuinticPerameters[2] = Quintic(x0.Y, xf.Y, xm.Y, x0d.Y, xfd.Y, xmd.Y, _trajectoryTime.TotalMilliseconds);
            _QuinticPerameters[3] = Quintic(x0.Z, xf.Z, xm.Z, x0d.Z, xfd.Z, xmd.Z, _trajectoryTime.TotalMilliseconds);
            _QuinticPerameters[4] = Quintic(0, _finalAngle, _finalAngle / 2, 0, 0, _trajectoryTime.TotalMilliseconds);
            _isActive = false;
            _isTranslating = false;
            _isRotating = false;
        }

        public Trajectory()
        {
            _isActive = false;
            _isTranslating = false;
            _isRotating = false;
            // TODO: Load dummy or initial trajectoy values which move nowhere
        }

        public bool isRotating
        {
            get { return _isRotating; }
        }

        public void hasStopped()
        {
            _elapsedTime.Stop();
            
            // TODO: Add code to pause the trajectory
        }

        public void updateFinalPose(int trigger, Pose finalPose, Pose startPose, Pose startVelocity, Pose finalVelocity)
        {
            Pose trajectoryPose;
            // TODO: update new Quintic and new trajectory axis, does not initialise only updates values, (called from external not kuka)
        }

        public void loadNew()
        {
            // TODO: Stopes and resets trajectory and loads new quintic / axis for trajectory
        }
        public void stop()
        {
            _elapsedTime.Reset();
            _isActive = false;
        }

        public void start()
        {
            _elapsedTime.Reset();
            _isActive = true;
            _elapsedTime.Start();
        }
        public Pose[] reference()
        {
            // TODO: returns an array of reference position and velocity knowing the trajectory time which is updated and paused accordingly
            return new Pose[] { _startPose, _startVelocity };
        }

        /// <summary>
        /// Loads the finalPose with the trigger setting changes in tanslation (1), orientation (-1) or both (0).
        /// A thread lock must be imposed externally as load() and reference() overlap and are accessed in different threads.
        /// The finalPose and startPose must be a translation from base coordinates.
        /// </summary>
        /// <param name="trigger"></param>
        /// <param name="finalPose"></param>
        /// <param name="startPose"></param>
        /// <param name="startVelocity"></param>
        /// <param name="startAcceleration"></param>
        /// <param name="timespan"></param>
        public void load(int trigger, Pose finalPose, Pose startPose, Pose startVelocity, Pose finalVelocity)
        {
            if (trigger >= 0)
            {
                _finalPose.Translation = finalPose.Translation;
                _startPose.Translation = startPose.Translation;
                _changePose.Translation = finalPose.Translation - startPose.Translation;
                _startVelocity.Translation = startVelocity.Translation;
                _isTranslating = true;
            }
            if (trigger <= 0)
            {
                _finalPose.Orientation = finalPose.Orientation;
                _startPose.Orientation = startPose.Orientation;
                _startInverse = Quaternion.Inverse(startPose.Orientation);
                _changePose.Orientation = _startInverse * finalPose.Orientation;
                _startVelocity.Orientation = startVelocity.Orientation;
            //    _trajectoryTime = new TimeSpan(timespan);
            //    _elapsedTime.Restart();
                _isRotating = true;
            }
            _isActive = true;
        }

        public Pose getReferencePosition(Pose currentPose)
        {
            if (_isActive)
            {
                if (_hasCompleated)
                {
                    return _finalPose;
                }
                double t = _elapsedTime.Elapsed.TotalMilliseconds;
                if (t>=_trajectoryTime.TotalMilliseconds)
                {
                    _hasCompleated = true;
                    _elapsedTime.Stop();
                }
                Vector3 translation = new Vector3(getPosition(t, _QuinticPerameters[1]), getPosition(t, _QuinticPerameters[2]), getPosition(t, _QuinticPerameters[3]));
                Pose ReferencePose = new Pose(Quaternion.CreateFromAxisAngle(_TrajectoryAxis, getPosition(t, _QuinticPerameters[4])), translation);

            }
            return currentPose;
        }


        public Pose getReferenceVelocity(Pose CurrentVelocity)
        {
            if (_isActive)
            {
                if (_hasCompleated)
                {
                    return _finalVelocity;
                }
                double t = _elapsedTime.Elapsed.TotalMilliseconds;
                if (t >= _trajectoryTime.TotalMilliseconds)
                {
                    _hasCompleated = true;
                    _elapsedTime.Stop();
                }
                Vector3 translation = new Vector3(getVelocity(t, _QuinticPerameters[1]), getVelocity(t, _QuinticPerameters[2]), getVelocity(t, _QuinticPerameters[3]));
                Pose ReferencePose = new Pose(Quaternion.CreateFromAxisAngle(_TrajectoryAxis, getVelocity(t, _QuinticPerameters[4])), translation);
            }
            return CurrentVelocity;
        }


        public bool checkFinish(Pose currentPose, double error)
        {
            if (Vector3.Distance(currentPose.Translation, _finalPose.Translation) <= error && SF.getOrientationError(Matrix.CreateFromQuaternion(_finalPose.Orientation), Matrix.CreateFromQuaternion(_changePose.Orientation)).Length() <= error)
            {
                _isActive = false;
                _hasCompleated = false;
                _elapsedTime.Stop();
                return true;
            }
            return false;
        }

        float getPosition(double t, double[] a)
        {
            return (float)(a[0] + a[1] * t + a[2] * Math.Pow(t, 2) + a[3] * Math.Pow(t, 3) + a[4] * Math.Pow(t, 4) + a[5] * Math.Pow(t, 5));
        }

        float getVelocity(double t, double[] a)
        {
            return (float)(a[1] + 2 * a[2] * Math.Pow(t, 1) + 3 * a[3] * Math.Pow(t, 2) + 4 * a[4] * Math.Pow(t, 3) + 5 * a[5] * Math.Pow(t, 4));
        }

        /// <summary>
        /// Gets the Reference velocity in TimeCoordinate and handles all termination triggers for tanslation and orientations.
        /// The velocities and acceleration in mm and ms (where once Kuka cycle = 4ms.)
        /// </summary>
        /// <param name="currentPose"></param>
        /// <param name="currentVelocity"></param>
        /// <param name="currentAcceleration"></param>
        /// <param name="linearVelcoty"></param>
        /// <param name="maxAngularVelocity"></param>
        /// <returns></returns>
        public Pose reference(Pose currentPose, Pose currentVelocity, double maxLinearVelocity, float maxAngularVelocity, float linearAcceleration, float angularAcceleration)
        {
            if (!_isTranslating && !_isRotating)
            {
                _isActive = false;
                return new Pose(Quaternion.Identity,new Vector3(0,0,0));
            }
            Vector3 translationComponent = Vector3.Zero;
            Quaternion changeQ = Quaternion.Identity;
            if (_isTranslating && _isActive)
            {
                // d = distance for acceleration phase
                float d = (float)(maxLinearVelocity * maxLinearVelocity / (2 * linearAcceleration));
                if (Vector3.Distance(currentPose.Translation, _finalPose.Translation) > d)
                {
                    // In the acceleration phase if current velocity is less than maxVelecity otherwise in constant velocity region
                    translationComponent = Vector3.Multiply(Vector3.Normalize(_finalPose.Translation - currentPose.Translation),
                        (Math.Abs((float)maxLinearVelocity - currentVelocity.Translation.Length()) < linearAcceleration * 4) ? (float)maxLinearVelocity : currentVelocity.Translation.Length() + Math.Sign((float)maxLinearVelocity - currentVelocity.Translation.Length()) * linearAcceleration * 4);// scale the velocity according to acceleration      //Math.Sign(maxLinearVelocity - currentVelocity.Translation.Length())*currentVelocity.Translation;
                }
                else
                {
                    // In the deceleration phase
                    if (Vector3.Distance(_finalPose.Translation, currentPose.Translation) < linearAcceleration * 4)
                    {
                        translationComponent = _finalPose.Translation - currentPose.Translation;
                        _isTranslating = false;
                    }
                    else
                    {
                        translationComponent = Vector3.Multiply(Vector3.Normalize(_finalPose.Translation - currentPose.Translation),
                            (float)maxLinearVelocity * Math.Abs(Vector3.Distance(_finalPose.Translation, currentPose.Translation) / d));
                    }
                }
            }

            if (_isRotating && _isActive)
            {

                // d = distance for acceleration phase
                float d = (float)(maxAngularVelocity * maxAngularVelocity / (2 * angularAcceleration));
                Quaternion CurrentChange = _startInverse * currentPose.Orientation;
                float angle;
                Vector3 axis;
                SF.getAxisAngle(CurrentChange, out axis, out angle);


                if (angle < _changePose.angle - d)
                {
                    // In the acceleration phase if current velocity is less than maxVelecity otherwise in constant velocity region
                    changeQ = Quaternion.CreateFromAxisAngle(_changePose.axis,
                        (Math.Abs((float)maxAngularVelocity - currentVelocity.angle) < angularAcceleration * 4) ? (float)maxAngularVelocity : currentVelocity.Translation.Length() + Math.Sign((float)maxAngularVelocity - currentVelocity.angle) * angularAcceleration * 4);// scale the velocity according to acceleration      //Math.Sign(maxLinearVelocity - currentVelocity.Translation.Length())*currentVelocity.Translation;
                }
                else
                {
                    // In the deceleration phase
                    Quaternion CurrentChangeEnd = Quaternion.Inverse(currentPose.Orientation) * _finalPose.Orientation;
                    SF.getAxisAngle(CurrentChangeEnd, out axis, out angle);
                    if (angle <= angularAcceleration * 4)
                    {
                        changeQ = CurrentChangeEnd;
                        _isRotating = false;
                    }
                    else
                    {
                        changeQ = Quaternion.CreateFromAxisAngle(axis,
                            (float)maxAngularVelocity * (angle / d));
                    }
                }

            }

            return new Pose(changeQ, new Vector3(translationComponent.X, translationComponent.Y, translationComponent.Z));

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

        /*
         * 
         * 
         * 
         * ***********   This creates a quintic in position to allow smooth acceleration transitions when creating paths.
         *                  It has spikes of lagg of 100ms which trigger a disconnection with Kuka
         *                  
         * 
         * 
        double[,] _quinticParameters = new double[4, 6];
        Stopwatch _elapsedTime = new Stopwatch();
        bool _isActive = false;
        Matrix _finalPose, _startPose;
        TimeSpan _trajectoryTime;
        Vector3 _axis;
        float _finalAngle;

        //double[] normalVector, displacemnet;

        RobotInfo _robot;

        public Trajectory(RobotInfo robot)
        {
            _robot = robot;
        }


        public double ElapsedMilliseconds { get { return 1.0 * _elapsedTime.ElapsedTicks / TimeSpan.TicksPerMillisecond; } }
        /// <summary>
        /// Controls when the trajectory begins moving
        /// </summary>
        public Boolean IsActive
        {
            get
            {
                return _isActive;

            }
            set
            {
                if (!_isActive & value)
                {
                    _elapsedTime.Restart();
                    _isActive = value;
                }
                else if (_isActive & !value)
                {
                    _elapsedTime.Reset();
                    _isActive = value;
                }

            }
        }

        /// <summary>
        /// Overloaded constructor for various poses
        /// 
        /// Set up variables, quaternion for slerp and time from cardinal distance
        /// Populate quintinParameters with x y z quintic curves where each entry in the list is the next point
        /// </summary>
        public Trajectory(double finalX, double finalY, double finalZ, RobotInfo robot)
            : this(robot)
        {
            _finalPose = Matrix.Identity;
            _finalPose.Translation = new Vector3((float)finalX, (float)finalY, (float)finalZ);
        }

        public Trajectory(double[] finalPosition, Vector3[] finalRotation, RobotInfo robot)
            : this(robot)
        {
            _finalPose = new Matrix(finalRotation[0].X, finalRotation[0].Y, finalRotation[0].Z, 0f,
                                    finalRotation[1].X, finalRotation[1].Y, finalRotation[1].Z, 0f,
                                    finalRotation[2].X, finalRotation[2].Y, finalRotation[2].Z, 0f,
                                    (float)finalPosition[0], (float)finalPosition[1], (float)finalPosition[2], 1f);

        }

        public Trajectory(Matrix finalPose, RobotInfo robot)
            : this(robot)
        {
            _finalPose = finalPose;
            _isActive = false;
        }

        public void Start(Matrix startPose)
        {
            if (IsActive)
            {
                _robot.updateError("Trying to start while active...");
            }
            else
            {
                _startPose = startPose;
                Vector3 distance = _finalPose.Translation - _startPose.Translation;
                //double totalDistance = Math.Sqrt(Math.Pow(_finalPose.Translation.X - _robot.CurrentPosition(0), 2) + Math.Pow(_finalPose.Translation.Y - _robot.CurrentPosition(1), 2) + Math.Pow(_finalPose.Translation.Z - _robot.CurrentPosition(2), 2));
                _trajectoryTime = new TimeSpan(Convert.ToInt32(TimeSpan.TicksPerSecond * distance.Length() / _robot.MaxOrientationDisplacement));
                FindQuintic();
                _robot.updateError("Start Position: " + _startPose.Translation.ToString());
                _robot.updateError("Final Position: " + _finalPose.Translation.ToString());
                _robot.updateError("Trajectory Time: " + _trajectoryTime.TotalMilliseconds.ToString());
                for (int i = 0; i < 4; i++)
                {
                    _robot.updateError(i.ToString() + ": " + _quinticParameters[0, i].ToString() + ": " + _quinticParameters[1, i].ToString() + ": " + _quinticParameters[2, i].ToString() + ": " + _quinticParameters[3, i].ToString() + ": " + _quinticParameters[4, i].ToString() + ": " + _quinticParameters[5, i].ToString());
                }
                IsActive = true;
            }
        }

        public void Start(Matrix startPose, Vector3 startAcceleration)
        {
            _startPose = startPose;
            Vector3 distance = _finalPose.Translation - _startPose.Translation;
            //double totalDistance = Math.Sqrt(Math.Pow(_finalPose.Translation.X - _robot.CurrentPosition(0), 2) + Math.Pow(_finalPose.Translation.Y - _robot.CurrentPosition(1), 2) + Math.Pow(_finalPose.Translation.Z - _robot.CurrentPosition(2), 2));
            _trajectoryTime = new TimeSpan(Convert.ToInt32(TimeSpan.TicksPerSecond * distance.Length() / _robot.MaxOrientationDisplacement));
            FindQuintic(startAcceleration);
            _robot.updateError("Start Position: " + _startPose.Translation.ToString());
            _robot.updateError("Final Position: " + _finalPose.Translation.ToString());
            _robot.updateError("Trajectory Time: " + _trajectoryTime.TotalMilliseconds.ToString());
            for (int i = 0; i < 4; i++)
            {
                _robot.updateError(i.ToString() + ": " + _quinticParameters[0, i].ToString() + ": " + _quinticParameters[1, i].ToString() + ": " + _quinticParameters[2, i].ToString() + ": " + _quinticParameters[3, i].ToString() + ": " + _quinticParameters[4, i].ToString() + ": " + _quinticParameters[5, i].ToString());
            }
            IsActive = true;
        }

        public void Stop()
        {
            IsActive = false;
        }

        public bool hasFinished(Matrix currentPose)
        {
            if (1.0 * _elapsedTime.ElapsedTicks / TimeSpan.TicksPerMillisecond > _trajectoryTime.TotalMilliseconds)
            {
                return true;
            } return false;
            /*
            bool isEqual = false;
            Matrix tempDisplacementPose = Matrix.Invert(currentPose) * _finalPose;
            if (Math.Abs(tempDisplacementPose.M11 - 1) < 0.01 && Math.Abs(tempDisplacementPose.M22 - 1) < 0.01 && Math.Abs(tempDisplacementPose.M33 - 1) < 0.01)
            {
                isEqual = true;
            }
            else isEqual = false;
            if (Math.Abs(tempDisplacementPose.Translation.X) < 0.1 && Math.Abs(tempDisplacementPose.Translation.Y) < 0.1 && Math.Abs(tempDisplacementPose.Translation.Z) < 0.1)
            {
                _robot.updateError("Position Finished");
                isEqual = true;
            }
            else isEqual = false;
            return isEqual;
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
        


        public Matrix getReferencePose()
        {
            Matrix referencePose = Matrix.Identity;
            if (1.0 * _elapsedTime.ElapsedTicks / TimeSpan.TicksPerMillisecond > _trajectoryTime.TotalMilliseconds)
            {
                _robot.updateError("Elapse time reached");
                referencePose.M11 = _finalPose.M11;
                referencePose.M12 = _finalPose.M12;
                referencePose.M13 = _finalPose.M13;
                referencePose.M14 = _finalPose.M14;
                referencePose.M21 = _finalPose.M21;
                referencePose.M22 = _finalPose.M22;
                referencePose.M23 = _finalPose.M23;
                referencePose.M24 = _finalPose.M24;
                referencePose.M31 = _finalPose.M31;
                referencePose.M32 = _finalPose.M32;
                referencePose.M33 = _finalPose.M33;
                referencePose.M34 = _finalPose.M34;
                referencePose.M41 = _finalPose.M41;
                referencePose.M42 = _finalPose.M42;
                referencePose.M43 = _finalPose.M43;
                referencePose.M44 = _finalPose.M44;
                return referencePose;
            }
            referencePose = Matrix.CreateFromQuaternion(Quaternion.CreateFromAxisAngle(_axis, (float)RefPosition(3)));
            referencePose.Translation = RefPos();
            return referencePose;
        }

        public Vector3 RefPos()
        {
            return new Vector3((float)RefPosition(0), (float)RefPosition(1), (float)RefPosition(2));
        }

        /// <summary>
        /// Gets the Position at a specified time, returns -1 if out of time boundaries
        /// </summary>
        public double RefPosition(int Axis)
        {
            return _quinticParameters[0, Axis] +
                _quinticParameters[1, Axis] * Math.Pow(1.0 * _elapsedTime.ElapsedTicks / TimeSpan.TicksPerMillisecond, 1) +
                _quinticParameters[2, Axis] * Math.Pow(1.0 * _elapsedTime.ElapsedTicks / TimeSpan.TicksPerMillisecond, 2) +
                _quinticParameters[3, Axis] * Math.Pow(1.0 * _elapsedTime.ElapsedTicks / TimeSpan.TicksPerMillisecond, 3) +
                _quinticParameters[4, Axis] * Math.Pow(1.0 * _elapsedTime.ElapsedTicks / TimeSpan.TicksPerMillisecond, 4) +
                _quinticParameters[5, Axis] * Math.Pow(1.0 * _elapsedTime.ElapsedTicks / TimeSpan.TicksPerMillisecond, 5);
        }

        /// <summary>
        /// Gets the velocity at a specified time, returns -1 if out of time boundaries
        /// </summary>
        public double RefVel(int Axis)
        {
            if (_elapsedTime.ElapsedMilliseconds > _trajectoryTime.TotalMilliseconds)
            {
                return 0;
            }
            else
                return _quinticParameters[1, Axis] +
                    2 * _quinticParameters[2, Axis] * Math.Pow(_elapsedTime.ElapsedMilliseconds, 1) +
                    3 * _quinticParameters[3, Axis] * Math.Pow(_elapsedTime.ElapsedMilliseconds, 2) +
                    4 * _quinticParameters[4, Axis] * Math.Pow(_elapsedTime.ElapsedMilliseconds, 3) +
                    5 * _quinticParameters[5, Axis] * Math.Pow(_elapsedTime.ElapsedMilliseconds, 4);
        }

        /// <summary>
        /// Gets the acceleration at a specified time, returns -1 if out of time boundaries
        /// </summary>
        public double RefAcc(int Axis)
        {
            if (_elapsedTime.ElapsedMilliseconds > _trajectoryTime.TotalMilliseconds)
            {
                return 0;
            }
            else
                return 2 * _quinticParameters[2, Axis] +
                    6 * _quinticParameters[3, Axis] * Math.Pow(_elapsedTime.ElapsedMilliseconds, 1) +
                    12 * _quinticParameters[4, Axis] * Math.Pow(_elapsedTime.ElapsedMilliseconds, 2) +
                    20 * _quinticParameters[5, Axis] * Math.Pow(_elapsedTime.ElapsedMilliseconds, 3);
        }

        Vector3[] getTrajectoryVector(TimeSpan totalTime, TimeSpan startTime, TimeSpan viaTime, TimeSpan finishTime, Matrix startPose, Matrix finalPose, Matrix viaPose)
        {
            Vector3[] vectorOut = new Vector3[3];
            vectorOut[1] = Vector3.Multiply((startPose.Translation + (viaPose.Translation - startPose.Translation)), (float)(totalTime.TotalMilliseconds + viaTime.TotalMilliseconds - startTime.TotalMilliseconds));

            return vectorOut;
        }


        private void FindQuintic()
        {
            Quaternion finalRotation = Quaternion.CreateFromRotationMatrix(_finalPose);
            Quaternion currentRotation = Quaternion.CreateFromRotationMatrix(_startPose);
            Quaternion toRotation = Quaternion.Conjugate(currentRotation) * finalRotation;
            SF.getAxisAngle(toRotation, ref _axis, ref _finalAngle);
            Matrix<double> tempQuinticParam = Matrix<double>.Build.Dense(6, 4);
            // magic numbers are zero start time and zero final velocity
            tempQuinticParam.SetColumn(0, FindCurve(0, _trajectoryTime.TotalMilliseconds, _startPose.Translation.X, _finalPose.Translation.X, _robot.CurrentVelocity(0) / 1000, 0));
            tempQuinticParam.SetColumn(1, FindCurve(0, _trajectoryTime.TotalMilliseconds, _startPose.Translation.Y, _finalPose.Translation.Y, _robot.CurrentVelocity(1) / 1000, 0));
            tempQuinticParam.SetColumn(2, FindCurve(0, _trajectoryTime.TotalMilliseconds, _startPose.Translation.Z, _finalPose.Translation.Z, _robot.CurrentVelocity(2) / 1000, 0));
            tempQuinticParam.SetColumn(3, FindCurve(0, _trajectoryTime.TotalMilliseconds, 0, _finalAngle, 0, 0));
            _quinticParameters = tempQuinticParam.ToArray();
        }

        private void FindQuintic(Vector3 startAcceleration)
        {
            Quaternion finalRotation = Quaternion.CreateFromRotationMatrix(_finalPose);
            Quaternion currentRotation = Quaternion.CreateFromRotationMatrix(_startPose);
            Quaternion toRotation = Quaternion.Conjugate(currentRotation) * finalRotation;
            SF.getAxisAngle(toRotation, ref _axis, ref _finalAngle);
            Matrix<double> tempQuinticParam = Matrix<double>.Build.Dense(6, 4);
            // magic numbers are zero start time and zero final velocity
            tempQuinticParam.SetColumn(0, FindCurve(0, _trajectoryTime.TotalMilliseconds, _startPose.Translation.X, _finalPose.Translation.X, _robot.CurrentVelocity(0) / 1000, 0, startAcceleration.X / 1000000));
            tempQuinticParam.SetColumn(1, FindCurve(0, _trajectoryTime.TotalMilliseconds, _startPose.Translation.Y, _finalPose.Translation.Y, _robot.CurrentVelocity(1) / 1000, 0, startAcceleration.Y / 1000000));
            tempQuinticParam.SetColumn(2, FindCurve(0, _trajectoryTime.TotalMilliseconds, _startPose.Translation.Z, _finalPose.Translation.Z, _robot.CurrentVelocity(2) / 1000, 0, startAcceleration.Z / 1000000));
            tempQuinticParam.SetColumn(3, FindCurve(0, _trajectoryTime.TotalMilliseconds, 0, _finalAngle, 0, 0));
            _quinticParameters = tempQuinticParam.ToArray();
        }

        /// <summary>
        /// Given a pose it calculates the quintic coefficients conserving the current position and velocity
        /// It assumes zero initial time, zero final velocity and when half the time has elapsed it is hafway  with the average velocity
        /// </summary>
        /// <param name="poses"></param>
        private void FindQuintic(double[] poses)
        {
            Matrix<double> tempQuinticParam = Matrix<double>.Build.Dense(6, 3);
            for (int i = 0; i < 3; i++)
            {
                tempQuinticParam.SetColumn(i, FindCurve(0, _trajectoryTime.TotalMilliseconds, _robot.CurrentPosition(i), poses[i], _robot.CurrentVelocity(i), 0)); // magic numbers are zero start time and zero final velocity

            }
            _quinticParameters = tempQuinticParam.ToArray();
        }

        /// <summary>
        /// Finds the coefficients to describe a quintic path using start and final time, position and velocity
        /// It assumes when half the time has elapsed it is hafway and with the average velocity
        /// </summary>
        /// <param name="t0"></param> Start Time milliseconds
        /// <param name="tf"></param> Final Time milliseconds
        /// <param name="x0"></param> Start Position
        /// <param name="xf"></param> Final Position
        /// <param name="v0"></param> Start Velocity mm/ milliseconds
        /// <param name="vf"></param> Final Velocity mm/ milliseconds
        /// <returns></returns>
        private Vector<double> FindCurve(double t0, double tf, double x0, double xf, double v0, double vf)
        {
            tf += 1e-12;
            double tm = (tf - t0) / 2;
            double xm = x0 + 1.0 * (xf - x0) / 2.0;
            _robot.updateError("T0: " + t0.ToString() + "| Tf: " + tf.ToString() + "| Tm: " + tm.ToString() + "| x0: " + x0.ToString() + "| xf: " + xf.ToString() + "| v0: " + v0.ToString() + "| vf: " + vf.ToString());
            Matrix<Double> A = Matrix<Double>.Build.DenseOfArray(new double[,] {{1,  t0  ,Math.Pow(t0,2),Math.Pow(t0,3)  , Math.Pow(t0,4)  , Math.Pow(t0,5)  },  // start position
                                                                                {0,  1   , 2*t0         ,3*Math.Pow(t0,2), 4*Math.Pow(t0,3), 5*Math.Pow(t0,4)},  // start velocity 
                                                                                {1,  tm  ,Math.Pow(tm,2),Math.Pow(tm,3)  ,  Math.Pow(tm,4) , Math.Pow(tm,5)  },  // mid position 
                                                                                {1,  tf  ,Math.Pow(tf,2),Math.Pow(tf,3)  ,  Math.Pow(tf,4) , Math.Pow(tf,5)  },  // final position
                                                                                {0,  1   , 2*tf         ,3*Math.Pow(tf,2), 4*Math.Pow(tf,3), 5*Math.Pow(tf,4)},  // final velocity
                                                                                {0,  0   ,    2         ,6*Math.Pow(tm,1), 12*Math.Pow(tm,2), 20*Math.Pow(tm,3)}}); // mid acceleration 
            Matrix<Double> Y = Matrix<Double>.Build.DenseOfArray(new double[,] {{x0},                           // start position
                                                                                {v0},                           // start velocity 
                                                                                {xm},                 // mid position (half way)
                                                                                {xf},                           // final position
                                                                                {vf},                           // final velocity
                                                                                {0}});                   // mid acceleration 
            Matrix<Double> X = A.Solve(Y);
            double[] solution = X.Column(0).ToArray();
            StringBuilder solutionstring = new StringBuilder();
            for (int i = 0; i < solution.Length; i++)
            {
                solutionstring.Append("[" + i + "] : " + solution[i].ToString());
            }
            _robot.updateError(solutionstring.ToString());
            return X.Column(0);
        }


        private Vector<double> FindCurve(double t0, double tf, double x0, double xf, double v0, double vf, double a0)
        {
            tf += 1e-12;
            double tm = (tf - t0) / 2;
            double xm = x0 + 1.0 * (xf - x0) / 2.0;
            _robot.updateError("T0: " + t0.ToString() + "| Tf: " + tf.ToString() + "| Tm: " + tm.ToString() + "| x0: " + x0.ToString() + "| xf: " + xf.ToString() + "| v0: " + v0.ToString() + "| vf: " + vf.ToString());
            Matrix<Double> A = Matrix<Double>.Build.DenseOfArray(new double[,] {{1,  t0  ,Math.Pow(t0,2),Math.Pow(t0,3)  , Math.Pow(t0,4)  , Math.Pow(t0,5)  },  // start position
                                                                                {0,  1   , 2*t0         ,3*Math.Pow(t0,2), 4*Math.Pow(t0,3), 5*Math.Pow(t0,4)},  // start velocity 
                                                                                {0,  0   ,    2         ,6*Math.Pow(tf,1), 12*Math.Pow(tf,2), 20*Math.Pow(tf,3)},
                                                                              //  {1,  tm  ,Math.Pow(tm,2),Math.Pow(tm,3)  ,  Math.Pow(tm,4) , Math.Pow(tm,5)  },  // mid position 
                                                                                {1,  tf  ,Math.Pow(tf,2),Math.Pow(tf,3)  ,  Math.Pow(tf,4) , Math.Pow(tf,5)  },  // final position
                                                                                {0,  1   , 2*tf         ,3*Math.Pow(tf,2), 4*Math.Pow(tf,3), 5*Math.Pow(tf,4)},  // final velocity
                                                                                {0,  0   ,    2         ,6*Math.Pow(t0,1), 12*Math.Pow(t0,2), 20*Math.Pow(t0,3)}}); // start acceleration 
            Matrix<Double> Y = Matrix<Double>.Build.DenseOfArray(new double[,] {{x0},                           // start position
                                                                                {v0},                           // start velocity 
                                                                                {0},                 // mid position (half way)
                                                                                {xf},                           // final position
                                                                                {vf},                           // final velocity
                                                                                {a0}});                   // start acceleration 
            Matrix<Double> X = A.Solve(Y);
            double[] solution = X.Column(0).ToArray();
            StringBuilder solutionstring = new StringBuilder();
            for (int i = 0; i < solution.Length; i++)
            {
                solutionstring.Append("[" + i + "] : " + solution[i].ToString());
            }
            _robot.updateError(solutionstring.ToString());
            return X.Column(0);
        }
        
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