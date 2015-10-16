﻿using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace LightWeight_Server
{

    class TrajectoryHandler
    {
        object BufferLock = new object();

        Stopwatch _TrajectoryTime;
        bool _isActive, _BufferLoaded;
        int _nSegments, _CurrentSegment;
        Trajectory[] _ActiveTrajectories;
        Trajectory[] _bufferTrajectories;
        Controller TrajectoryController;

        public bool IsActive { get { return _isActive; } }

        RobotInfo _ThisRobot;

        bool BufferLoaded
        {
            get
            {
                lock (BufferLock)
                {
                    return _BufferLoaded;
                }
            }
            set
            {
                lock (BufferLock)
                {
                    _BufferLoaded = value;
                }
            }
        }

        // Constrctor
        public TrajectoryHandler(RobotInfo robot)
        {
            _ThisRobot = robot;
            _TrajectoryTime = new Stopwatch();
            TrajectoryController = new Controller(_ThisRobot);
            _ActiveTrajectories = new Trajectory[] { new TrajectoryQuintic(Pose.Zero, Guid.NewGuid()) };
            _nSegments = 0;
            _CurrentSegment = 0;
            _isActive = false;
        }

        public TrajectoryHandler(Pose currentPose)
        {
            _TrajectoryTime = new Stopwatch();
            TrajectoryController = new Controller(_ThisRobot);
            _ActiveTrajectories = new Trajectory[] { new TrajectoryQuintic(currentPose, Guid.NewGuid()) };
            _nSegments = 0;
            _CurrentSegment = 0;
            _isActive = false;
        }

        public void Start(Pose startPose, Pose startVelocity)
        {
            try
            {
                
            if (!(startPose == _ActiveTrajectories[0].startPose))
            {
                _ActiveTrajectories[0].updateStartPosition(startPose, startVelocity);
            }
            _isActive = true;
            _TrajectoryTime.Restart();
            }
            catch (Exception e )
            {
                _ThisRobot.updateError("error in start" ,e);
            }
        }

        public void ReStart(Pose CurrentPose, Pose CurrentVelocity)
        {
            if (!(CurrentPose.Equals(_ActiveTrajectories[_CurrentSegment].startPose, 1e-4)))
            {
                _ActiveTrajectories[_CurrentSegment].updateStartPosition(CurrentPose, CurrentVelocity);
            }
            _isActive = true;
            _TrajectoryTime.Restart();
        }

        public void Stop(Pose StoppedPose)
        {
            _isActive = false;
            _TrajectoryTime.Reset();
            // Async call to populate trajectories when robot is stationary
            _ActiveTrajectories[_CurrentSegment].updateStartPosition(StoppedPose, Pose.Zero);
        }

        public void Reset()
        {
            _isActive = false;
            _nSegments = 0;
            _CurrentSegment = 0;
            _TrajectoryTime.Reset();

        }

        public void LodeBuffer(Pose currentPose, Pose CurrentVelocity)
        {
            // Check and load from buffer
            lock (BufferLock)
            {
                if (BufferLoaded)
                {
                    _nSegments = _bufferTrajectories.Length;
                    _CurrentSegment = 0;
                    if (!(currentPose == _bufferTrajectories[0].startPose))
                    {
                        _bufferTrajectories[0].updateStartPosition(currentPose, CurrentVelocity);
                    }
                    _ActiveTrajectories = new Trajectory[_nSegments];
                    for (int i = 0; i < _nSegments; i++)
                    {
                        _ActiveTrajectories[i] = _bufferTrajectories[i];
                    }
                    _bufferTrajectories = null;
                    BufferLoaded = false;
                    Start(currentPose, CurrentVelocity);
                }
            }
        }

        public void Load(Trajectory[] NewTrajectoryList)
        {
            try
            {

                _bufferTrajectories = new Trajectory[NewTrajectoryList.Length];
                for (int i = 0; i < NewTrajectoryList.Length; i++)
                {
                    _bufferTrajectories[i] = NewTrajectoryList[i];
                }
                lock (BufferLock)
                {
                    _BufferLoaded = true;
                }
            }
            catch (Exception e)
            {
                _ThisRobot.updateError("error in load, ", e);
            }
        }

        public double[] RobotChange(Pose currentPose, Pose CurrentVelocity, double[,] inverseJacobian)
        {
            // Check if its active
            if (_isActive)
            {
                // Check and load from buffer
                LodeBuffer( currentPose,  CurrentVelocity);
                Pose ReferencePose = _ActiveTrajectories[_CurrentSegment].getReferencePosition(_TrajectoryTime.Elapsed.TotalMilliseconds);
                Pose ReferenceVelocity = _ActiveTrajectories[_CurrentSegment].getReferenceVelocity(_TrajectoryTime.Elapsed.TotalMilliseconds);
                double[] AxisCommand = TrajectoryController.getControllerEffort(ReferencePose, ReferenceVelocity, currentPose, CurrentVelocity, inverseJacobian);
                if (currentPose.Equals(ReferencePose, 1))
                {
                    _CurrentSegment++;
                    if (_CurrentSegment == _nSegments)
                    {
                        Stop(currentPose);
                        Reset();
                    }
                }
                return AxisCommand;
            }
            return new double[] { 0, 0, 0, 0, 0, 0 };


        }

    }
}
