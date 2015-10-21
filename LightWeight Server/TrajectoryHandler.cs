using System;
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
        object DesiredPoseLock = new object();

        Stopwatch _TrajectoryTime;
        bool _isActive, _BufferLoaded;
        int _nSegments, _CurrentSegment;
        TaskTrajectory[] _ActiveTrajectories;
        TaskTrajectory[] _bufferTrajectories;
        Controller TrajectoryController;
        FixedSizedQueue<Pose> _ReferencePose, _ReferenceVelocity;
        Pose _desiredPose= Pose.Zero;
        

        public bool IsActive { get { return _isActive; } }
        public Pose ReferencePose { get { return _ReferencePose.LastElement; } }
        public Pose ReferenceVelocity { get { return _ReferenceVelocity.LastElement; } }
        public Pose DesiredPose
        {
            get
            {
                lock (DesiredPoseLock)
                {
                    return _desiredPose;
                } 
            }
        }

        RobotInfo _ThisRobot;


        // Constrctor
        public TrajectoryHandler(RobotInfo robot)
        {
            _ThisRobot = robot;
            _ReferencePose = new FixedSizedQueue<Pose>(10);
            _ReferenceVelocity = new FixedSizedQueue<Pose>(10);
            _ReferencePose.Enqueue(Pose.Zero);
            _ReferenceVelocity.Enqueue(Pose.Zero);
            _TrajectoryTime = new Stopwatch();
            TrajectoryController = new Controller(_ThisRobot);
            _ActiveTrajectories = new TaskTrajectory[] { new TrajectoryQuintic(Pose.Zero, Guid.NewGuid()) };
            _nSegments = 0;
            _CurrentSegment = 0;
            _isActive = false;
        }
        public void goHome(Trajectory[] newTrajectory)
        {

        }


        public void Start(Pose startPose, Pose startVelocity)
        {
            try
            {
                _CurrentSegment = 0;
                if (!(startPose == _ActiveTrajectories[0].startPose))
                {
                    _ActiveTrajectories[0].updateStartPosition(startPose, startVelocity);
                }
                _desiredPose = _ActiveTrajectories[_CurrentSegment].finalPose;
                _isActive = true;
                _TrajectoryTime.Restart();
            }
            catch (Exception e)
            {
                _ThisRobot.updateError("error in start", e);
            }
        }

        public void ReStart(Pose CurrentPose, Pose CurrentVelocity)
        {
            if (!(CurrentPose.Equals(_ActiveTrajectories[_CurrentSegment].startPose, 1e-4)))
            {
                _ActiveTrajectories[_CurrentSegment].updateStartPosition(CurrentPose, CurrentVelocity);
            }
            _desiredPose = _ActiveTrajectories[_CurrentSegment].finalPose;
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
                if (_BufferLoaded)
                {
                    _nSegments = _bufferTrajectories.Length;
                    _CurrentSegment = 0;
                    if (!(currentPose == _bufferTrajectories[0].startPose))
                    {
                        _bufferTrajectories[0].updateStartPosition(currentPose, CurrentVelocity);
                    }
                    _ActiveTrajectories = new TaskTrajectory[_nSegments];
                    for (int i = 0; i < _nSegments; i++)
                    {
                        _ActiveTrajectories[i] = _bufferTrajectories[i];
                    }
                    _bufferTrajectories = null;
                    _BufferLoaded = false;
                    Start(currentPose, CurrentVelocity);
                }
            }
        }

        public void Load(TaskTrajectory[] NewTrajectoryList)
        {
            try
            {
                lock (BufferLock)
                {
                    _bufferTrajectories = new TaskTrajectory[NewTrajectoryList.Length];
                    for (int i = 0; i < NewTrajectoryList.Length; i++)
                    {
                        _bufferTrajectories[i] = NewTrajectoryList[i];
                    }
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
                _ReferencePose.Enqueue(ReferencePose);
                _ReferenceVelocity.Enqueue(ReferenceVelocity);


                // IK solver method:
                double[] AxisCommand = TrajectoryController.getControllerErrort(ReferencePose, _ThisRobot.currentAxisAngle, _ThisRobot);


                // Jacobian method
                //double[] AxisCommand = TrajectoryController.getControllerEffort(ReferencePose, ReferenceVelocity, currentPose, CurrentVelocity, inverseJacobian);
                if (currentPose.Equals(ReferencePose, 5e-1))
                {
                    _CurrentSegment++;
                    if (_CurrentSegment == _nSegments)
                    {
                        Stop(currentPose);
                        Reset();
                    }
                    else
                    {
                        _desiredPose = _ActiveTrajectories[_CurrentSegment].finalPose;
                    }
                }
                return AxisCommand;
            }
            return new double[] { 0, 0, 0, 0, 0, 0 };


        }

    }
}
