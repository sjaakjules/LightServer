using Microsoft.Xna.Framework;
using System;
using System.Collections.Generic;
using System.Collections.Specialized;
using System.ComponentModel;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace LightWeight_Server
{

#region async controller support

    public delegate void updatedCommandCompletedEventHandler(object sender, UpdatedCommandCompleatedEventArgs e);
    public delegate void UpdateCommandProgressChangedEventHandler(UpdatedCommandProgressEventArgs e);

    public class UpdatedCommandProgressEventArgs : ProgressChangedEventArgs
    {
        long _Ipoc;
        double[] _newCommand = new double[6];
        public long Ipoc { get { return _Ipoc; } }
        public double[] newCommand { get { return _newCommand; } }

        public UpdatedCommandProgressEventArgs(long ipoc, double[] newCommand, int percent, object state)
            : base(percent, state)
        {
            this._Ipoc = ipoc;
            newCommand.CopyTo(_newCommand, 0);
        }
    }

    public class UpdatedCommandCompleatedEventArgs : AsyncCompletedEventArgs
    {
        long _Ipoc;
        double[] _newCommand = new double[6];
        public long Ipoc { get { RaiseExceptionIfNecessary(); return _Ipoc; } }
        public double[] newCommand { get { RaiseExceptionIfNecessary(); return _newCommand; } }

        public UpdatedCommandCompleatedEventArgs(long ipoc, double[] newCommand, Exception e, bool canceled, object state)
            : base(e, canceled, state)
        {
            this._Ipoc = ipoc;
            newCommand.CopyTo(_newCommand, 0);
        }
    }
#endregion


    class TrajectoryHandler
    {

        #region async controller support

        public event updatedCommandCompletedEventHandler updateCommandCompleted;
        public event UpdateCommandProgressChangedEventHandler updatedCommandProgressChanged;

        private SendOrPostCallback onProgressReportDelegate;
        private SendOrPostCallback onCompletedDelegate;
        private delegate void WorkerEventhandler(long ipoc, double[] newPosition, double[] newAngles, Pose currentVelocity, AsyncOperation asyncOp);

        protected virtual void InitialiseDelegates()
        {
            onProgressReportDelegate = new SendOrPostCallback(ReportProgress);
            onCompletedDelegate = new SendOrPostCallback(UpdateCommandCompleted);
        }

        private HybridDictionary userStateToLifetime = new HybridDictionary();

        private void UpdateCommandCompleted(object operationState)
        {
            UpdatedCommandCompleatedEventArgs e = operationState as UpdatedCommandCompleatedEventArgs;
            OnUpdateCommandCompleted(e);
        }

        private void ReportProgress(object state)
        {
            UpdatedCommandProgressEventArgs e = state as UpdatedCommandProgressEventArgs;
            OnProgressChanged(e);
        }

        protected void OnUpdateCommandCompleted(UpdatedCommandCompleatedEventArgs e)
        {
            if (updateCommandCompleted != null)
            {
                updateCommandCompleted(this, e);
            }
        }

        protected void OnProgressChanged(UpdatedCommandProgressEventArgs e)
        {
            if (updatedCommandProgressChanged != null)
            {
                updatedCommandProgressChanged(e);
            }
        }

        private void CompleationMethod(long ipoc, double[] newCommand, Exception exception, bool canceled, AsyncOperation asyncOp)
        {
            if (!canceled)
            {
                lock (userStateToLifetime.SyncRoot)
                {
                    userStateToLifetime.Remove(asyncOp.UserSuppliedState);
                }
            }

            UpdatedCommandCompleatedEventArgs e = new UpdatedCommandCompleatedEventArgs(ipoc, newCommand, exception, canceled, asyncOp.UserSuppliedState);

            asyncOp.PostOperationCompleted(onCompletedDelegate, e);
        }

        private bool TaskCanceled(object taskId)
        {
            return (userStateToLifetime[taskId] == null);
        }

        private void updateCommandWorker(long ipocStart, double[] newPosition, double[] newAngles, Pose currentVelocity, AsyncOperation asyncOp)
        {
            double[] newCommand = new double[] { 0, 0, 0, 0, 0, 0 };
            long Ipoc = ipocStart;
            Exception e = null;

            if (!TaskCanceled(asyncOp.UserSuppliedState))
            {
                try
                {
                    newCommand = getNextControllEffort(ipocStart, newPosition, newAngles,currentVelocity, asyncOp);
                }
                catch (Exception ex)
                {
                    e = ex;
                }
            }
            this.CompleationMethod(Ipoc, newCommand, e, TaskCanceled(asyncOp.UserSuppliedState), asyncOp);

        }

        private double[] getNextControllEffort(long ipocStart, double[] newPosition, double[] newAngles, Pose CurrentVelocity, AsyncOperation asyncOp)
        {
            UpdatedCommandProgressEventArgs e = null;
            double[] newCommand = new double[] { 0, 0, 0, 0, 0, 0 };
            Pose newPose = new Pose(newPosition);
            double[,] InverseJacobian = SF.GetInverseJacobian(newAngles, _ThisRobot.EndEffector);
         //   return RobotChange(newPose, CurrentVelocity, InverseJacobian);
            return newCommand;
            /*
            int stepsAhead = 10;
            int n = 0;
            Pose LastsimulatedPose = new Pose(newPosition);
            Pose simulatedPose = new Pose(newPosition);
            Pose simulatedVelocity = CurrentVelocity;
            double[] simulatedAngles = new double[6];
            newAngles.CopyTo(simulatedAngles, 0);

            // Do work:
            while (n < stepsAhead && !TaskCanceled(asyncOp.UserSuppliedState))
            {
                // sync call to get controll effort:
                double[] nCommand = RobotChange(simulatedPose, simulatedVelocity, InverseJacobian);
                e = new UpdatedCommandProgressEventArgs(ipocStart + 4 * n * TimeSpan.TicksPerMillisecond, nCommand, (n + 1) * 10, asyncOp.UserSuppliedState);
                asyncOp.Post(this.onProgressReportDelegate, e);

                SF.addtoDoubles(simulatedAngles, nCommand);
                InverseJacobian = SF.GetInverseJacobian(simulatedAngles, _ThisRobot.EndEffector);
                LastsimulatedPose = simulatedPose;
                simulatedPose = SF.forwardKinimatics(simulatedAngles,_ThisRobot.EndEffector);
                simulatedVelocity = new Pose(LastsimulatedPose, simulatedPose, 4);
                n++;
                Thread.Sleep(0);
            }
             */
        }

        public virtual void getRobotChangeAsync(long ipocStart, double[] newPosition, double[] newAngle, Pose currentVelocity, object taskId)
        {
            AsyncOperation asyncOp = AsyncOperationManager.CreateOperation(taskId);
            lock (userStateToLifetime.SyncRoot)
            {
                if (userStateToLifetime.Contains(taskId))
                {
                    throw new ArgumentException("Task ID parameter must be unique", "taskId");
                }
                userStateToLifetime[taskId] = asyncOp;
            }
            WorkerEventhandler workerDelegate = new WorkerEventhandler(updateCommandWorker);
            workerDelegate.BeginInvoke(ipocStart, newPosition, newAngle, currentVelocity, asyncOp, null, null);
        }

        public void CancelAsync(object taskId)
        {
            AsyncOperation asuncOp = userStateToLifetime[taskId] as AsyncOperation;
            if (asuncOp != null)
            {
                lock (userStateToLifetime.SyncRoot)
                {
                    userStateToLifetime.Remove(taskId);
                }
            }
        }

        #endregion

        object BufferLock = new object();
        object DesiredPoseLock = new object();


        Stopwatch _TrajectoryTime;
        bool _isActive, _BufferLoaded, _timerHasElapsed;
        int _nSegments, _CurrentSegment;
        Trajectory[] _ActiveTrajectories;
        Trajectory[] _bufferTrajectories;
        Controller TrajectoryController;
        FixedSizedQueue<Pose> _ReferencePose, _ReferenceVelocity;
        Pose _desiredPose = Pose.Zero;
        Pose _lastdesiredPose = Pose.Zero;
        

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

        public Pose LastDesiredPose
        {
            get
            {
                lock (DesiredPoseLock)
                {
                    return _lastdesiredPose;
                }
            }
        }

        RobotInfo _ThisRobot;


        // Constrctor
        public TrajectoryHandler(RobotInfo robot, ScreenWriter GUI)
        {
            _ThisRobot = robot;
            InitialiseDelegates();
            _ReferencePose = new FixedSizedQueue<Pose>(10);
            _ReferenceVelocity = new FixedSizedQueue<Pose>(10);
            _ReferencePose.Enqueue(Pose.Zero);
            _ReferenceVelocity.Enqueue(Pose.Zero);
            _TrajectoryTime = new Stopwatch();
            TrajectoryController = new Controller(_ThisRobot, GUI);
            _ActiveTrajectories = new Trajectory[] { new TrajectoryQuintic(Pose.Zero, Guid.NewGuid()) };
            _nSegments = 0;
            _CurrentSegment = 0;
            _isActive = false;
        }

        #region JointSpace Methods:

        public void goHome(Trajectory[] newTrajectory)
        {
            Load(newTrajectory);
        }

        public void Start()
        {
            try
            {
                _CurrentSegment = 0;
                _isActive = true;
                _timerHasElapsed = false;
                _TrajectoryTime.Reset();
                _lastdesiredPose = _desiredPose;
                _desiredPose = SF.forwardKinimatics((((JointTrajectory)_ActiveTrajectories[_CurrentSegment]).finalJoint),_ThisRobot.EndEffector);
            }
            catch (Exception e )
            {
                _ThisRobot.updateError("error in start", e);
            }
        }



        public void Stop()
        {
            _isActive = false;
            _TrajectoryTime.Stop();
            // Async call to populate trajectories when robot is stationary
            //((JointTrajectory)_ActiveTrajectories[_CurrentSegment]).updateStartPosition(StoppedAngle);
        }


        #endregion

        #region TaskSpace Methods:

        public void Start(Pose startPose, Pose startVelocity)
        {
            try
            {
                _CurrentSegment = 0;
                if (!(startPose == ((TaskTrajectory)_ActiveTrajectories[0]).startPose))
                {
                    ((TaskTrajectory)_ActiveTrajectories[0]).updateStartPosition(startPose, startVelocity); // can changed start velocity to be zero if velocity is noisy
                }
                _lastdesiredPose = _desiredPose;
                _desiredPose = ((TaskTrajectory)_ActiveTrajectories[_CurrentSegment]).finalPose;
                _isActive = true;
                _TrajectoryTime.Reset();
                _timerHasElapsed = false;
            }
            catch (Exception e)
            {
                _ThisRobot.updateError("error in start", e);
            }
        }

        public void ReStart(Pose CurrentPose, Pose CurrentVelocity)
        {
            if (!(CurrentPose.Equals(((TaskTrajectory)_ActiveTrajectories[_CurrentSegment]).startPose, 5e-1)))
            {
                ((TaskTrajectory)_ActiveTrajectories[_CurrentSegment]).updateStartPosition(CurrentPose, CurrentVelocity); // can changed start velocity to be zero if velocity is noisy
            }
            _lastdesiredPose = _desiredPose;
            _desiredPose = ((TaskTrajectory)_ActiveTrajectories[_CurrentSegment]).finalPose;
            _isActive = true;
            _TrajectoryTime.Reset();
            _timerHasElapsed = false;
        }

        public void Pause(Pose StoppedPose)
        {
            _isActive = false;
            _TrajectoryTime.Stop();
            // Async call to populate trajectories when robot is stationary
          //  ((TaskTrajectory)_ActiveTrajectories[_CurrentSegment]).updateStartPosition(StoppedPose, Pose.Zero);
        }

        public void Stop(Pose StoppedPose)
        {
            _isActive = false;
            _TrajectoryTime.Reset();

            _timerHasElapsed = false;
            // Async call to populate trajectories when robot is stationary
            //((TaskTrajectory)_ActiveTrajectories[_CurrentSegment]).updateStartPosition(StoppedPose, Pose.Zero);
        }


        #endregion

        #region Mixed methods

        public void Reset()
        {
            _isActive = false;
            _nSegments = 0;
            _CurrentSegment = 0;
            _TrajectoryTime.Reset();
            _timerHasElapsed = false;

        }

        public void LodeBuffer(Pose currentPose, Pose CurrentVelocity)
        {
            // Check and load from buffer
            lock (BufferLock)
            {
                if (_BufferLoaded)
                {
                    if (_bufferTrajectories[0].type != TrajectoryTypes.Joint)
                    {
                        _nSegments = _bufferTrajectories.Length;
                        _CurrentSegment = 0;
                        if (!(currentPose.Equals(((TaskTrajectory)_bufferTrajectories[0]).startPose,5e-1)))
                        {
                            ((TaskTrajectory)_bufferTrajectories[0]).updateStartPosition(currentPose, CurrentVelocity);
                        }
                        _ActiveTrajectories = new Trajectory[_nSegments];
                        for (int i = 0; i < _nSegments; i++)
                        {
                            _ActiveTrajectories[i] = _bufferTrajectories[i];
                        }
                        _bufferTrajectories = null;
                        _BufferLoaded = false;
                        Start(currentPose, CurrentVelocity);
                    }
                    else
                    {
                        _nSegments = _bufferTrajectories.Length;
                        _CurrentSegment = 0;
                        _ActiveTrajectories = new Trajectory[_nSegments];
                        for (int i = 0; i < _nSegments; i++)
                        {
                            _ActiveTrajectories[i] = _bufferTrajectories[i];
                        }
                        _bufferTrajectories = null;
                        _BufferLoaded = false;
                        Start();
                    }
                }
            }
        }

        public void startDesired(Pose StartPose)
        {
            lock (DesiredPoseLock)
            {
                _desiredPose = StartPose;
                _lastdesiredPose = StartPose;
            }
        }

        public void Load(Trajectory[] NewTrajectoryList)
        {
            try
            {
                lock (BufferLock)
                {
                    _bufferTrajectories = new Trajectory[NewTrajectoryList.Length];
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

        /// <summary>
        /// Updates the command in cartesian coordinates.
        /// </summary>
        /// <param name="currentPose"></param>
        /// <param name="CurrentVelocity"></param>
        /// <param name="currentAngle"></param>
        /// <param name="EE"></param>
        public void GetCommandCart(Pose currentPose, Pose CurrentVelocity, double[] currentAngle, Vector3 EE)
        {
            
            // Check if its active
            if (_isActive)
            {
                // Check and load from buffer
                LodeBuffer(currentPose, CurrentVelocity);

                _TrajectoryTime.Start();

                if (_ActiveTrajectories[_CurrentSegment].type != TrajectoryTypes.Joint)
                {
                    if (_ActiveTrajectories[_CurrentSegment].type == TrajectoryTypes.Quintic)
                    {
                        
                        Pose ReferencePose = ((TaskTrajectory)_ActiveTrajectories[_CurrentSegment]).getReferencePosition(_TrajectoryTime.Elapsed.TotalMilliseconds);
                        Pose ReferenceVelocity = ((TaskTrajectory)_ActiveTrajectories[_CurrentSegment]).getReferenceVelocity(_TrajectoryTime.Elapsed.TotalMilliseconds);
                        _ReferencePose.Enqueue(ReferencePose);
                        _ReferenceVelocity.Enqueue(ReferenceVelocity);

                        Vector3 ErrorTranslation = ReferencePose.Translation - currentPose.Translation;
                        Vector3 ErrorOrientation = SF.getOrientationError(ReferencePose.Orientation, currentPose.Orientation);

                        if (ErrorTranslation.Length() > 1)
                        {
                            ErrorTranslation.Normalize();
                        }

                        Vector3 ControlTranslation = ReferenceVelocity.Translation + Vector3.Multiply(ErrorTranslation, 0.01f);
                        Vector3 ControlOrientation = Vector3.Multiply(ReferenceVelocity.axis, ReferenceVelocity.angle) + Vector3.Multiply(ErrorOrientation, 0.005f);

                        double[] TipVeloicty = new double[] { ControlTranslation.X, ControlTranslation.Y, ControlTranslation.Z, ControlOrientation.X, ControlOrientation.Y, ControlOrientation.Z };
                        _ThisRobot.checkCartVelocty(TipVeloicty);
                        _ThisRobot._Commands.Enqueue(TipVeloicty);
                    }
                }
            }
        }



        public void GetCommandAxis(Pose currentPose, Pose CurrentVelocity, double[] currentAngle)
        {
            // Check and load from buffer
            LodeBuffer(currentPose, CurrentVelocity);

            // Check if its active
            if ( _isActive)
            {

                _TrajectoryTime.Start();
                if (_ActiveTrajectories[_CurrentSegment].type != TrajectoryTypes.Joint)
                {
                        Pose ReferencePose = ((TaskTrajectory)_ActiveTrajectories[_CurrentSegment]).getReferencePosition(_TrajectoryTime.Elapsed.TotalMilliseconds);
                        Pose ReferenceVelocity = ((TaskTrajectory)_ActiveTrajectories[_CurrentSegment]).getReferenceVelocity(_TrajectoryTime.Elapsed.TotalMilliseconds);
                        _ReferencePose.Enqueue(ReferencePose);
                        _ReferenceVelocity.Enqueue(ReferenceVelocity);

                       // double[,] InverseJacob = SF.GetInverseJacobian(currentAngle, _ThisRobot.EndEffector);

                        // Jacobian method
                        TrajectoryController.getControllerEffort(ReferencePose, ReferenceVelocity, currentPose, CurrentVelocity, currentAngle, _ThisRobot, _timerHasElapsed, ((TaskTrajectory)_ActiveTrajectories[_CurrentSegment]).averageVelocity);
                        
                        if (_ActiveTrajectories[_CurrentSegment].trajectoryTime.TotalMilliseconds < _TrajectoryTime.Elapsed.TotalMilliseconds)
                        {
                            _timerHasElapsed = true;
                            if (!currentPose.Equals(((TaskTrajectory)_ActiveTrajectories[_CurrentSegment]).finalPose, 1))
                            {
                                ReStart(currentPose, _ThisRobot.currentVelocity);
                            }
                        }
                        if (_timerHasElapsed && currentPose.Equals(((TaskTrajectory)_ActiveTrajectories[_CurrentSegment]).finalPose, 5))
                        {
                            _CurrentSegment++;
                            _TrajectoryTime.Reset();
                            _timerHasElapsed = false;
                            _lastdesiredPose = _desiredPose;
                            if (_CurrentSegment >= _nSegments)
                            {
                                Stop(currentPose);
                                Reset();
                            }
                            else
                            {
                                _desiredPose = ((TaskTrajectory)_ActiveTrajectories[_CurrentSegment]).finalPose;
                            }
                        }

                    
                }
                else
                {
                    double[] referenceAngles = ((JointTrajectory)_ActiveTrajectories[_CurrentSegment]).finalJoint;
                    double[] controlAngles = new double[referenceAngles.Length];

                    for (int i = 0; i < referenceAngles.Length; i++)
                    {
                        double changeAngle = referenceAngles[i] - currentAngle[i];
                        if (Math.Abs(changeAngle) <= _ThisRobot._MaxAxisChange)
                        {
                            controlAngles[i] = changeAngle;
                        }
                        else
                        {
                            controlAngles[i] = Math.Sign(changeAngle) * _ThisRobot._MaxAxisChange;
                        }
                        
                    }
                    if (controlAngles.Max() < 1e-2 && controlAngles.Min() > -1e-2 )
                    {
                        Stop();
                    }
                    _ThisRobot._Commands.Enqueue(controlAngles);
                    //return controlAngles;
                }
            }
           // return new double[] { 0, 0, 0, 0, 0, 0 };


        }

        #endregion

    }
}
