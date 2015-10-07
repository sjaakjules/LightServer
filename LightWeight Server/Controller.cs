using Microsoft.Xna.Framework;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace LightWeight_Server
{
    class Controller
    {
        //object movementLock = new object();
        Stopwatch _elapsedTime = new Stopwatch();
        bool _isActive = false;
        public bool _isRotating, _isMoving;
        Matrix _finalPose, _startPose;
        Quaternion _startOrientation, _lastOrientation;
        TimeSpan _trajectoryTime;
        Vector3 _axis;
        float _finalAngle;
        double P, I, D;


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
                   // _elapsedTime.Restart();
                    _isActive = value;
                }
                else if (_isActive & !value)
                {
                   // _elapsedTime.Reset();
                    _isActive = value;
                }

            }
        }


        public Controller(RobotInfo robot)
        {
            _isActive = false;
            _isMoving = false;
            _isRotating = false;
            _finalPose = Matrix.Identity;
        }

        public void updateP(double newP)
        {
            // TODO: link update
            P = newP;
        }
        public void updateI(double newI)
        {
            // TODO: link update
            I = newI;
        }
        public void updateD(double newD)
        {
            // TODO: link update
            D = newD;
        }

        public double[] getControllerEffort(Pose referencePosition, Pose referenceVelocity, Pose measuredPosition, Pose measuredVelocity,double[,] Jacobian)
        {
            // TODO: write PID controller, may need karman filter for noise
            return new double[] { 0, 0, 0, 0, 0, 0 };
        }

        public void load(Vector3 position, Matrix startPose)
        {
            _finalPose.Translation = position;
            _startPose.Translation = startPose.Translation;
            _startOrientation = Quaternion.CreateFromRotationMatrix(startPose);
            _isMoving = true;
            _isActive = true;
        }

        public void load(Quaternion FinalOrientation, Matrix startPose, long timespan)
        {
            SF.getAxisAngle(FinalOrientation, out _axis, out _finalAngle);
            _trajectoryTime = new TimeSpan(timespan);
            Vector3 translation = _startPose.Translation;
            _startPose = startPose;
            _startPose.Translation = translation;
            _startOrientation = Quaternion.CreateFromRotationMatrix(startPose);
            _lastOrientation = new Quaternion(_startOrientation.X, _startOrientation.Y, _startOrientation.Z, _startOrientation.W);
            translation = _finalPose.Translation;
            _finalPose = Matrix.CreateFromQuaternion(FinalOrientation);
            _finalPose.Translation = translation;
            _elapsedTime.Restart();
            _isRotating = true;
            _isActive = true;
        }

        public void load(Vector3 position, Quaternion FinalOrientation, Matrix startPose, long timespan)
        {
            SF.getAxisAngle(FinalOrientation, out _axis, out _finalAngle);
            _trajectoryTime = new TimeSpan(timespan);
            _startPose = startPose;
            _startOrientation = Quaternion.CreateFromRotationMatrix(startPose);
            _lastOrientation = new Quaternion(_startOrientation.X, _startOrientation.Y, _startOrientation.Z, _startOrientation.W);
            _finalPose = Matrix.CreateFromQuaternion(FinalOrientation);
            _finalPose.Translation = position;
            _elapsedTime.Restart();
            _isRotating = true;
            _isMoving = true;
            _isActive = true;
        }



        public Vector3 getDisplacement(Vector3 currentPosition, double maxChange)
        {
            if (!_isMoving && !_isRotating)
            {
                IsActive = false;
            }
            if (_isMoving && IsActive)
            {
                if (Math.Abs(Vector3.Distance(_startPose.Translation,currentPosition)) < 1)
                {
                    return Vector3.Multiply(Vector3.Normalize(_finalPose.Translation - currentPosition),
                        (Math.Abs(Vector3.Distance(_startPose.Translation, currentPosition)) < 0.01 / maxChange) ? 0.01f : (float)maxChange * Math.Abs(Vector3.Distance(_startPose.Translation, currentPosition)));
                    
                }
                if (Math.Abs(Vector3.Distance(_finalPose.Translation, currentPosition)) < 0.05 / maxChange)
                {
                    _isMoving = false;
                    return Vector3.Multiply(Vector3.Normalize(_finalPose.Translation - currentPosition), Math.Abs(Vector3.Distance(_finalPose.Translation, currentPosition)));
                }
                return Vector3.Multiply(Vector3.Normalize(_finalPose.Translation - currentPosition),
                          (Vector3.Distance(_finalPose.Translation, currentPosition) > 5) ? (float)maxChange : (float)maxChange * Math.Abs(Vector3.Distance(_finalPose.Translation, currentPosition)) / 5);
                
            }
            return Vector3.Zero;
        }

        public Vector3 getOrientation(Quaternion currentOrientation, float maxChange)
        {
            float Duration = (float)(_elapsedTime.Elapsed.TotalMilliseconds / _trajectoryTime.TotalMilliseconds);
            Duration = (Duration >= 1.0) ? 1.0f : Duration;
            if (!_isMoving && !_isRotating)
            {
                IsActive = false;
            }
            if (_isRotating && IsActive)
            {
                Quaternion refChange = Quaternion.CreateFromAxisAngle(_axis, _finalAngle*Duration);
                Quaternion referenceQ = _startOrientation * refChange;
                Quaternion changeQ = Quaternion.Inverse(_lastOrientation)* referenceQ  ;
                _lastOrientation = new Quaternion(referenceQ.X,referenceQ.Y,referenceQ.Z,referenceQ.W);

             
                float[] kukaAngles = new float[6];
                SF.getKukaAngles(changeQ, ref kukaAngles);
                //_robot.updateError("kukaAngles: " + kukaAngles[0].ToString() + " : " + kukaAngles[1].ToString() + " : " + kukaAngles[2].ToString() + " : " + kukaAngles[3].ToString() + " : " + kukaAngles[4].ToString() + " : " + kukaAngles[5].ToString() );
                if (Duration >= 1.0f)
                {
                    _isRotating = false;
                }

                return new Vector3(kukaAngles[3], kukaAngles[4], kukaAngles[5]);
            }
            return Vector3.Zero;
        }


    }
}
