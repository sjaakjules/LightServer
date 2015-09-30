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
        object movementLock = new object();
        object viaModeLock = new object();


        Stopwatch _elapsedTime = new Stopwatch();
        bool _isActive = false;
        public bool _isRotating, _isTranslating;
        Matrix _finalPose, _startPose;
        Quaternion _startOrientation, __FinalOrientation;
        TimeSpan _trajectoryTime;
        Vector3 _axis;
        float _finalAngle;
        bool _ViaMode = true;

        RobotInfo _robot;

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

        public bool ViaMode
        {
            get 
            {
                lock (viaModeLock)
                {
                    return _ViaMode;
                }
            }
            set 
            {
                lock (viaModeLock)
                {
                    _ViaMode = value;
                }
            }
        }

        public Controller(RobotInfo robot)
        {
            _robot = robot;
            _isActive = false;
            _isTranslating = false;
            _isRotating = false;
            _finalPose = Matrix.Identity;
        }

        public void load(Vector3 position, Matrix startPose)
        {
            _finalPose.Translation = position;
            if (!_isTranslating)
            {
                _startPose.Translation = startPose.Translation;
                _startOrientation = Quaternion.CreateFromRotationMatrix(startPose);
            }
            _isTranslating = true;
            _isActive = true;
        }

        public void load(Quaternion FinalOrientation, Quaternion startOrientation)
        {
            //SF.getAxisAngle(FinalOrientation, ref _axis, ref _finalAngle);
            //_trajectoryTime = new TimeSpan(timespan);
            _startOrientation = new Quaternion(startOrientation.X, startOrientation.Y, startOrientation.Z, startOrientation.W);
            __FinalOrientation = new Quaternion(FinalOrientation.X,FinalOrientation.Y,FinalOrientation.Z,FinalOrientation.W);
            Vector3 translation = _finalPose.Translation;
            _finalPose = Matrix.CreateFromQuaternion(FinalOrientation);
            _finalPose.Translation = translation;
            _isRotating = true;
            _isActive = true;
        }

        public void load(Vector3 position, Quaternion FinalOrientation, Quaternion startOrientation, Vector3 startPosition)
        {
            //SF.getAxisAngle(FinalOrientation, ref _axis, ref _finalAngle);
            //_trajectoryTime = new TimeSpan(timespan);
            if (!_isTranslating)
            {
                _startPose = Matrix.CreateFromQuaternion(startOrientation);
                _startPose.Translation = startPosition;
            }
            _startOrientation = new Quaternion(startOrientation.X, startOrientation.Y, startOrientation.Z, startOrientation.W);
            __FinalOrientation = new Quaternion(FinalOrientation.X, FinalOrientation.Y, FinalOrientation.Z, FinalOrientation.W);
            _finalPose = Matrix.CreateFromQuaternion(FinalOrientation);
            _finalPose.Translation = position;
            _elapsedTime.Restart();
            _isRotating = true;
            _isTranslating = true;
            _isActive = true;
        }



        public Vector3 getDisplacement(Vector3 currentPosition, double maxChange)
        {
            if (!_isTranslating && !_isRotating)
            {
                IsActive = false;
            }
            if (_isTranslating && IsActive)
            {

                if ((Math.Abs(Vector3.Distance(_finalPose.Translation, currentPosition))) < maxChange)
                {
                    _isTranslating = false;
                    return Vector3.Multiply(Vector3.Normalize(_finalPose.Translation - currentPosition), (float)maxChange);
                }
                if (_ViaMode)
                {
                    return Vector3.Multiply(Vector3.Normalize(_finalPose.Translation - currentPosition), (float)maxChange);
                }
                float accelerationDistance = 1.0f;
                if ((Math.Abs(Vector3.Distance(_startPose.Translation, _finalPose.Translation)) > accelerationDistance))
                {
                    // in the start/final region
                    // Check for last step occurance

                    if ((Math.Abs(Vector3.Distance(_finalPose.Translation, currentPosition))) < 0.05f)
                    {
                        _isTranslating = false;
                        return _finalPose.Translation - currentPosition;
                    }
                    else if (Math.Abs(Vector3.Distance(_startPose.Translation, currentPosition)) < accelerationDistance / 2 ||
                        (Math.Abs(Vector3.Distance(_finalPose.Translation, currentPosition)) < accelerationDistance / 2))
                    {
                        // If its in start or final region find two distances
                        float disFromStart = Math.Abs(Vector3.Distance(_startPose.Translation, currentPosition)) + .01f;
                        float disFromFinal = Math.Abs(Vector3.Distance(_finalPose.Translation, currentPosition));
                        return Vector3.Multiply(Vector3.Normalize(_finalPose.Translation - currentPosition),
                            (float)maxChange * ((disFromStart < disFromFinal) ? disFromStart : disFromFinal) / (accelerationDistance / 2));
                    }
                    else
                    {
                        return Vector3.Multiply(Vector3.Normalize(_finalPose.Translation - currentPosition), (float)maxChange);
                    }
                }
                else
                {
                    return Vector3.Multiply(Vector3.Normalize(_finalPose.Translation - currentPosition), (float)maxChange);
                    // TODO code when agent is on robot, ie less than 1mm commands
                }

            
            }
            return Vector3.Zero;
        }

        
        public Vector3 getOrientation(Quaternion currentOrientation, float maxChange)
        {
           // float Duration = (float)(_elapsedTime.Elapsed.TotalMilliseconds / _trajectoryTime.TotalMilliseconds);
           // Duration = (Duration >= 1.0) ? 1.0f : Duration;

            if (!_isTranslating && !_isRotating)
            {
                IsActive = false;
            }
            if (_isRotating && IsActive)
            {
                Quaternion changeQ = Quaternion.Identity;
                Quaternion currentInvers = Quaternion.Inverse(currentOrientation);
                Quaternion toFinal = __FinalOrientation * currentInvers;
                Vector3 toFinalAxis = Vector3.Zero;
                float toFinalAngle = 0;
                SF.getAxisAngle(toFinal, ref toFinalAxis, ref toFinalAngle);
                if (Math.Abs(MathHelper.ToDegrees(toFinalAngle)) < maxChange)
                {
                    _isRotating = false;
                }
                toFinalAxis.Normalize();
                changeQ = Quaternion.CreateFromAxisAngle(toFinalAxis, Math.Sign(toFinalAngle) * MathHelper.ToRadians(maxChange));
                float[] kukaAngles = new float[6];
                SF.getKukaAngles(changeQ, ref kukaAngles);
                return new Vector3(kukaAngles[3], kukaAngles[4], kukaAngles[5]);
            }
            return Vector3.Zero;
        }


    }
}
