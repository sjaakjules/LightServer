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
        Stopwatch _elapsedTime = new Stopwatch();
        bool _isActive = false;
        public bool _isRotating, _isMoving;
        Matrix _finalPose, _startPose;
        Quaternion _startOrientation, __FinalOrientation;
        TimeSpan _trajectoryTime;
        Vector3 _axis;
        float _finalAngle;

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


        public Controller(RobotInfo robot)
        {
            _robot = robot;
            _isActive = false;
            _isMoving = false;
            _isRotating = false;
            _finalPose = Matrix.Identity;
        }

        public void load(Vector3 position, Matrix startPose)
        {
            _finalPose.Translation = position;
            if (!_isMoving)
            {
                _startPose.Translation = startPose.Translation;
                _startOrientation = Quaternion.CreateFromRotationMatrix(startPose);
            }
            _isMoving = true;
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
            if (!_isMoving)
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
                float accelerationDistance = 5.0f;
                if ((Math.Abs(Vector3.Distance(_startPose.Translation, _finalPose.Translation)) > accelerationDistance))
                {
                    // in the start/final region
                    // Check for last step occurance

                    if ((Math.Abs(Vector3.Distance(_finalPose.Translation, currentPosition))) < 0.01f)
                    {
                        _isMoving = false;
                        return _finalPose.Translation - currentPosition;
                    }
                    else if (Math.Abs(Vector3.Distance(_startPose.Translation, currentPosition)) < accelerationDistance / 2 ||
                        (Math.Abs(Vector3.Distance(_finalPose.Translation, currentPosition)) < accelerationDistance / 2))
                    {
                        // If its in start of final region find two distances
                        float disFromStart = Math.Abs(Vector3.Distance(_startPose.Translation, currentPosition)) + .05f;
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
                    // TODO code when agent is on robot, ie less than 10mm commands
                }


                /*
                // If you are wihtin 1mm of the start
                if (Math.Abs(Vector3.Distance(_startPose.Translation,currentPosition)) < 3)
                {
                    return Vector3.Multiply(Vector3.Normalize(_finalPose.Translation - currentPosition),
                        (Math.Abs(Vector3.Distance(_startPose.Translation, currentPosition)) < 0.01 / maxChange) ? 0.01f : (float)maxChange * Math.Abs(Vector3.Distance(_startPose.Translation, currentPosition)));
                    
                }
                // If you are within 0.05mm of the end
                if (Math.Abs(Vector3.Distance(_finalPose.Translation, currentPosition)) < 0.05 / maxChange)
                {
                    _isMoving = false;
                    return Vector3.Multiply(Vector3.Normalize(_finalPose.Translation - currentPosition), Math.Abs(Vector3.Distance(_finalPose.Translation, currentPosition)));
                }
                 * 
                // in all other times
                return Vector3.Multiply(Vector3.Normalize(_finalPose.Translation - currentPosition),
                          (Vector3.Distance(_finalPose.Translation, currentPosition) > 5) ? (float)maxChange : (float)maxChange * Math.Abs(Vector3.Distance(_finalPose.Translation, currentPosition)) / 5 );
                
                 */
            
            }
            return Vector3.Zero;
        }


        public Vector3 getDisplacement(Vector3 currentPosition, Vector3 lastPosition, double maxChange)
        {
            if (!_isMoving && !_isRotating)
            {
                IsActive = false;
            }
            if (_isMoving && IsActive)
            {
                if (Math.Abs(Vector3.Distance(lastPosition, currentPosition)) < 0.0001)
                {
                    return Vector3.Multiply(Vector3.Normalize(_finalPose.Translation - currentPosition), 0.1f);
                }
                if (Vector3.Distance(_finalPose.Translation, currentPosition) > maxChange / 100)
                {
                    return Vector3.Multiply(Vector3.Normalize(_finalPose.Translation - currentPosition),
                          (Vector3.Distance(_finalPose.Translation, currentPosition) > 20) ? (float)maxChange : (float)maxChange * Vector3.Distance(_finalPose.Translation, currentPosition) / 20);
                }
                else
                {
                    _isMoving = false;
                }
                return _finalPose.Translation - currentPosition;
            }
            return Vector3.Zero;
        }

        public Vector3 getOrientation(Quaternion currentOrientation, float maxChange)
        {
          //  float Duration = (float)(_elapsedTime.Elapsed.TotalMilliseconds / _trajectoryTime.TotalMilliseconds);
          //  Duration = (Duration >= 1.0) ? 1.0f : Duration;
            if (!_isMoving && !_isRotating)
            {
                IsActive = false;
            }
            if (_isRotating && IsActive)
            {
                Quaternion changeQ = Quaternion.Identity;
                Quaternion currentInvers = Quaternion.Inverse(currentOrientation);
                Quaternion toFinal = currentInvers * __FinalOrientation;
                Vector3 toFinalAxis = Vector3.Zero;
                float toFinalAngle = 0;
                SF.getAxisAngle(toFinal, ref toFinalAxis, ref toFinalAngle);
                if (Math.Abs(toFinalAngle) < 1e-2)
                {
                    _isRotating = false;
                }
                else
                {
                    toFinalAxis.Normalize();
                    changeQ = Quaternion.CreateFromAxisAngle(toFinalAxis, Math.Sign(toFinalAngle) * MathHelper.ToRadians(maxChange));
                }
                /*
                Quaternion refChange = Quaternion.CreateFromAxisAngle(_axis, _finalAngle*Duration);
                Quaternion referenceQ = _startOrientation * refChange;
                Quaternion changeQ = currentInvers * referenceQ;
                */


                /*
                _robot.updateError(Duration.ToString() + " Current Z: " + Matrix.CreateFromQuaternion(Quaternion.Inverse(currentOrientation) * Quaternion.CreateFromRotationMatrix(_startPose)).ToString());
                _robot.updateError(Duration.ToString() + " changeq: " + Matrix.CreateFromQuaternion(changeQ).ToString());
                _robot.updateError(Duration.ToString() + " ref change: " + Matrix.CreateFromQuaternion(refChange).ToString());
                
                 * 
                 * 
                 *
                Vector3 axis = Vector3.Zero;
                float angle = 0;
                SF.getAxisAngle(changeQ, ref axis, ref angle);
                _robot.updateError(Duration.ToString() + "Change Angle: " + angle.ToString());
                
                if (Math.Abs(angle) > maxChange)
                {
                    _robot.updateError("large angle: " + angle.ToString());
                    angle = Math.Sign(angle) * maxChange;
                }
                _robot.updateError("angle: " + angle.ToString());
                */
                float[] kukaAngles = new float[6];
                SF.getKukaAngles(changeQ, ref kukaAngles);
                //_robot.updateError("kukaAngles: " + kukaAngles[0].ToString() + " : " + kukaAngles[1].ToString() + " : " + kukaAngles[2].ToString() + " : " + kukaAngles[3].ToString() + " : " + kukaAngles[4].ToString() + " : " + kukaAngles[5].ToString() );
                
                /*
                
                if (Duration >= 1.0f)
                {
                    float finalAngle = 0;
                    Vector3 finalAxis = Vector3.Zero;
                    Quaternion finalError = currentInvers * __FinalOrientation;
                    SF.getAxisAngle(finalError, ref finalAxis, ref finalAngle);
                    if (Math.Abs(finalAngle) < 1e-1)
                    {
                        _isRotating = false;

                    }
                    else
                    {
                        
                        long orientationDuration = (long)(TimeSpan.TicksPerSecond * (finalAngle / (maxChange * 10)));
                        load(finalError, currentPose, orientationDuration);
                    }
                }

                */
                //_robot.updateError(Duration.ToString() + " Angles: {" + kukaAngles[3].ToString() + " , " + kukaAngles[4].ToString() + " , " + kukaAngles[5].ToString() + ") ");
               
               // _robot.updateError(Duration.ToString() + "Orientation Timer: " + timer.Elapsed.TotalMilliseconds.ToString());

                return new Vector3(kukaAngles[3], kukaAngles[4], kukaAngles[5]);
            }
            return Vector3.Zero;
        }


    }
}
