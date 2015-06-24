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
        bool _isRotating, _isMoving;
        Matrix _finalPose, _startPose;
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


        public Controller(RobotInfo robot)
        {
            _robot = robot;
            _isActive = false;
            _isMoving = false;
            _isRotating = false;
            _finalPose = Matrix.Identity;
        }

        public void load(Vector3 position)
        {
            _finalPose.Translation = position;
            _isMoving = true;
            _isActive = true;
        }

        public void load(Quaternion orientation)
        {
            Vector3 translation = _finalPose.Translation;
            _finalPose = Matrix.CreateFromQuaternion(orientation);
            _finalPose.Translation = translation;
            _isRotating = true;
            _isActive = true;
        }

        public void load(Vector3 position, Quaternion orientation)
        {
            _finalPose = Matrix.CreateFromQuaternion(orientation);
            _finalPose.Translation = position;
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
                if (Vector3.Distance(_finalPose.Translation, currentPosition) > maxChange)
                {
                    return Vector3.Multiply(Vector3.Normalize(_finalPose.Translation - currentPosition),
                          (Vector3.Distance(_finalPose.Translation, currentPosition) > 20) ? (float)maxChange : (float)maxChange*Vector3.Distance(_finalPose.Translation, currentPosition) / 20);
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
            if (!_isMoving && !_isRotating)
            {
                IsActive = false;
            }
            if (_isRotating && IsActive)
            {
                Matrix changeOrientation = Matrix.Invert(Matrix.CreateFromQuaternion(currentOrientation)) * _finalPose;
                Vector3 axis = Vector3.Zero;
                float angle = 0;
                StaticFunctions.getAxisAngle(Quaternion.CreateFromRotationMatrix(changeOrientation), ref axis, ref angle);
                if (Math.Abs(angle) > maxChange)
                {
                    angle = Math.Sign(angle) * ((Math.Abs(angle) > (5.0f * (float)Math.PI / 180)) ? maxChange : Math.Abs(angle) / maxChange*(5.0f * (float)Math.PI / 180));
                }
                else
                {
                    _isRotating = false;
                }
                float[] kukaAngles = new float[6];
                StaticFunctions.getKukaAngles(Quaternion.CreateFromAxisAngle(axis, angle), ref kukaAngles);
                return new Vector3(kukaAngles[3], kukaAngles[4], kukaAngles[5]);
            }
            return Vector3.Zero;
        }


    }
}
