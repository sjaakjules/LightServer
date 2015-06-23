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
        bool _loadedOrientation;
        Matrix _finalPose, _startPose;
        TimeSpan _trajectoryTime;
        Vector3 _axis;
        float _finalAngle;

        RobotInfo _robot;

        public Controller(RobotInfo robot)
        {
            _robot = robot;
            _isActive = false;
            _loadedOrientation = false;
            _finalPose = Matrix.Identity;
        }

        public void load(Vector3 position)
        {
            _finalPose.Translation = position;
        }

        public void load(Quaternion orientation)
        {
            Vector3 translation = _finalPose.Translation;
            _finalPose = Matrix.CreateFromQuaternion(orientation);
            _finalPose.Translation = translation;
        }

        public void load(Vector3 position, Quaternion orientation)
        {
            _finalPose = Matrix.CreateFromQuaternion(orientation);
            _finalPose.Translation = position;
        }



        public Vector3 getDisplacement(Vector3 currentPosition, double maxChange)
        {
            if (Vector3.Distance(_finalPose.Translation, currentPosition) > maxChange)
            {
                return Vector3.Multiply(Vector3.Normalize(_finalPose.Translation - currentPosition), (float)maxChange);
            }
            return _finalPose.Translation - currentPosition;
        }

        public Vector3 getOrientation(Quaternion currentOrientation, float maxChange)
        {
            Quaternion changeOrientation = Quaternion.Inverse(currentOrientation) * Quaternion.CreateFromRotationMatrix(_finalPose);
            Vector3 axis = Vector3.Zero;
            float angle = 0; 
            StaticFunctions.getAxisAngle(changeOrientation, ref axis, ref angle);
            if (Math.Abs(angle) > maxChange)
            {
                angle = Math.Sign(angle) * maxChange;
            }
            float[] kukaAngles = new float[6];
            StaticFunctions.getKukaAngles(Quaternion.CreateFromAxisAngle(axis, angle), ref kukaAngles);
            return new Vector3(kukaAngles[3], kukaAngles[4], kukaAngles[5]);
        }


    }
}
