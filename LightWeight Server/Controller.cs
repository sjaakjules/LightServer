using Microsoft.Xna.Framework;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace LightWeight_Server
{
    class Controller
    {
    //    StringBuilder DataWriter = new StringBuilder();
    //    string dataWriterFile = "ControllerData";
    //    StreamWriter Datafile;
        //object movementLock = new object();
        Stopwatch _DataTime = new Stopwatch();
        double Pa = 0.0001, Px = 0.005, Pt = 0.0001;



        public Controller(RobotInfo ThisRobot)
        {
            //this._x, this._y, this._z, this.angle, this.axis.X, this.axis.Y, this.axis.Z
            /*
            DataWriter.AppendFormat("time,ref_rx,ref_ry,ref_rz,ref_ra,ref_rax,ref_ray,ref_raz,act_rx,act_ry,act_rz,act_ra,act_rax,act_ray,act_raz,ref_vx,ref_vy,ref_vz,ref_va,ref_vax,ref_vay,ref_vaz,act_vx,act_vy,act_vz,act_va,act_vax,act_vay,act_vaz,axis1,axis2,axis3,axis4,axis5,axis6,Saxis1,Saxis2,Saxis3,Saxis4,Saxis5,Saxis6,Eaxis1,Eaxis2,Eaxis3,Eaxis4,Eaxis5,Eaxis6");
            //DataWriter.AppendFormat("time,ref_rx,ref_ry,ref_rz,ref_ra,ref_rax,ref_ray,ref_raz,act_rx,act_ry,act_rz,act_ra,act_rax,act_ray,act_raz,act_vx,act_vy,act_vz,act_va,act_vax,act_vay,act_vaz,refa1,refa2,refa3,refa4,refa5,refa6,axis1,axis2,axis3,axis4,axis5,axis6,err1,err2,err3,err4,err5,err6;");
            try
            {
                using (Datafile = new StreamWriter(dataWriterFile + ".csv"))
                {
                    Datafile.WriteLine(DataWriter);
                    DataWriter.Clear();
                    Datafile.Flush();
                    Datafile.Close();
                }
            }
            catch (Exception)
            {
                dataWriterFile = dataWriterFile + Guid.NewGuid().ToString("N");
                using (Datafile = new StreamWriter(dataWriterFile + ".csv"))
                {
                    Datafile.WriteLine(DataWriter);
                    DataWriter.Clear();
                    Datafile.Flush();
                    Datafile.Close();
                }
            }
             */
            _DataTime.Start();
        }


        /// <summary>
        /// IK solver method, uses axis position to calculate next change.
        /// </summary>
        /// <param name="referencePosition"></param>
        /// <param name="measuredAngles"></param>
        /// <param name="measuredPosition"></param>
        /// <param name="measuredVelocity"></param>
        /// <param name="robot"></param>
        public void getControllerErrort(Pose referencePosition, double[] measuredAngles,Pose measuredPosition,Pose measuredVelocity, RobotInfo robot)
        {
            double[] referenceAngles = robot.IKSolver(referencePosition, robot.EndEffector, measuredAngles, ref robot._elbow, ref robot._base);
            double[] controlAngles = new double[referenceAngles.Length];
            for (int i = 0; i < referenceAngles.Length; i++)
            {
                controlAngles[i] = (referenceAngles[i] - measuredAngles[i])*Pa;
            }

            robot.checkLimits(controlAngles);
            robot._Commands.Enqueue(controlAngles);

            /*
            SF.updateDataFile(referencePosition, measuredPosition, measuredVelocity, _DataTime.Elapsed.TotalMilliseconds, referenceAngles, controlAngles,measuredAngles, DataWriter);
            using (StreamWriter Datafile = new StreamWriter(dataWriterFile + ".csv", true))
            {
                Datafile.WriteLine(DataWriter);
            }
            DataWriter.Clear();
             */
            
        }


        /*
        public void getControllerEffort(Pose referencePosition, Pose referenceVelocity, Pose measuredPosition, Pose measuredVelocity, double[,] inverseJoc,double[] measuredAngle, RobotInfo robot,bool hasElapsed)
        {
                if (hasElapsed)
                {
                    P3 = 0.005;
                }
                else
                {
                    P3 = 0.001;
                }

                double[] ReferenceAngle = SF.IKSolver(referencePosition,robot.EndEffector,measuredAngle,ref robot._elbow, ref robot._base);

                Vector3 ErrorTranslation = referencePosition.Translation - measuredPosition.Translation;
                Vector3 ErrorOrientation = SF.getOrientationError(Matrix.CreateFromQuaternion(referencePosition.Orientation), Matrix.CreateFromQuaternion(measuredPosition.Orientation));

                if (ErrorTranslation.Length() < 1)
                {
                    Vector3.Multiply(ErrorTranslation, ErrorTranslation.LengthSquared());
                }
                else if (ErrorTranslation.Length() < 10)
                {
                    Vector3.Multiply(ErrorTranslation,(ErrorTranslation.Length())/10);
                }
                else
                {
                    ErrorTranslation.Normalize();
                }

              //  double[] AngleError = SF.addDoubles(ReferenceAngle, SF.multiplyMatrix(currentAngles, -1.0));

                Vector3 ControlTranslation = referenceVelocity.Translation + Vector3.Multiply(ErrorTranslation, (float)P);
                Vector3 ControlOrientation = Vector3.Multiply(referenceVelocity.axis, referenceVelocity.angle) + Vector3.Multiply(ErrorOrientation, (float)P / 100);

                double[] TipVeloicty = new double[] { ControlTranslation.X, ControlTranslation.Y, ControlTranslation.Z, ControlOrientation.X, ControlOrientation.Y, ControlOrientation.Z };
              //  robot._Commands.Enqueue(TipVeloicty);
                
                double[] AxisSpeed = SF.multiplyMatrix(inverseJoc, TipVeloicty);
             //   AxisSpeed = SF.addDoubles(AxisSpeed, SF.multiplyMatrix(AngleError, P));

           //     double[] SimulatedAngles = SF.addDoubles(currentAngles, SF.multiplyMatrix(AxisSpeed, 4));
            //    double[] AngleError2 = SF.addDoubles(ReferenceAngle, SF.multiplyMatrix(SimulatedAngles, -1.0));
               // AxisSpeed = SF.addDoubles(AxisSpeed, SF.multiplyMatrix(AngleError2, P2));

                robot.checkLimits(AxisSpeed);
                robot._Commands.Enqueue(AxisSpeed);


                SF.updateDataFile(referencePosition, referenceVelocity, measuredPosition, measuredVelocity, _DataTime.Elapsed.TotalMilliseconds, ReferenceAngle, measuredAngle, DataWriter);
                using (StreamWriter Datafile = new StreamWriter(dataWriterFile + ".csv", true))
                {
                    Datafile.WriteLine(DataWriter);
                }
                DataWriter.Clear();


                //return AxisSpeed;
            
        }

         */

        void setGain(ref Vector3 ErrorTranslation, ref Vector3 ErrorOrientation, ref double Px, ref double Pt, bool hasElapsed, double averageSpeed)
        {

            if (hasElapsed)
            {
                Px = averageSpeed / 5;
                Pt = averageSpeed / 20;
                if (ErrorTranslation.Length() < 5)
                {
                    Px = 1.0 * averageSpeed / 1;
                    ErrorTranslation = Vector3.Multiply(ErrorTranslation, ErrorTranslation.Length()/10);
                }
                if (ErrorTranslation.Length() < 200)
                {
                   // Px = 1.0 * averageSpeed / 5;
                    ErrorTranslation = Vector3.Multiply(Vector3.Normalize(ErrorTranslation), 1.0f * ErrorTranslation.Length() / 200);
                }
                else
                {
                   // Px = averageSpeed/2;
                    ErrorTranslation.Normalize();
                }

                if (ErrorOrientation.Length() < 0.01)
                {
                    //Pt = 0.0001;
                    ErrorOrientation = Vector3.Multiply(ErrorOrientation, ErrorOrientation.Length());
                }
                else if (ErrorOrientation.Length() < 0.01)
                {
                   // Pt = 0.0005;
                    ErrorOrientation = Vector3.Multiply(Vector3.Normalize(ErrorOrientation), 0.01f * ErrorOrientation.Length() / 0.1f);
                }
                else
                {
                   // Pt = 0.001;
                    Vector3.Multiply(Vector3.Normalize(ErrorOrientation), 0.001f);
                }
            }
            else
            {
                Px = 0.01; // 0.01
                Pt = 0.0001;
                /*
                if (ErrorTranslation.Length() < 1)
                {
                  //  Px = 1.0 * averageSpeed / 8;
                    ErrorTranslation = Vector3.Multiply(ErrorTranslation, ErrorTranslation.Length());
                }
                 */
                if (ErrorTranslation.Length() < 10)
                {
                  //  Px = 1.0 * averageSpeed / 10;
                    ErrorTranslation = Vector3.Multiply(Vector3.Normalize(ErrorTranslation), 1.0f * ErrorTranslation.Length() / 10);
                }
                else
                {
                  //  Px = 1.0 * averageSpeed / 2;
                    ErrorTranslation.Normalize();
                }

                if (ErrorOrientation.Length() < 0.001)
                {
                  //  Pt = 0;
                    ErrorOrientation = Vector3.Multiply(ErrorOrientation, ErrorOrientation.LengthSquared());
                }
                else if (ErrorOrientation.Length() < 0.01)
                {
                   // Pt = 0.0005;
                    ErrorOrientation = Vector3.Multiply(Vector3.Normalize(ErrorOrientation), 0.001f * ErrorOrientation.Length() / 0.01f);
                }
                else
                {
                   // Pt = 0.001;
                    Vector3.Multiply(Vector3.Normalize(ErrorOrientation), 0.001f);
                }
            }
        }

        public void getControllerEffort(Pose referencePosition, Pose referenceVelocity, Pose measuredPosition, Pose measuredVelocity, double[] measuredAngle, RobotInfo robot, bool hasElapsed, double averageSpeed)
        {
            double[] ReferenceAngle = robot.IKSolver(referencePosition,measuredAngle);
            double[] AngleError = SF.addDoubles(ReferenceAngle, SF.multiplyMatrix(measuredAngle, -1.0));

            Vector3 ErrorTranslation = referencePosition.Translation - measuredPosition.Translation;
            Vector3 ErrorOrientation = SF.getOrientationError(Matrix.CreateFromQuaternion(referencePosition.Orientation), Matrix.CreateFromQuaternion(measuredPosition.Orientation));

            setGain(ref ErrorTranslation, ref ErrorOrientation, ref Px, ref Pt, hasElapsed, averageSpeed);
            
           // Pt = 0;
            Px =  0.2;
            Pt = Px / 200;
            Vector3 ControlTranslation = referenceVelocity.Translation + Vector3.Multiply(ErrorTranslation, (float)Px);
            Vector3 ControlOrientation = Vector3.Multiply(Vector3.Normalize(referenceVelocity.axis), referenceVelocity.angle) + Vector3.Multiply(ErrorOrientation, (float)Pt);

            double[] TipVeloicty = new double[] { ControlTranslation.X, ControlTranslation.Y, ControlTranslation.Z, ControlOrientation.X, ControlOrientation.Y, ControlOrientation.Z };

            // IK solver method
            float tipVelocityAngle = ControlOrientation.Length();
            Vector3 tipVelocityAxis = Vector3.Normalize(ControlOrientation);
            Quaternion EstimatedOrientation = Quaternion.CreateFromAxisAngle(tipVelocityAxis, tipVelocityAngle) * measuredPosition.Orientation;
            Vector3 EstimatedTranslation = measuredPosition.Translation + ControlTranslation;
            Pose EstimatedPose = new Pose(EstimatedOrientation, EstimatedTranslation);

            double[] EstimatedAxis = robot.IKSolver(EstimatedPose, measuredAngle);
            double[] AxisSpeedIK = SF.addDoubles(EstimatedAxis, SF.multiplyMatrix(measuredAngle, -1));
            // Jacobian method
         //   double[] AxisSpeed = SF.multiplyMatrix(inverseJoc, TipVeloicty);
        //    robot.updateError(string.Format("IK:\t{0}\nJ:\t{1}", SF.printDouble(AxisSpeedIK), SF.printDouble(AxisSpeed)), new MatrixException("Axis ik and jac reuslts:"));

            double[] SatAxisSpeed = robot.checkLimits(AxisSpeedIK);
            double[] Com = new double[6];
            double[] ComSat = new double[6];
            for (int i = 0; i < 6; i++)
            {
                Com[i] = AxisSpeedIK[i];
                ComSat[i] = SatAxisSpeed[i];
            }
            robot._Commands.Enqueue(SatAxisSpeed);

            /*
            SF.updateDataFile(referencePosition, referenceVelocity, measuredPosition, measuredVelocity, _DataTime.Elapsed.TotalMilliseconds, Com, ComSat, AngleError, DataWriter);
            using (StreamWriter Datafile = new StreamWriter(dataWriterFile + ".csv", true))
            {
                Datafile.WriteLine(DataWriter);
                DataWriter.Clear();
            }
            
             */

            //return AxisSpeed;
        }
    }
}
