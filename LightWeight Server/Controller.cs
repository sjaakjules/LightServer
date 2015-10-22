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
        StringBuilder DataWriter = new StringBuilder();
        string dataWriterFile = "ControllerData";
        StreamWriter Datafile;
        //object movementLock = new object();
        Stopwatch _DataTime = new Stopwatch();
        double P = 0, I, D;



        public Controller(RobotInfo ThisRobot)
        {
            //this._x, this._y, this._z, this.angle, this.axis.X, this.axis.Y, this.axis.Z
            DataWriter.AppendFormat("time,ref_rx,ref_ry,ref_rz,ref_ra,ref_rax,ref_ray,ref_raz,act_rx,act_ry,act_rz,act_ra,act_rax,act_ray,act_raz,ref_vx,ref_vy,ref_vz,ref_va,ref_vax,ref_vay,ref_vaz,act_vx,act_vy,act_vz,act_va,act_vax,act_vay,act_vaz,tip1,tip2,tip3,tip4,tip5,tip6,axis1,axis2,axis3,axis4,axis5,axis6");
            //DataWriter.AppendFormat("time,ref_rx,ref_ry,ref_rz,ref_ra,ref_rax,ref_ray,ref_raz,act_rx,act_ry,act_rz,act_ra,act_rax,act_ray,act_raz,act_vx,act_vy,act_vz,act_va,act_vax,act_vay,act_vaz,refa1,refa2,refa3,refa4,refa5,refa6,axis1,axis2,axis3,axis4,axis5,axis6,err1,err2,err3,err4,err5,err6;");
            try
            {
                Datafile = new StreamWriter(dataWriterFile + ".csv");
            }
            catch (Exception)
            {
                dataWriterFile = dataWriterFile + Guid.NewGuid().ToString("N");
                Datafile = new StreamWriter(dataWriterFile + ".csv");
            }
            Datafile.WriteLine(DataWriter);
            DataWriter.Clear();
            Datafile.Flush();
            Datafile.Close();

            _DataTime.Start();
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

        public double[] getControllerErrort(Pose referencePosition, double[] measuredAngles,Pose measuredPosition,Pose measuredVelocity, RobotInfo robot)
        {
            double[] referenceAngles = robot.IKSolver(referencePosition, robot.EndEffector, measuredAngles, ref robot._elbow, ref robot._base);
            double[] controlAngles = new double[referenceAngles.Length];
            for (int i = 0; i < referenceAngles.Length; i++)
            {
                controlAngles[i] = (referenceAngles[i] - measuredAngles[i])*P;
            }
            SF.updateDataFile(referencePosition, measuredPosition, measuredVelocity, _DataTime.Elapsed.TotalMilliseconds, referenceAngles, controlAngles,measuredAngles, DataWriter);
            using (StreamWriter Datafile = new StreamWriter(dataWriterFile + ".csv", true))
            {
                Datafile.WriteLine(DataWriter);
            }
            DataWriter.Clear();
            return controlAngles;
        }

        public double[] getControllerEffort(Pose referencePosition, Pose referenceVelocity, Pose measuredPosition, Pose measuredVelocity,double[,] inverseJoc, RobotInfo robot)
        {
            Vector3 ErrorTranslation = referencePosition.Translation - measuredPosition.Translation ;
            //Vector3 ErrorOrientation = SF.getOrientationError(Matrix.CreateFromQuaternion(referencePosition.Orientation), Matrix.CreateFromQuaternion(measuredPosition.Orientation));
            Vector3 ErrorOrientation = SF.getOrientationError(referencePosition.Orientation, measuredPosition.Orientation);
            Vector3 ControlTranslation = referenceVelocity.Translation + Vector3.Multiply(ErrorTranslation, (float)P);
            Vector3 ControlOrientation = Vector3.Multiply(referenceVelocity.axis, referenceVelocity.angle) +Vector3.Multiply(ErrorOrientation, (float)P/200);
            // TODO: write PI controller, may need karman filter for noise
            //double JacTimer = R.IPOC.Elapsed.TotalMilliseconds;
            //Mat Jac = new Mat(Jacobian);
           // R.P3 = R.IPOC.Elapsed.TotalMilliseconds - JacTimer;
            double[] TipVeloicty = new double[] { ControlTranslation.X, ControlTranslation.Y, ControlTranslation.Z, ControlOrientation.X, ControlOrientation.Y, ControlOrientation.Z };
            double[] AxisSpeed =  SF.multiplyMatrix(inverseJoc, TipVeloicty);
            double[] ReferenceAngle = robot.IKSolver(referencePosition);
            SF.updateDataFile(referencePosition, referenceVelocity, measuredPosition, measuredVelocity, _DataTime.Elapsed.TotalMilliseconds, ReferenceAngle, robot.currentAxisAngle, DataWriter);
            using ( StreamWriter Datafile = new StreamWriter(dataWriterFile + ".csv", true))
            {
                Datafile.WriteLine(DataWriter);
            }
            DataWriter.Clear();
            return AxisSpeed;
        }
    }
}
