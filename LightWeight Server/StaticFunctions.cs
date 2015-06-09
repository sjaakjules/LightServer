using Microsoft.Xna.Framework;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace LightWeight_Server
{
    public static class StaticFunctions
    {
        /// <summary>
        /// Creates a quaternion from Kuka coordinates, ABC
        /// </summary>
        public static Quaternion MakeQuaternion(double[] poses)
        {
            Matrix Rz = Matrix.CreateRotationZ((float)poses[3]);
            Matrix Ry = Matrix.CreateRotationY((float)poses[4]);
            Matrix Rx = Matrix.CreateRotationX((float)poses[5]);
            Matrix Rotation = Matrix.Multiply(Matrix.Multiply(Rz, Ry), Rx);
            return Quaternion.CreateFromRotationMatrix(Rotation);
        }
    }


}
