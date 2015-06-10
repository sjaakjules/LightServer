using Microsoft.Xna.Framework;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace LightWeight_Server
{
    public static class StaticFunctions
    {

        static String[] cardinalKeys = new String[] {"X","Y","Z","A","B","C"};
        static String[] axisKeys = new String[] { "A1", "A2", "A3", "A4", "A5", "A6" };

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

        public static double Getvalue(ConcurrentDictionary<string, double> dictionary, String key)
        {
            double outValue = 0;
            bool hasGotValule = false;
            while (!hasGotValule)
            {
                hasGotValule = dictionary.TryGetValue(key, out outValue);
            }
            return outValue;
        }

        public static String getCardinalKey(int Index)
        {
            return cardinalKeys[Index];
        }

        public static String getAxisKey(int Index)
        {
            return cardinalKeys[Index];
        }
    }



}
