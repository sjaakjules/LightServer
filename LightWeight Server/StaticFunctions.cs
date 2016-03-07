using LightWeight_Server;
using Microsoft.Xna.Framework;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CustomExtensions
{
    public static class doubleArrayExtensions
    {

        public static double[] truncate(this double[] array, int decimals)
        {
            double[] newArray = new double[array.Length];
            for (int i = 0; i < array.Length; i++)
            {
                newArray[i] = Math.Round(array[i], decimals);
            }
            return newArray;
        }

        public static double[] Copy(this double[] array)
        {
            double[] newArray = new double[array.Length];
            for (int i = 0; i < array.Length; i++)
            {
                newArray[i] = array[i];
            }
            return newArray;
        }

        public static double[] getRadian(this double[] array)
        {
            double[] newArray = new double[array.Length];
            for (int i = 0; i < array.Length; i++)
            {
                newArray[i] = 1.0 * array[i] * Math.PI / 180;
            }
            return newArray;
        }

        public static double[] getDegree(this double[] array)
        {
            double[] newArray = new double[array.Length];
            for (int i = 0; i < array.Length; i++)
            {
                newArray[i] = 180.0 * array[i] / Math.PI;
            }
            return newArray;
        }

        public static bool isClose(this double[] A1, double[] A2)
        {
            if (A1.Length != A2.Length)
                throw new Exception("Array length not same");
            for (int i = 0; i < A1.Length; i++)
            {
                if (Math.Abs(A1[i] - A2[i]) > 1e-1)
                {
                    return false;
                }
            }
            return true;
        }

        static public double[] createArray(this double value, int length)
        {
            double[] arrayout = new double[length];
            for (int i = 0; i < length; i++)
            {
                arrayout[i] = value;
            }
            return arrayout;
        }


        static public double[] add(this double[] a1, double[] a2)
        {
            if (a1.Length != a2.Length)
                throw new Exception("Double arrays are not same length");
            double[] arrayout = new double[a2.Length];
            for (int i = 0; i < a1.Length; i++)
            {
                arrayout[i] = a1[i] + a2[i];
            }
            return arrayout;
        }


        static public double[] subtract(this double[] a1, double[] a2)
        {
            if (a1.Length != a2.Length)
                throw new Exception("Double arrays are not same length");
            double[] arrayout = new double[a2.Length];
            for (int i = 0; i < a1.Length; i++)
            {
                arrayout[i] = a1[i] - a2[i];
            }
            return arrayout;
        }

        public static double[] multiply(this double[] M, double value)
        {
            double[] Mout = new double[M.Length];
            for (int i = 0; i < M.Length; i++)
            {
                Mout[i] = M[i] * value;
            }
            return Mout;
        }

        public static double multiply(this double[] V1, double[] V2)
        {
            int m_iRows = V1.Length;
            int m_iCols = V2.Length;
            if (m_iCols != m_iRows)
                throw new Exception("Invalid Matrix specified, not correct size");
            double product = 0;
            for (int i = 0; i < m_iRows; i++)
            {
                product += V1[i] * V2[i];
            }
            return product;
        }


        public static double dotProduct(this double[] V1, double[] V2)
        {
            int m_iRows = V1.Length;
            int m_iCols = V2.Length;
            if (m_iCols != m_iRows)
                throw new Exception("Invalid Matrix specified, not correct size");
            double product = 0;
            for (int i = 0; i < m_iRows; i++)
            {
                product += V1[i] * V2[i];
            }
            return product;
        }

        public static string toStringVector(this double[] Array)
        {
            StringBuilder textOut = new StringBuilder();
            textOut.Append("(");
            for (int i = 0; i < Array.Length; i++)
            {
                textOut.Append((180.0 * Array[i] / Math.PI) + ",");
            }
            textOut.Append(")");
            return textOut.ToString();
        }
    }

    public static class doubleMatrixExtensions
    {
        public static double[,] NullMatrix(double[,] mat, int row, int col)
        {
            double[,] result = new double[row, col];
            for (int i = 0; i < row; i++)
            {
                for (int j = 0; j < col; j++)
                {
                    result[i, j] = 0;
                }
            }
            return result;
        }

        /// <summary>
        /// Internal Fucntions for the above operators
        /// </summary>
        public static double[,] Negate(this double[,] matrix)
        {
            return matrix.Multiply(-1);
        }

        public static double[,] Add(this double[,] matrix1, double[,] matrix2)
        {
            if (matrix1.GetLength(0) != matrix2.GetLength(0) || matrix1.GetLength(1) != matrix2.GetLength(1))
                throw new Exception("Invalid Matrix specified, not correct size");
            double[,] result = new double[matrix1.GetLength(0), matrix1.GetLength(1)];
            for (int i = 0; i < result.GetLength(0); i++)
                for (int j = 0; j < result.GetLength(1); j++)
                    result[i, j] = matrix1[i, j] + matrix2[i, j];
            return result;
        }

        public static double[,] Multiply(this double[,] matrix1, double[,] matrix2)
        {
            if (matrix1.GetLength(1) != matrix2.GetLength(0))
                throw new Exception("Invalid Matrix specified, not correct size");
            double[,] result = new double[matrix1.GetLength(0), matrix2.GetLength(1)];
            for (int i = 0; i < result.GetLength(0); i++)
                for (int j = 0; j < result.GetLength(1); j++)
                    for (int k = 0; k < matrix1.GetLength(1); k++)
                        result[i, j] += matrix1[i, k] * matrix2[k, j];
            return result;
        }


        public static double[] Multiply(this double[,] matrix1, double[] colMatrix)
        {
            int m_iRows = matrix1.GetLength(0);
            int m_iCols = matrix1.GetLength(1);
            if (m_iCols != colMatrix.Length)
                throw new Exception("Invalid Matrix specified, not correct size");
            double[] result = new double[m_iRows];
            for (int i = 0; i < m_iRows; i++)
            {
                for (int j = 0; j < m_iCols; j++)
                {
                    result[i] += matrix1[i, j] * colMatrix[j];
                }
            }
            return result;
        }

        public static double[,] Multiply(this double[,] matrix, int iNo)
        {
            double[,] result = new double[matrix.GetLength(0), matrix.GetLength(1)];
            for (int i = 0; i < matrix.GetLength(0); i++)
                for (int j = 0; j < matrix.GetLength(1); j++)
                    result[i, j] = matrix[i, j] * iNo;
            return result;
        }

        public static double[,] Multiply(this double[,] matrix, double frac)
        {
            double[,] result = new double[matrix.GetLength(0), matrix.GetLength(1)];
            for (int i = 0; i < matrix.GetLength(0); i++)
                for (int j = 0; j < matrix.GetLength(1); j++)
                    result[i, j] = matrix[i, j] * frac;
            return result;
        }



        /// <summary>
        /// The function return the Minor of element[Row,Col] of a Matrix object 
        /// </summary>
        public static double[,] Minor(this double[,] matrix, int iRow, int iCol)
        {
            double[,] minor = new double[matrix.GetLength(0) - 1, matrix.GetLength(1) - 1];
            int m = 0, n = 0;
            for (int i = 0; i < matrix.GetLength(0); i++)
            {
                if (i == iRow)
                    continue;
                n = 0;
                for (int j = 0; j < matrix.GetLength(1); j++)
                {
                    if (j == iCol)
                        continue;
                    minor[m, n] = matrix[i, j];
                    n++;
                }
                m++;
            }
            return minor;
        }

        /// <summary>
        /// The helper function for the above Determinent() method
        /// it calls itself recursively and computes determinent using minors
        /// </summary>
        public static double Determinent(this double[,] matrix)
        {
            double det = 0;
            if (matrix.GetLength(0) != matrix.GetLength(1))
                throw new Exception("Determinent of a non-square matrix doesn't exist");
            if (matrix.GetLength(0) == 1)
                return matrix[0, 0];
            for (int j = 0; j < matrix.GetLength(1); j++)
                det += (matrix[0, j] * Determinent(matrix.Minor(0, j)) * (int)System.Math.Pow(-1, 0 + j));
            return det;
        }


        /// <summary>
        /// The function returns the inverse of the current matrix in the traditional way(by adjoint method)
        /// It can be much slower if the given matrix has order greater than 6
        /// Try using InverseFast() function if the order of matrix is greater than 6
        /// </summary>
        public static double[,] Inverse(this double[,] matrix)
        {
            if (matrix.Determinent() == 0)
                throw new Exception("Inverse of a singular matrix is not possible");
            return (matrix.Adjoint().Multiply(1.0 / matrix.Determinent()));
        }


        public static double[,] PsuInverse(this double[,] matrix)
        {
            return matrix.Transpose().Multiply(matrix.Multiply(matrix.Transpose()).Inverse(1e3));
        }



        /// <summary>
        /// The function returns the inverse of the current matrix in the traditional way(by adjoint method)
        /// It can be much slower if the given matrix has order greater than 6
        /// Try using InverseFast() function if the order of matrix is greater than 6
        /// </summary>
        public static double[,] Inverse(this double[,] matrix, double error)
        {
            double det = matrix.Determinent();
            if (det < error)
            {
                return matrix.Adjoint().Multiply(1.0 / error);
            }
            return matrix.Adjoint().Multiply(1.0 / det);
        }

        /// <summary>
        /// The function returns the adjoint of the current matrix
        /// </summary>
        public static double[,] Adjoint(this double[,] matrix)
        {
            if (matrix.GetLength(0) != matrix.GetLength(1))
                throw new Exception("Adjoint of a non-square matrix does not exists");
            double[,] AdjointMatrix = new double[matrix.GetLength(0), matrix.GetLength(1)];
            for (int i = 0; i < matrix.GetLength(0); i++)
                for (int j = 0; j < matrix.GetLength(1); j++)
                    AdjointMatrix[i, j] = Math.Pow(-1, i + j) * (Minor(matrix, i, j).Determinent());
            AdjointMatrix = AdjointMatrix.Transpose();
            return AdjointMatrix;
        }

        /// <summary>
        /// The function returns the transpose of the current matrix
        /// </summary>
        public static double[,] Transpose(this double[,] matrix)
        {
            double[,] TransposeMatrix = new double[matrix.GetLength(1), matrix.GetLength(0)];
            for (int i = 0; i < TransposeMatrix.GetLength(0); i++)
                for (int j = 0; j < TransposeMatrix.GetLength(1); j++)
                    TransposeMatrix[i, j] = matrix[j, i];
            return TransposeMatrix;
        }

        public static double[] row(this double[,] M, int row)
        {
            int m_iRows = M.GetLength(0);
            int m_iCols = M.GetLength(1);
            if (m_iRows < row)
                throw new Exception("Invalid Matrix specified, not correct size");
            double[] Rout = new double[m_iCols];
            for (int i = 0; i < m_iCols; i++)
            {
                Rout[i] = M[row, i];
            }
            return Rout;
        }

        public static double[] column(this double[,] M, int col)
        {
            int m_iRows = M.GetLength(0);
            int m_iCols = M.GetLength(1);
            if (m_iCols < col)
                throw new Exception("Invalid Matrix specified, not correct size");
            double[] Cout = new double[m_iRows];
            for (int i = 0; i < m_iRows; i++)
            {
                Cout[i] = M[i, col];
            }
            return Cout;
        }
    }

    public static class MatrixExtensions
    {
        public static string ToDataString(this Vector3 vec)
        {
            return string.Format("{0},{1},{2}", vec.X, vec.Y, vec.Z);
        }

        public static Vector3 getAxis(this Matrix rotation, string column)
        {
            if (column.Equals("X", StringComparison.OrdinalIgnoreCase))
            {
                return new Vector3(rotation.M11, rotation.M12, rotation.M13);
            }
            else if (column.Equals("Y", StringComparison.OrdinalIgnoreCase))
            {
                return new Vector3(rotation.M21, rotation.M22, rotation.M23);
            }
            else if (column.Equals("Z", StringComparison.OrdinalIgnoreCase))
            {
                return new Vector3(rotation.M31, rotation.M32, rotation.M33);
            }
            else
                throw new Exception("Matrix string did not match X, Y, Z");
        }


        public static Matrix multiply(this Matrix mat1, Matrix mat2)
        {
            return Matrix.Transpose(Matrix.Transpose(mat1) * Matrix.Transpose(mat2));
        }

        public static Matrix multiply(this Matrix Rz, Matrix Ry, Matrix Rx)
        {
            return (Rz.multiply(Ry)).multiply(Rx);
        }

    }

    public static class VectorExtensions
    {

        public static Vector3 getOrientationError(this Vector3 error, Matrix reference, Matrix measured)
        {
            return Vector3.Multiply((Vector3.Cross(measured.getAxis("x"), reference.getAxis("x")) + Vector3.Cross(measured.getAxis("y"), reference.getAxis("y")) + Vector3.Cross(measured.getAxis("z"), reference.getAxis("z"))), 0.5f);
        }


        public static Vector3 getOrientationError(this Vector3 error, Quaternion reference, Quaternion measured)
        {
            Vector3 changeAxis = Vector3.Zero;
            float changeAngle = 0;
            Quaternion changeOrientation = Quaternion.Inverse(measured) * reference;
            changeOrientation.toAxisAngle(out changeAxis, out changeAngle);
            return Vector3.Transform(changeAxis, measured) * changeAngle;
        }
    }

    public static class QuaternionExtensions
    {

        public static Quaternion createFromXaxisZaxis(this Quaternion Q, Vector3 xAxis, Vector3 zAxis)
        {
            xAxis.Normalize();
            zAxis.Normalize();
            Vector3 yAxis = Vector3.Cross(zAxis, xAxis);
            yAxis.Normalize();
            return Quaternion.CreateFromRotationMatrix(new Matrix(xAxis.X, xAxis.Y, xAxis.Z, 0, yAxis.X, yAxis.Y, yAxis.Z, 0, zAxis.X, zAxis.Y, zAxis.Z, 0, 0, 0, 0, 1));
        }

        public static Quaternion createFromZaxis(this Quaternion Q, Vector3 Zaxis, Quaternion _currentOrientation)
        {
            Matrix _currentPose = Matrix.CreateFromQuaternion(_currentOrientation);
            Vector3 axis = Vector3.Cross(Vector3.Normalize(_currentPose.Backward), Vector3.Normalize(Zaxis));
            float angle = (float)Math.Asin((double)axis.Length());
            if (Math.Abs(angle) < MathHelper.ToRadians(0.2f))
            {
                return _currentOrientation;
            }
            if (Vector3.Transform(Zaxis, Quaternion.Inverse(_currentOrientation)).Z < 0)
            {
                angle = Math.Sign(angle) * ((float)Math.PI - Math.Abs(angle));
            }
            return Quaternion.CreateFromAxisAngle(Vector3.Normalize(axis), angle) * _currentOrientation;
        }

        public static bool isOrientationAligned(this Quaternion Q1, Quaternion Q2, double error)
        {
            float angle;
            Vector3 axis;
            Quaternion change = Quaternion.Inverse(Q1) * Q2;
            change.toAxisAngle(out axis, out angle);
            if (Math.Abs(angle) <= error)
            {
                return true;
            }
            return false;
        }


        public static void toAxisAngle(this Quaternion quaternion, out Vector3 outAxis, out float outAngle)
        {
            quaternion.Normalize();
            if (quaternion.W < 0)
            {
                quaternion = Quaternion.Negate(quaternion);
            }
            float angle = 2 * (float)Math.Acos(quaternion.W);
            float s = (float)Math.Sin(1.0 * angle / 2);
            if (Math.Abs(s) < 1e-6)
            {
                outAxis.X = quaternion.X;
                outAxis.Y = quaternion.Y;
                outAxis.Z = quaternion.Z;
                if (Single.IsNaN(quaternion.Z))
                {
                    outAxis.Z = 1;
                }
                outAxis.Normalize();
                if (quaternion.X != 0)
                {
                    if (quaternion.X < 0)
                    {
                        outAngle = (float)(2 * Math.Atan2(-(double)quaternion.X, (double)quaternion.W * -outAxis.X));
                    }
                    else
                    {
                        outAngle = (float)(2 * Math.Atan2((double)quaternion.X, (double)quaternion.W * outAxis.X));
                    }
                }
                else if (quaternion.Y != 0)
                {
                    if (quaternion.Y < 0)
                    {
                        outAngle = (float)(2 * Math.Atan2(-(double)quaternion.Y, (double)quaternion.W * -outAxis.Y));
                    }
                    else
                    {
                        outAngle = (float)(2 * Math.Atan2((double)quaternion.Y, (double)quaternion.W * outAxis.Y));
                    }
                }
                else if (quaternion.Z != 0)
                {
                    if (quaternion.Z < 0)
                    {
                        outAngle = (float)(2 * Math.Atan2(-(double)quaternion.Z, (double)quaternion.W * -outAxis.Z));
                    }
                    else
                    {
                        outAngle = (float)(2 * Math.Atan2((double)quaternion.Z, (double)quaternion.W * outAxis.Z));
                    }
                }
                else
                {
                    outAngle = 0;
                    outAxis = new Vector3(0, 0, 1);
                }

            }
            else
            {
                Vector3 axis = new Vector3(1.0f * quaternion.X / s, 1.0f * quaternion.Y / s, 1.0f * quaternion.Z / s);
                axis.Normalize();
                s = quaternion.X == 0 ? s : 1.0f * quaternion.X / axis.X;
                s = quaternion.Y == 0 ? s : 1.0f * quaternion.Y / axis.Y;
                s = quaternion.Z == 0 ? s : 1.0f * quaternion.Z / axis.Z;

                if (float.IsNaN(quaternion.Z))
                {
                    outAxis.Z = 1;
                }
                outAxis = new Vector3(1.0f * quaternion.X / s, 1.0f * quaternion.Y / s, 1.0f * quaternion.Z / s);
                outAxis.Normalize();
                if (quaternion.X != 0)
                {
                    if (quaternion.X < 0)
                    {
                        outAngle = (float)(2 * Math.Atan2(-(double)quaternion.X, (double)quaternion.W * -outAxis.X));
                    }
                    else
                    {
                        outAngle = (float)(2 * Math.Atan2((double)quaternion.X, (double)quaternion.W * outAxis.X));
                    }
                }
                else if (quaternion.Y != 0)
                {
                    if (quaternion.Y < 0)
                    {
                        outAngle = (float)(2 * Math.Atan2(-(double)quaternion.Y, (double)quaternion.W * -outAxis.Y));
                    }
                    else
                    {
                        outAngle = (float)(2 * Math.Atan2((double)quaternion.Y, (double)quaternion.W * outAxis.Y));
                    }
                }
                else if (quaternion.Z != 0)
                {
                    if (quaternion.Z < 0)
                    {
                        outAngle = (float)(2 * Math.Atan2(-(double)quaternion.Z, (double)quaternion.W * -outAxis.Z));
                    }
                    else
                    {
                        outAngle = (float)(2 * Math.Atan2((double)quaternion.Z, (double)quaternion.W * outAxis.Z));
                    }
                }
                else
                {
                    outAngle = 0;
                }
            }
        }


    }

    public static class MiskExtensions
    {
        public static double[] getLatest(this FilterButterworth[] filterValues)
        {
            double[] filtValues = new double[filterValues.Length];
            for (int i = 0; i < filterValues.Length; i++)
            {
                filtValues[i] = filterValues[i].Value;
            }
            return filtValues;
        }

        public static void updateValues(this FilterButterworth[] doubleFilter, double[] newValues)
        {
            if (newValues.Length == doubleFilter.Length)
            {
                for (int i = 0; i < doubleFilter.Length; i++)
                {
                    doubleFilter[i].Update((float)newValues[i]);
                }
            }
            else
            {
                throw new Exception("Filter length not equal to value array.");
            }
        }
    }
}

namespace LightWeight_Server
{
    // CONVENTIONS of Matrix class:
    // Identity matrix. Forwards =   0, 0,-1
    //                  Down     =   0,-1, 0
    //                  Left     =  -1, 0, 0
    //                  Xaxis    =   Right
    //                  Yaxis    =   Up
    //                  Zaxis    =   Backwards
    //
    ///
    // Matrix are row basis, where litrature is column basis!
    // This requires transpose before and after any multiplication

    public enum ElbowPosition { up, down, stretched };
    public enum BasePosition { front, back, vertical };


    public struct Pose : IFormattable
    {
        double _x, _y, _z;
        float _angle;
        Quaternion _Orientation;
        Vector3 _axis;
        double[] _kukaValues;

        public static Pose Zero { get { return new Pose(Quaternion.Identity, Vector3.Zero); } }

        /// <summary>
        /// Loads a new Pose using a string with the infomation of position, velocity and orientation. Can handle partial information.
        /// </summary>
        /// <param name="newPose"></param>
        /// <param name="LastPose"></param>
        public Pose(string[] newPose, Pose LastPose)
        {
            int shift = newPose.Length % 3;
            // The following If statements catch error while trying to load positions, use last position and keep loading.
            // TODO: Add signals and more complex error handeling to send back to external server. Perhaps the string which was received for error checking?
            if (!double.TryParse(newPose[0 + shift], out _x))
            {
                _x = LastPose._x;
            }
            if (!double.TryParse(newPose[1 + shift], out _y))
            {
                _y = LastPose._y;
            }
            if (!double.TryParse(newPose[2 + shift], out _z))
            {
                _z = LastPose._z;
            }
            double[] Axis = new double[newPose.Length - (3 + shift)];
            bool loadedAxis = true;
            for (int i = 3 + shift; i < newPose.Length; i++)
            {
                if (!double.TryParse(newPose[i], out Axis[i - (3 + shift)]))
                {
                    loadedAxis = false;
                    break;
                }
            }
            if (loadedAxis)
            {
                // only Z axis loaded
                if (Axis.Length == 3)
                {
                    this._Orientation = SF.QfromZaxis(new Vector3((float)Axis[0], (float)Axis[1], (float)Axis[2]), LastPose.Orientation);
                    SF.getAxisAngle(_Orientation, out _axis, out _angle);
                    SF.getKukaAngles(_Orientation, out _kukaValues);
                    _kukaValues[0] = (float)_x;
                    _kukaValues[1] = (float)_y;
                    _kukaValues[2] = (float)_z;
                }
                    // Z axis and X axis loaded
                else if (Axis.Length == 6)
                {
                    this._Orientation = SF.QfromXZ(new Vector3((float)Axis[0], (float)Axis[1], (float)Axis[2]), new Vector3((float)Axis[3], (float)Axis[4], (float)Axis[5]));
                    SF.getAxisAngle(_Orientation, out _axis, out _angle);
                    SF.getKukaAngles(_Orientation, out _kukaValues);
                    _kukaValues[0] = (float)_x;
                    _kukaValues[1] = (float)_y;
                    _kukaValues[2] = (float)_z;
                }
                    // No rotation loaded, position movement only
                else
                {
                    this._Orientation = LastPose.Orientation;
                    SF.getAxisAngle(LastPose.Orientation, out _axis, out _angle);
                    SF.getKukaAngles(LastPose.Orientation, out _kukaValues);
                    _kukaValues[0] = (float)_x;
                    _kukaValues[1] = (float)_y;
                    _kukaValues[2] = (float)_z;
                }
            }
                // Failed while trying to load rotaition, use last orientation and keep moving.
                // TODO: Add signals and more complex error handeling to send back to external server. Perhaps the string which was received for error checking?
            else
            {
                this._Orientation = LastPose.Orientation;
                SF.getAxisAngle(LastPose.Orientation, out _axis, out _angle);
                SF.getKukaAngles(LastPose.Orientation, out _kukaValues);
                _kukaValues[0] = (float)_x;
                _kukaValues[1] = (float)_y;
                _kukaValues[2] = (float)_z;
            }
        }


        /// <summary>
        /// Constructor used for commands from an external server.
        /// Loads a new Pose using a string with the infomation of position, velocity and orientation. Can handle partial information.
        /// </summary>
        /// <param name="PositionInfo"></param>
        /// <param name="LastPose"></param>
        /// <param name="TaskspaceRotation"></param> this is the constant rotation of the tool tip
        public Pose(string[] PositionInfo, string[] OrientationInfo, Pose LastPose, Quaternion TaskspaceRotation)
        {
            // The following If statements catch error while trying to load positions, use last position and keep loading.
            // TODO: Add signals and more complex error handeling to send back to external server. Perhaps the string which was received for error checking?
            if (PositionInfo == null || PositionInfo.Length != 3)
            {
                _x = LastPose._x;
                _y = LastPose._y;
                _z = LastPose._z;
            }
            else
            {
                if (!double.TryParse(PositionInfo[0], out _x))
                {
                    _x = LastPose._x;
                }
                if (!double.TryParse(PositionInfo[1], out _y))
                {
                    _y = LastPose._y;
                }
                if (!double.TryParse(PositionInfo[2], out _z))
                {
                    _z = LastPose._z;
                }
            }
            if (OrientationInfo == null)
            {
                this._Orientation = LastPose.Orientation;
                SF.getAxisAngle(LastPose.Orientation, out _axis, out _angle);
                SF.getKukaAngles(LastPose.Orientation, out _kukaValues);
                _kukaValues[0] = (float)_x;
                _kukaValues[1] = (float)_y;
                _kukaValues[2] = (float)_z;
            }
            else
            {
                double[] Axis = new double[OrientationInfo.Length];
                bool loadedAxis = true;
                for (int i = 0; i < OrientationInfo.Length; i++)
                {
                    if (!double.TryParse(OrientationInfo[i], out Axis[i]))
                    {
                        loadedAxis = false;
                        break;
                    }
                }
                if (loadedAxis)
                {
                    // only Z axis loaded
                    if (Axis.Length == 3)
                    {
                        this._Orientation = SF.QfromZaxis(new Vector3((float)Axis[0], (float)Axis[1], (float)Axis[2]), LastPose.Orientation) * TaskspaceRotation;
                        SF.getAxisAngle(_Orientation, out _axis, out _angle);
                        SF.getKukaAngles(_Orientation, out _kukaValues);
                        _kukaValues[0] = (float)_x;
                        _kukaValues[1] = (float)_y;
                        _kukaValues[2] = (float)_z;
                    }
                    // Z axis and X axis loaded
                    else if (Axis.Length == 6)
                    {
                        this._Orientation = SF.QfromXZ(new Vector3((float)Axis[0], (float)Axis[1], (float)Axis[2]), new Vector3((float)Axis[3], (float)Axis[4], (float)Axis[5])) * TaskspaceRotation;
                        SF.getAxisAngle(_Orientation, out _axis, out _angle);
                        SF.getKukaAngles(_Orientation, out _kukaValues);
                        _kukaValues[0] = (float)_x;
                        _kukaValues[1] = (float)_y;
                        _kukaValues[2] = (float)_z;
                    }
                    // No rotation loaded, position movement only
                    else
                    {
                        this._Orientation = LastPose.Orientation;
                        SF.getAxisAngle(LastPose.Orientation, out _axis, out _angle);
                        SF.getKukaAngles(LastPose.Orientation, out _kukaValues);
                        _kukaValues[0] = (float)_x;
                        _kukaValues[1] = (float)_y;
                        _kukaValues[2] = (float)_z;
                    }

                }
                // Failed while trying to load rotaition, use last orientation and keep moving.
                // TODO: Add signals and more complex error handeling to send back to external server. Perhaps the string which was received for error checking?
                else
                {
                    this._Orientation = LastPose.Orientation;
                    SF.getAxisAngle(LastPose.Orientation, out _axis, out _angle);
                    SF.getKukaAngles(LastPose.Orientation, out _kukaValues);
                    _kukaValues[0] = (float)_x;
                    _kukaValues[1] = (float)_y;
                    _kukaValues[2] = (float)_z;
                }
            }
        }

        public Pose(double[] KukaValues)
        {
            _kukaValues = new double[6];
            KukaValues.CopyTo(_kukaValues, 0);
            this._x = KukaValues[0];
            this._y = KukaValues[1];
            this._z = KukaValues[2];
            _Orientation = SF.MakeQuaternionFromKuka(KukaValues);
            SF.getAxisAngle(_Orientation, out _axis, out _angle);
        }

        public Pose(Pose lastPose, Pose newPose, double elapsedTime)
        {
            if (elapsedTime == 0)
            {
                elapsedTime = 4;
            }
            Vector3 translation = newPose.Translation - lastPose.Translation;
            float angle;
            Vector3 axis;
            SF.getAxisAngle(Quaternion.Inverse(lastPose.Orientation) * newPose.Orientation,out axis,out angle);
            _axis = Vector3.Transform(axis, lastPose.Orientation);
            _angle = (float)(1.0 * angle / elapsedTime);
            _x = translation.X / elapsedTime;
            _y = translation.Y / elapsedTime;
            _z = translation.Z / elapsedTime;
            _Orientation = Quaternion.CreateFromAxisAngle(_axis, _angle);
            SF.getKukaAngles(_Orientation, out _kukaValues);
            _kukaValues[0] = (float)_x;
            _kukaValues[1] = (float)_y;
            _kukaValues[2] = (float)_z;
            for (int i = 0; i < _kukaValues.Length; i++)
            {
                 if (double.IsNaN(_kukaValues[i]))
                {
                    _kukaValues[i] = 0;
                }
            }
        }


        public Pose(Matrix Pose) : this(Quaternion.CreateFromRotationMatrix(Pose), Pose.Translation) { }

        public Pose(Quaternion Orientation, Vector3 Position)
        {
            this._x = Position.X;
            this._y = Position.Y;
            this._z = Position.Z;
            this._Orientation = Orientation;
            SF.getAxisAngle(Orientation, out _axis, out _angle);
            SF.getKukaAngles(Orientation, out _kukaValues);
            _kukaValues[0] = (float)_x;
            _kukaValues[1] = (float)_y;
            _kukaValues[2] = (float)_z;
        }

        public Pose(TimeCoordinate Pose) : this(Pose.Orientation, Pose.Translation) { }


        public double[,] getMatrix
        {
            get
            {
                Matrix orientation = Matrix.CreateFromQuaternion(this.Orientation);
                double[,] matrix = new double[,] {  { orientation.M11, orientation.M21, orientation.M31, this.Translation.X }, 
                                                        { orientation.M12, orientation.M22, orientation.M32, this.Translation.Y }, 
                                                        { orientation.M13, orientation.M23, orientation.M33, this.Translation.Z }, { 0, 0, 0, 1 } };
                return matrix;
            }
        }


        public override int GetHashCode()
        {
            return ShiftAndWrap(_x.GetHashCode(), 2) ^ _y.GetHashCode();
        }

        public bool Equals(Pose pose2, double error)
        {
            if (Vector3.Distance(this.Translation, pose2.Translation) <= (error * 4) && SF.isOrientationAligned(this.Orientation, pose2.Orientation, error))
            {
                return true;
            }
            return false;
        }

        private int ShiftAndWrap(int value, int positions)
        {
            positions = positions & 0x1F;

            // Save the existing bit pattern, but interpret it as an unsigned integer.
            uint number = BitConverter.ToUInt32(BitConverter.GetBytes(value), 0);
            // Preserve the bits to be discarded.
            uint wrapped = number >> (32 - positions);
            // Shift and wrap the discarded bits.
            return BitConverter.ToInt32(BitConverter.GetBytes((number << positions) | wrapped), 0);
        }

        public override bool Equals(object o)
        {
            if (Vector3.Distance(this.Translation, ((Pose)o).Translation) == 1e-6 && SF.isOrientationAligned(this.Orientation, ((Pose)o).Orientation, 1e-6))
            {
                return true;
            }
            return false;
        }

        public static bool operator ==(Pose pose1, Pose pose2)
        {
            if (Vector3.Distance( pose1.Translation,pose2.Translation) == 1e-6 && pose1.Orientation.Equals(pose2.Orientation))
            {
                return true;
            }
            return false;
        }

        public static bool operator !=(Pose pose1, Pose pose2)
        {
            if (Vector3.Distance(pose1.Translation, pose2.Translation) == 1e-6 && pose1.Orientation.Equals(pose2.Orientation))
            {
                return false;
            }
            return true;
        }
         

        public Vector3 Translation
        {
            get { return new Vector3((float)_x, (float)_y, (float)_z); }
            set { _x = value.X; _y = value.Y; _z = value.Z; }
        }

        public Quaternion Orientation
        {
            get { return _Orientation; }
            set
            {
                _Orientation = value;
                SF.getAxisAngle(value, out _axis, out _angle);
            }
        }

        public double[] kukaValues
        {
            get { return _kukaValues; }
        }

        public Vector3 zAxis
        {
            get { return Vector3.Transform(new Vector3(0, 0, 1),_Orientation); }
        }


        public Vector3 yAxis
        {
            get { return Vector3.Transform(new Vector3(0, 1, 0), _Orientation); }
        }

        public Vector3 xAxis
        {
            get { return Vector3.Transform(new Vector3(1, 0, 0), _Orientation); }
        }

        public Vector3 Velocity
        {
            get { return _axis * _angle; }
        }

        public Vector3 axis
        {
            get { return _axis; }
        }

        public float angle
        {
            get { return _angle; }
        }

        public static Pose inverse(Pose pose)
        {
            return new Pose(Quaternion.Inverse(pose.Orientation), -Vector3.Transform(pose.Translation, Quaternion.Inverse(pose.Orientation)));
        }

        public string ToString(string format, IFormatProvider formatProvider)
        {

            if (format == null) format = "G";

            if (formatProvider != null)
            {
                ICustomFormatter formatter = formatProvider.GetFormat(this.GetType()) as ICustomFormatter;
                if (formatter != null)
                    return formatter.Format(format, this, formatProvider);
            }

            switch (format)
            {

                case "PosZ": return string.Format("{0},{1},{2},{3},{4},{5}", this._x, this._y, this._z, this.zAxis.X, this.zAxis.Y, this.zAxis.Z);

                case "data": return string.Format("{0},{1},{2},{3},{4},{5},{6}", this._x, this._y, this._z, this.angle, this.axis.X, this.axis.Y, this.axis.Z);

                case "Display": return string.Format("({0:0.000},{1:0.000},{2:0.000},{3:0.000},{4:0.000},{5:0.000})", this._x, this._y, this._z, this._kukaValues[3], this._kukaValues[4], this._kukaValues[5]);

                case "G":

                default: return String.Format("({0,5},{1,5},{2,5})", this._x, this._y, this._z);

            }

        }


        public override string ToString()
        {
            return ToString("G", null);
        }



        public string ToString(string format)
        {
            return ToString(format, null);
        }

        public string ToString(IFormatProvider formatProvider)
        {
            return ToString(null, formatProvider);
        }
        

        public Pose invert
        {
            get { return new Pose(Quaternion.Inverse(this.Orientation), -Vector3.Transform(this.Translation, Quaternion.Inverse(this.Orientation))); }
        }

        public static Vector3 operator *(Pose Pose, Vector3 Position)
        {
            Matrix PoseM = Matrix.Transpose(Matrix.CreateFromQuaternion(Pose._Orientation));
            return new Vector3( (float)Pose._x + PoseM.M11 * Position.X + PoseM.M12 * Position.Y + PoseM.M13 * Position.Z,
                                (float)Pose._y + PoseM.M21 * Position.X + PoseM.M22 * Position.Y + PoseM.M23 * Position.Z,
                                (float)Pose._z + PoseM.M31 * Position.X + PoseM.M32 * Position.Y + PoseM.M33 * Position.Z);
        }
        
        public static Pose operator *(Pose Pose1, Pose Pose2)
        {
            return new Pose(Pose1.Orientation * Pose2.Orientation, Pose1 * Pose2.Translation);
        }
         

    }

    public struct TimeCoordinate
    {
        public readonly long Ipoc;
        public readonly int LocalIpoc;
        public double x, y, z, a, b, c;
        public float angle;
        Quaternion _Orientation;
        public Vector3 axis;
        double[] _kukaValues;

        public TimeCoordinate(double x, double y, double z, double a, double b, double c, long ipoc)
        {
            this.x = x;
            this.y = y;
            this.z = z;
            this.a = a;
            this.b = b;
            this.c = c;
            _kukaValues = new double[] { x, y, z, a, b, c };
            _Orientation = SF.MakeQuaternionFromKuka(a, b, c);
            SF.getAxisAngle(_Orientation, out axis, out angle);
            this.Ipoc = ipoc;
            this.LocalIpoc = Environment.TickCount;
        }

        public TimeCoordinate(Vector3 Position, Quaternion Orientation, long ipoc)
        {
            this.x = Position.X;
            this.y = Position.Y;
            this.z = Position.Z;
            Vector3 angles = SF.getKukaAngles(Orientation);
            this.a = angles.X;
            this.b = angles.Y;
            this.c = angles.Z;
            _kukaValues = new double[] { x, y, z, a, b, c };
            _Orientation = new Quaternion(Orientation.X, Orientation.Y, Orientation.Z, Orientation.W);
            SF.getAxisAngle(Orientation, out axis, out angle);
            this.Ipoc = ipoc;
            this.LocalIpoc = Environment.TickCount;
        }

        public TimeCoordinate(Pose Pose, long ipoc) : this(Pose.Translation, Pose.Orientation, ipoc) { }

        public TimeCoordinate(double x, double y, double z, Quaternion Orientation, long ipoc) : this(new Vector3((float)x, (float)y, (float)z), Orientation, ipoc) { }

        public TimeCoordinate(double x, double y, double z, Vector3 axis, float angle, long ipoc) : this(new Vector3((float)x, (float)y, (float)z), Quaternion.CreateFromAxisAngle(axis, angle), ipoc) { }

        public Vector3 Translation
        {
            get { return new Vector3((float)x, (float)y, (float)z); }
            set { x = value.X; y = value.Y; z = value.Z; }
        }

        public Quaternion Orientation
        {
            get { return _Orientation; }
            set
            {
                _Orientation = value;
                SF.getAxisAngle(value, out axis, out angle);
                KukaAngles = SF.getKukaAngles(Orientation);
            }
        }

        public Vector3 KukaAngles
        {
            get { return new Vector3((float)a, (float)b, (float)c); }
            set
            {
                this.a = value.X;
                this.b = value.Y;
                this.c = value.Z;
            }
        }

        public double[] kukaValues
        {
            get { return _kukaValues; }
        }

        public Pose Pose
        {
            get { return new Pose(Orientation, Translation); }
            set
            {
                Orientation = value.Orientation;
                Translation = value.Translation;
            }
        }

        public static TimeCoordinate getAverage(TimeCoordinate[] list)
        {
            if (list.Length > 1)
            {
                TimeCoordinate poseOut = list[0];
                for (int i = 1; i < list.Length; i++)
                {
                    poseOut = poseOut + list[i];
                }
                poseOut = poseOut / list.Length;
                return poseOut;
            }
            return list[0];
        }

        public static TimeCoordinate operator +(TimeCoordinate Pose1, TimeCoordinate Pose2)
        {
            return new TimeCoordinate(Pose1.x + Pose2.x, Pose1.y + Pose2.y, Pose1.z + Pose2.z, Pose1.axis + Pose2.axis, Pose1.angle + Pose2.angle,Pose2.Ipoc);
        }

        public static TimeCoordinate operator /(TimeCoordinate Pose1, float Value)
        {
            return new TimeCoordinate(1.0f * Pose1.x / Value, 1.0f * Pose1.y / Value, 1.0f * Pose1.z / Value, Vector3.Divide(Pose1.axis, Value), 1.0f * Pose1.angle / Value, Pose1.Ipoc);
        }
        
        public TimeCoordinate getRateOfChange(TimeCoordinate pose1, double framerate)
        {
            Vector3 changeAxis;
            float changeAngle;
            if (this.Ipoc < pose1.Ipoc)
            {
                Quaternion changeOrientation = Quaternion.Inverse(this._Orientation) * pose1._Orientation;
                SF.getAxisAngle(changeOrientation, out changeAxis, out changeAngle);
                return new TimeCoordinate(1.0f * (pose1.x - this.x) / framerate,
                                            1.0f * (pose1.y - this.y) / framerate,
                                            1.0f * (pose1.z - this.z) / framerate,
                                            Vector3.Transform(changeAxis, this._Orientation),
                                            1.0f * changeAngle / (float)framerate,
                                            pose1.Ipoc);
            }
            else
            {
                Quaternion changeOrientation = Quaternion.Inverse(pose1._Orientation) * this._Orientation;
                SF.getAxisAngle(changeOrientation, out changeAxis, out changeAngle);
                return new TimeCoordinate(1.0f * (this.x - pose1.x) / framerate,
                                            1.0f * (this.y - pose1.y) / framerate,
                                            1.0f * (this.z - pose1.z) / framerate,
                                            Vector3.Transform(changeAxis, pose1._Orientation),
                                            1.0f * changeAngle / (float)framerate,
                                            this.Ipoc);
            }
        }


        public TimeCoordinate getRateOfChange(TimeCoordinate pose1)
        {
            Vector3 changeAxis;
            float changeAngle;
            if (this.Ipoc < pose1.Ipoc)
            {
                double delTime = 1.0 * (pose1.Ipoc - this.Ipoc);
                if (delTime == 0)
                {
                    delTime = 1;
                }
                Quaternion changeOrientation = Quaternion.Inverse(this._Orientation) * pose1._Orientation;
                SF.getAxisAngle(changeOrientation, out changeAxis, out changeAngle);
                return new TimeCoordinate(1.0f * (pose1.x - this.x) / delTime,
                                            1.0f * (pose1.y - this.y) / delTime,
                                            1.0f * (pose1.z - this.z) / delTime,
                                            Vector3.Transform(changeAxis,this._Orientation),
                                            1.0f * changeAngle / (float)delTime,
                                            pose1.Ipoc);
            }
            else
            {
                double delTime = 1.0 * (this.Ipoc - pose1.Ipoc) ;
                if (delTime == 0)
                {
                    delTime = 1;
                }
                Quaternion changeOrientation = Quaternion.Inverse(pose1._Orientation) * this._Orientation;
                SF.getAxisAngle(changeOrientation, out changeAxis, out changeAngle);
                return new TimeCoordinate(1.0f * (this.x - pose1.x) / delTime,
                                            1.0f * (this.y - pose1.y) / delTime,
                                            1.0f * (this.z - pose1.z) / delTime,
                                            Vector3.Transform(changeAxis, pose1._Orientation),
                                            1.0f * changeAngle / (float)delTime,
                                            this.Ipoc);
            }
        }

        public void AxisAngle(Vector3 newAxis, float newAngle)
        {
            this.axis = newAxis;
            this.angle = newAngle;
            this._Orientation = Quaternion.CreateFromAxisAngle(axis, angle);
        }

        /// <summary>
        /// Finds the angle in radians to transform paseB towards this pose.
        /// </summary>
        /// <param name="poseB"></param>
        /// <returns></returns>
        public float OrientationDisplacementBtoA(TimeCoordinate poseB)
        {
            Quaternion tempOrientation = this._Orientation * Quaternion.Inverse(poseB.Orientation);
            Vector3 axis;
            float angle;
            SF.getAxisAngle(tempOrientation, out axis, out angle);
            return angle;
        }

        /// <summary>
        /// Finds the angle in radians to transform this pose towards paseB.
        /// </summary>
        /// <param name="poseB"></param>
        /// <returns></returns>
        public float OrientationDisplacementAtoB(TimeCoordinate poseB)
        {
            Quaternion tempOrientation = Quaternion.Inverse(this._Orientation) * poseB.Orientation;
            Vector3 axis;
            float angle;
            SF.getAxisAngle(tempOrientation, out axis, out angle);
            return angle;
        }
    }

    public class FixedSizedQueue<T> : ConcurrentQueue<T>
    {
        private readonly object syncObject = new object();

        public int Size { get; private set; }

        public FixedSizedQueue(int size)
        {
            Size = size;
        }

        public new void Enqueue(T obj)
        {
            base.Enqueue(obj);
            lock (syncObject)
            {
                while (base.Count > Size)
                {
                    T outObj;
                    base.TryDequeue(out outObj);
                }
            }
        }

        public T LastElement
        {

            get
            {
                lock (syncObject)
                {
                    T[] list = base.ToArray();
                    return list[list.Length - 1];
                }
            }
        }

        public T[] ThreadSafeToArray
        {
            get
            {
                lock (syncObject)
                {
                    return base.ToArray();
                }
            }
        }


    }

    #region Personalised Exceptions

    /// <summary>
    /// Exception class for Matrix class, derived from System.Exception
    /// </summary>
    public class MatrixException : Exception
    {
        public MatrixException()
            : base()
        { }

        public MatrixException(string Message)
            : base(Message)
        { }

        public MatrixException(string Message, Exception InnerException)
            : base(Message, InnerException)
        { }
    }

    /// <summary>
    /// Exception class for Kuka related functions, derived from System.Exception
    /// </summary>
    public class KukaException : Exception
    {
        public KukaException()
            : base()
        { }

        public KukaException(string Message)
            : base(Message)
        { }

        public KukaException(string Message, Exception InnerException)
            : base(Message, InnerException)
        { }
    }

    /// <summary>
    /// Exception class for general orientation and rotation related functions, derived from System.Exception
    /// </summary>
    public class OrientationException : Exception
    {
        public OrientationException()
            : base()
        { }

        public OrientationException(string Message)
            : base(Message)
        { }

        public OrientationException(string Message, Exception InnerException)
            : base(Message, InnerException)
        { }
    }

    /// <summary>
    /// Exception class for general orientation and rotation related functions, derived from System.Exception
    /// </summary>
    public class TrajectoryException : Exception
    {
        public TrajectoryException()
            : base()
        { }

        public TrajectoryException(string Message)
            : base(Message)
        { }

        public TrajectoryException(string Message, Exception InnerException)
            : base(Message, InnerException)
        { }
    }



    /// <summary>
    /// Exception class for general orientation and rotation related functions, derived from System.Exception
    /// </summary>
    public class InverseKinematicsException : Exception
    {
        public InverseKinematicsException()
            : base()
        { }

        public InverseKinematicsException(string Message)
            : base(Message)
        { }

        public InverseKinematicsException(string Message, Exception InnerException)
            : base(Message, InnerException)
        { }
    }

    #endregion



    /// <summary>
    /// Filter with tests showing  samplerate 250, res sqrt2,freq 10
    /// </summary>
    public class FilterButterworth
    {
        /// <summary>
        /// rez amount, from sqrt(2) to ~ 0.1
        /// </summary>
        private readonly float resonance;

        private readonly float frequency;
        private readonly int sampleRate;
        private readonly PassType passType;

        private readonly float c, a1, a2, a3, b1, b2;

        /// <summary>
        /// Array of input values, latest are in front
        /// </summary>
        private float[] inputHistory = new float[2];

        /// <summary>
        /// Array of output values, latest are in front
        /// </summary>
        private float[] outputHistory = new float[3];

        /// <summary>
        /// Creates a filter object which is used to add new values at the given sampleRate and frequency.
        /// </summary>
        /// <param name="frequency"></param> Frequency, for 4ms data 10 is good
        /// <param name="sampleRate"></param> SampleRate, for 4ms data 250 is good
        /// <param name="passType"></param> Pass type either low or high, to filter jitters use lowpass
        /// <param name="resonance"></param> Resonance, from sqrt(2) to ~ 0.1, for 4ms data sqrt(2) was good
        public FilterButterworth(float frequency, int sampleRate, PassType passType, float resonance)
        {
            this.resonance = resonance;
            this.frequency = frequency;
            this.sampleRate = sampleRate;
            this.passType = passType;

            switch (passType)
            {
                case PassType.Lowpass:
                    c = 1.0f / (float)Math.Tan(Math.PI * frequency / sampleRate);
                    a1 = 1.0f / (1.0f + resonance * c + c * c);
                    a2 = 2f * a1;
                    a3 = a1;
                    b1 = 2.0f * (1.0f - c * c) * a1;
                    b2 = (1.0f - resonance * c + c * c) * a1;
                    break;
                case PassType.Highpass:
                    c = (float)Math.Tan(Math.PI * frequency / sampleRate);
                    a1 = 1.0f / (1.0f + resonance * c + c * c);
                    a2 = -2f * a1;
                    a3 = a1;
                    b1 = 2.0f * (c * c - 1.0f) * a1;
                    b2 = (1.0f - resonance * c + c * c) * a1;
                    break;
            }
        }



        public enum PassType
        {
            Highpass,
            Lowpass,
        }

        public void Update(float newInput)
        {
            float newOutput = a1 * newInput + a2 * this.inputHistory[0] + a3 * this.inputHistory[1] - b1 * this.outputHistory[0] - b2 * this.outputHistory[1];

            this.inputHistory[1] = this.inputHistory[0];
            this.inputHistory[0] = newInput;

            this.outputHistory[2] = this.outputHistory[1];
            this.outputHistory[1] = this.outputHistory[0];
            this.outputHistory[0] = newOutput;
        }

        public float Value
        {
            get { return this.outputHistory[0]; }
        }
    }

    public static class SF
    {
        #region Printing Functions
        /// <summary>
        /// Returns a string in CSV format. Will break if a null array is passed.
        /// Format returned looks like: [#,#,#] where # represent double values with 
        /// 7 trailing decimals and there is no splitter before or after the first and last number respectively
        /// </summary>
        /// <param name="array"></param> Array of >1 size to return as strings in CSV format.
        /// <returns></returns>
        public static string printDouble(double[] array)
        {
            StringBuilder printingstring = new StringBuilder();
            printingstring.Append(string.Format("{0:0.0000000}", array[0]));
            for (int i = 1; i < array.Length; i++)
            {
                printingstring.Append(string.Format(",{0:0.0000000}", array[i]));
            }
            return printingstring.ToString();
        }

        public static void updateDataFile(Pose refPos, Pose refVel, Pose actPos, Pose actVel, double time, double[] controlAngles, double[] satControl, double[] IKmethod,StringBuilder tableRow)
        {
            tableRow.Append(string.Format("{0:0.0},{1:data},{2:data},{3:data},{4:data},{5},{6},{7}", time, refPos, actPos, refVel, actVel, printDouble(controlAngles), printDouble(satControl), printDouble(IKmethod)));
        }

        internal static void updateDataFile(Pose referencePosition, Pose measuredPosition, Pose measuredVelocity, double time, double[] referenceAngles, double[] controlAngles, double[] measuredangles, StringBuilder DataWriter)
        {
            DataWriter.Append(string.Format("{0:0.0},{1:data},{2:data},{3:data},{4},{5},{6};", time, referencePosition, measuredPosition, measuredVelocity, printDouble(referenceAngles), printDouble(referenceAngles), printDouble(controlAngles)));
        }

        #endregion

        #region Matrix Functions

        public static void plotDoubles(double[] array, string Heading)
        {
            if (Heading.Length < 24)
            {
                string newheading = string.Concat(Heading, ":", new string(' ', 24 - Heading.Length));
            }
            Console.Write(String.Format("{0,-25}\t (", Heading));
            for (int i = 0; i < array.Length; i++)
            {
                Console.Write(String.Format(" {0:0.0} ,", array[i]));
            }
            Console.WriteLine(")");
        }

        public static string DoublesToString(double[] array)
        {
            StringBuilder strOut = new StringBuilder();
            strOut.Append("(");
            for (int i = 0; i < array.Length; i++)
            {
                strOut.Append(String.Format(" {0:0.0} ,", array[i]));
            }
            strOut.AppendLine(")");
            return strOut.ToString();
        }

        public static double[] getRadian(double[] array)
        {
            double[] newArray = new double[array.Length];
            for (int i = 0; i < array.Length; i++)
            {
                newArray[i] = 1.0 * array[i] * Math.PI / 180;
            }
            return newArray;
        }

        public static double[] getDegree(double[] array)
        {
            double[] newArray = new double[array.Length];
            for (int i = 0; i < array.Length; i++)
            {
                newArray[i] = 180.0 * array[i] / Math.PI;
            }
            return newArray;
        }

        public static bool IsClose(double[] A1, double[] A2)
        {
            for (int i = 0; i < A1.Length; i++)
            {
                if (Math.Abs(A1[i] - A2[i]) > 1e-1)
                {
                    return false;
                }
            }
            return true;
        }

        static public double[] createDouble(double value, int length)
        {
            double[] arrayout = new double[length];
            for (int i = 0; i < length; i++)
            {
                arrayout[i] = value;
            }
            return arrayout;
        }

        static public void addtoDoubles(double[] a1, double[] a2)
        {
            if (a1.Length != a2.Length)
            {
                throw new MatrixException("Double arrays are not same length");
            }
            for (int i = 0; i < a1.Length; i++)
            {
                a1[i] += a2[i];
            }
        }

        static public double[] addDoubles(double[] a1, double[] a2)
        {
            if (a1.Length != a2.Length)
            {
                throw new MatrixException("Double arrays are not same length");
            }
            double[] arrayout = new double[a2.Length];
            for (int i = 0; i < a1.Length; i++)
            {
                arrayout[i] = a1[i] + a2[i];
            }
            return arrayout;
        }

        public static double[] multiplyMatrix(double[] M, double value)
        {
            double[] Mout = new double[M.Length];
            for (int i = 0; i < M.Length; i++)
            {
                    Mout[i] = M[i] * value;
            }
            return Mout;
        }

        public static double[] multiplyMatrix(double[,] M, double[] value)
        {
            int m_iRows = M.GetLength(0);
            int m_iCols = M.GetLength(1);
            if (m_iCols != value.Length)
                throw new MatrixException("Invalid Matrix specified, not correct size");
            double[] Mout = new double[value.Length];
            for (int i = 0; i < m_iRows; i++)
            {
                for (int j = 0; j < m_iCols; j++)
                {
                    Mout[i] += M[i, j] * value[j];
                }
            }
            return Mout;
        }

        public static double[,] multiplyMatrix(double[,] M, double[,] value)
        {
            int m_iRows = M.GetLength(0);
            int m_iCols = M.GetLength(1);
            int v_iRows = value.GetLength(0);
            int v_iCols = value.GetLength(1);
            if (m_iCols != v_iRows)
            {
                return multiplyMatrix(value, M);
                throw new MatrixException("Invalid Matrix specified, not correct size");
            }
            double[,] Mout = new double[m_iRows, v_iCols];
            for (int i = 0; i < m_iRows; i++)
            {
                for (int j = 0; j < v_iCols; j++)
                {
                    Mout[i, j] = dotProduct(getRow(M, i), getCol(value, j));
                }
            }
            return Mout;
        }

        public static double[] getRow(double[,] M, int row)
        {
            int m_iRows = M.GetLength(0);
            int m_iCols = M.GetLength(1);
            if (m_iRows < row)
                throw new MatrixException("Invalid Matrix specified, not correct size");
            double[] Rout = new double[m_iCols];
            for (int i = 0; i < m_iCols; i++)
            {
                Rout[i] = M[row, i];
            }
            return Rout;
        }

        public static double[] getCol(double[,] M, int col)
        {
            int m_iRows = M.GetLength(0);
            int m_iCols = M.GetLength(1);
            if (m_iCols < col)
                throw new MatrixException("Invalid Matrix specified, not correct size");
            double[] Cout = new double[m_iRows];
            for (int i = 0; i < m_iRows; i++)
            {
                Cout[i] = M[i, col];
            }
            return Cout;
        }

        public static double dotProduct(double[] V1, double[] V2)
        {
            int m_iRows = V1.Length;
            int m_iCols = V2.Length;
            if (m_iCols != m_iRows)
                throw new MatrixException("Invalid Matrix specified, not correct size");
            double product = 0;
            for (int i = 0; i < m_iRows; i++)
            {
                product += V1[i] * V2[i];
            }
            return product;
        }

        #endregion

        #region Orientation Functions

        public static Quaternion QfromXZ(Vector3 xAxis, Vector3 zAxis)
        {
            xAxis.Normalize();
            zAxis.Normalize();
            Vector3 yAxis = Vector3.Cross(zAxis, xAxis);
            yAxis.Normalize();
            return Quaternion.CreateFromRotationMatrix(new Matrix(xAxis.X, xAxis.Y, xAxis.Z, 0, yAxis.X, yAxis.Y, yAxis.Z, 0, zAxis.X, zAxis.Y, zAxis.Z, 0, 0, 0, 0, 1));
        }

        public static Quaternion QfromZaxis(Vector3 Zaxis, Quaternion _currentOrientation)
        {
            Matrix _currentPose = Matrix.CreateFromQuaternion(_currentOrientation);
            Vector3 axis = Vector3.Cross(Vector3.Normalize(_currentPose.Backward), Vector3.Normalize(Zaxis));
            float angle = (float)Math.Asin((double)axis.Length());
            if (Math.Abs(angle) < MathHelper.ToRadians(0.2f))
            {
                return _currentOrientation;
            }
            if (Vector3.Transform(Zaxis, Quaternion.Inverse(_currentOrientation)).Z < 0)
            {
                angle = Math.Sign(angle) * ((float)Math.PI - Math.Abs(angle));
            }
            return Quaternion.CreateFromAxisAngle(Vector3.Normalize(axis), angle) * _currentOrientation;
        }

        public static bool isOrientationAligned(Quaternion Q1, Quaternion Q2, double error)
        {
            float angle;
            Vector3 axis;
            Quaternion change = Quaternion.Inverse(Q1) * Q2;
            getAxisAngle(change, out axis, out angle);
            if (Math.Abs(angle) < error)
            {
                return true;
            }
            return false;
        }

        public static Vector3 getOrientationError(Quaternion reference, Quaternion measured)
        {
            Vector3 changeAxis = Vector3.Zero;
            float changeAngle = 0;
            Quaternion changeOrientation = Quaternion.Inverse(measured) * reference;
            SF.getAxisAngle(changeOrientation, out changeAxis, out changeAngle);
            return Vector3.Transform(changeAxis, measured) * changeAngle;
        }

        public static Vector3 getOrientationError(Matrix reference, Matrix measured)
        {
            return Vector3.Multiply((Vector3.Cross(getAxisfromMatrix(measured, 1), getAxisfromMatrix(reference, 1)) + Vector3.Cross(getAxisfromMatrix(measured, 2), getAxisfromMatrix(reference, 2)) + Vector3.Cross(getAxisfromMatrix(measured, 3), getAxisfromMatrix(reference, 3))), 0.5f);
        }

        static Vector3 getAxisfromMatrix(Matrix rotation, int column)
        {
            if (column == 1)
            {
                return new Vector3(rotation.M11, rotation.M12, rotation.M13);
            }
            if (column == 2)
            {
                return new Vector3(rotation.M21, rotation.M22, rotation.M23);
            }
            if (column == 3)
            {
                return new Vector3(rotation.M31, rotation.M32, rotation.M33);
            }
            return Vector3.Zero;
        }

        public static Matrix M(Matrix mat1, Matrix mat2)
        {
            return Matrix.Transpose(Matrix.Transpose(mat1) * Matrix.Transpose(mat2));
        }

        static Matrix R(Matrix Rz, Matrix Ry, Matrix Rx)
        {
            return M(M(Rz, Ry), Rx);
        }

        public static void getAxisAngle(Quaternion quaternion, out Vector3 outAxis, out float outAngle)
        {
            quaternion.Normalize();
            if (quaternion.W < 0)
            {
                quaternion = Quaternion.Negate(quaternion);
            }
            float angle = 2 * (float)Math.Acos(quaternion.W);
            float s = (float)Math.Sin(1.0 * angle / 2);
            if (Math.Abs(s) < 1e-6)
            {
                outAxis.X = quaternion.X;
                outAxis.Y = quaternion.Y;
                outAxis.Z = quaternion.Z;
                if (Single.IsNaN(quaternion.Z))
                {
                    outAxis.Z = 1;
                }
                outAxis.Normalize();
                if (quaternion.X != 0)
                {
                    if (quaternion.X < 0)
                    {
                        outAngle = (float)(2 * Math.Atan2(-(double)quaternion.X, (double)quaternion.W * -outAxis.X));
                    }
                    else
                    {
                        outAngle = (float)(2 * Math.Atan2((double)quaternion.X, (double)quaternion.W * outAxis.X));
                    }
                }
                else if (quaternion.Y != 0)
                {
                    if (quaternion.Y < 0)
                    {
                        outAngle = (float)(2 * Math.Atan2(-(double)quaternion.Y, (double)quaternion.W * -outAxis.Y));
                    }
                    else
                    {
                        outAngle = (float)(2 * Math.Atan2((double)quaternion.Y, (double)quaternion.W * outAxis.Y));
                    }
                }
                else if (quaternion.Z != 0)
                {
                    if (quaternion.Z < 0)
                    {
                        outAngle = (float)(2 * Math.Atan2(-(double)quaternion.Z, (double)quaternion.W * -outAxis.Z));
                    }
                    else
                    {
                        outAngle = (float)(2 * Math.Atan2((double)quaternion.Z, (double)quaternion.W * outAxis.Z));
                    }
                }
                else
                {
                    outAngle = 0;
                    outAxis = new Vector3(0, 0, 1);
                }

            }
            else
            {
                Vector3 axis = new Vector3(1.0f * quaternion.X / s, 1.0f * quaternion.Y / s, 1.0f * quaternion.Z / s);
                axis.Normalize();
                s = quaternion.X == 0 ? s : 1.0f * quaternion.X / axis.X;
                s = quaternion.Y == 0 ? s : 1.0f * quaternion.Y / axis.Y;
                s = quaternion.Z == 0 ? s : 1.0f * quaternion.Z / axis.Z;

                if (float.IsNaN(quaternion.Z))
                {
                    outAxis.Z = 1;
                }
                outAxis = new Vector3(1.0f * quaternion.X / s, 1.0f * quaternion.Y / s, 1.0f * quaternion.Z / s);
                outAxis.Normalize();
                if (quaternion.X != 0)
                {
                    if (quaternion.X < 0)
                    {
                        outAngle = (float)(2 * Math.Atan2(-(double)quaternion.X, (double)quaternion.W * -outAxis.X));
                    }
                    else
                    {
                        outAngle = (float)(2 * Math.Atan2((double)quaternion.X, (double)quaternion.W * outAxis.X));
                    }
                }
                else if (quaternion.Y != 0)
                {
                    if (quaternion.Y < 0)
                    {
                        outAngle = (float)(2 * Math.Atan2(-(double)quaternion.Y, (double)quaternion.W * -outAxis.Y));
                    }
                    else
                    {
                        outAngle = (float)(2 * Math.Atan2((double)quaternion.Y, (double)quaternion.W * outAxis.Y));
                    }
                }
                else if (quaternion.Z != 0)
                {
                    if (quaternion.Z < 0)
                    {
                        outAngle = (float)(2 * Math.Atan2(-(double)quaternion.Z, (double)quaternion.W * -outAxis.Z));
                    }
                    else
                    {
                        outAngle = (float)(2 * Math.Atan2((double)quaternion.Z, (double)quaternion.W * outAxis.Z));
                    }
                }
                else
                {
                    outAngle = 0;
                }
            }
        }

        #endregion

        #region Kuka Functions


        public static readonly String[] cardinalKeys = new String[] { "X", "Y", "Z", "A", "B", "C" };
        public static readonly String[] axisKeys = new String[] { "A1", "A2", "A3", "A4", "A5", "A6" };
        public static readonly String[] rotationKeys = new String[] { "X1", "X2", "X3", "Y1", "Y2", "Y3", "Z1", "Z2", "Z3" };
        public static readonly String[] axisVecotrKeys = new String[] { "XX", "XY", "XZ", "ZX", "ZY", "ZZ" };

        public static TimeCoordinate AverageRateOfChange(TimeCoordinate[] list, double framerate)
        {
            TimeCoordinate averageRate = list[0].getRateOfChange(list[1],framerate); 

            for (int i = 2; i < list.Length; i++)
            {
                averageRate += list[i - 1].getRateOfChange(list[i],framerate);
            }
            return averageRate / (float)(list.Length - 1);
        }


        /// <summary>
        /// Creates a quaternion from Kuka coordinates, ABC in degrees
        /// </summary>
        /// <param name="pose"></param> A double[6] array {X, Y, Z, A, B, C} 
        /// <returns></returns>
        public static Quaternion MakeQuaternionFromKuka(double[] pose)
        {
            Quaternion Rz = Quaternion.CreateFromRotationMatrix(Matrix.CreateRotationZ((float)(1.0 * pose[3] * Math.PI / 180)));
            Quaternion Ry = Quaternion.CreateFromRotationMatrix(Matrix.CreateRotationY((float)(1.0 * pose[4] * Math.PI / 180)));
            Quaternion Rx = Quaternion.CreateFromRotationMatrix(Matrix.CreateRotationX((float)(1.0 * pose[5] * Math.PI / 180)));
            return (Rz * Ry * Rx);
        }

        public static Quaternion MakeQuaternionFromKuka(double A, double B, double C)
        {
            Quaternion Rz = Quaternion.CreateFromRotationMatrix(Matrix.CreateRotationZ((float)(1.0 * A * Math.PI / 180)));
            Quaternion Ry = Quaternion.CreateFromRotationMatrix(Matrix.CreateRotationY((float)(1.0 * B * Math.PI / 180)));
            Quaternion Rx = Quaternion.CreateFromRotationMatrix(Matrix.CreateRotationX((float)(1.0 * C * Math.PI / 180)));
            return (Rz * Ry * Rx);
        }

        public static Matrix MakeMatrixFromKuka(double[] pose)
        {
            Matrix Rz = Matrix.CreateRotationZ((float)(1.0 * pose[3] * Math.PI / 180));
            Matrix Ry = Matrix.CreateRotationY((float)(1.0 * pose[4] * Math.PI / 180));
            Matrix Rx = Matrix.CreateRotationX((float)(1.0 * pose[5] * Math.PI / 180));
            Matrix poseout = R(Rz, Ry, Rx);
            poseout.Translation = new Vector3((float)pose[0], (float)pose[1], (float)pose[2]);
            return poseout;
        }


        public static void getKukaAngles(TimeCoordinate Pose, ref float[] KukaAngleOut)
        {
            getKukaAngles(Pose.Orientation, ref KukaAngleOut);
            KukaAngleOut[0] = (float)Pose.x;
            KukaAngleOut[1] = (float)Pose.y;
            KukaAngleOut[2] = (float)Pose.z;
        }

        public static Vector3 getKukaAngles(Quaternion rotation)
        {

            float A = 0;
            float B = 0;
            float C = 0;

            Matrix rotationMat = Matrix.CreateFromQuaternion(rotation);
            rotationMat = Matrix.Transpose(rotationMat);

            B = (float)Math.Atan2(-rotationMat.M31, Math.Sqrt(rotationMat.M32 * rotationMat.M32 + rotationMat.M33 * rotationMat.M33));

            if (Math.Abs(Math.Abs(B) - Math.PI / 2) < 1e-6)
            {
                // Gimbal lock situation! A and C form a line of infinate solutions.
                C = 0;// (float)Math.PI / 5f;
                A = (float)Math.Atan2(Math.Sign(B) * rotationMat.M23, Math.Sign(B) * rotationMat.M13) + Math.Sign(B) * C;
            }
            else
            {
                A = (float)Math.Atan2(rotationMat.M21, rotationMat.M11);
                C = (float)Math.Atan2(rotationMat.M32, rotationMat.M33);
            }

            return new Vector3(MathHelper.ToDegrees(A), MathHelper.ToDegrees(B), MathHelper.ToDegrees(C));
        }

        /// <summary>
        /// Updates the float[6] array with kuka ABC angles in [X,Y,Z,A,B,C]. This only works when the rotation is less than 90 degrees 
        /// and will return iff angle in axis/angle representation is less than 90 degrees.
        /// </summary>
        /// <param name="rotation"></param> quaternion representing the rotation
        /// <returns></returns>
        public static void getKukaAngles(Quaternion rotation, out double[] KukaAngleOut)
        {
            KukaAngleOut = new double[6];
            double A = 0;
            double B = 0;
            double C = 0;

            Matrix rotationMat = Matrix.CreateFromQuaternion(rotation);
            rotationMat = Matrix.Transpose(rotationMat);

            B = Math.Atan2(-rotationMat.M31, Math.Sqrt(rotationMat.M32 * rotationMat.M32 + rotationMat.M33 * rotationMat.M33));

            if (Math.Abs(Math.Abs(B) - 1.0 * Math.PI / 2) < 1e-6)
            {
                // Gimbal lock situation! A and C form a line of infinate solutions.
                C = 0;// (float)Math.PI / 5f;
                A = Math.Atan2(Math.Sign(B) * rotationMat.M23, Math.Sign(B) * rotationMat.M13) + Math.Sign(B) * C;
            }
            else
            {
                A = Math.Atan2(rotationMat.M21, rotationMat.M11);
                C = Math.Atan2(rotationMat.M32, rotationMat.M33);
            }

            KukaAngleOut[3] = 1.0 * A * 180 / Math.PI;
            KukaAngleOut[4] = 1.0 * B * 180 / Math.PI;
            KukaAngleOut[5] = 1.0 * C * 180 / Math.PI;
        }

        /// <summary>
        /// Updates the float[6] array with kuka ABC angles in [X,Y,Z,A,B,C]. This only works when the rotation is less than 90 degrees 
        /// and will return iff angle in axis/angle representation is less than 90 degrees.
        /// </summary>
        /// <param name="rotation"></param> quaternion representing the rotation
        /// <returns></returns>
        public static void getKukaAngles(Quaternion rotation, ref float[] KukaAngleOut)
        {
            float A = 0;
            float B = 0;
            float C = 0;

            Matrix rotationMat = Matrix.CreateFromQuaternion(rotation);
            rotationMat = Matrix.Transpose(rotationMat);

            B = (float)Math.Atan2(-rotationMat.M31, Math.Sqrt(rotationMat.M32 * rotationMat.M32 + rotationMat.M33 * rotationMat.M33));

            if (Math.Abs(Math.Abs(B) - 1.0 * Math.PI / 2) < 1e-6)
            {
                // Gimbal lock situation! A and C form a line of infinate solutions.
                C = 0;// (float)Math.PI / 5f;
                A = (float)Math.Atan2(Math.Sign(B) * rotationMat.M23, Math.Sign(B) * rotationMat.M13) + Math.Sign(B) * C;
            }
            else
            {
                A = (float)Math.Atan2(rotationMat.M21, rotationMat.M11);
                C = (float)Math.Atan2(rotationMat.M32, rotationMat.M33);
            }

            KukaAngleOut[3] = MathHelper.ToDegrees(A);
            KukaAngleOut[4] = MathHelper.ToDegrees(B);
            KukaAngleOut[5] = MathHelper.ToDegrees(C);
        }

        /// <summary>
        /// Returns a double[6] array [X,Y,Z,A,B,C]. This only works when the rotation is less than 90 degrees 
        /// and will return iff angle in axis/angle representation is less than 90 degrees.
        /// </summary>
        /// <param name="pose"></param>
        /// <returns></returns>
        public static void getKukaAngles(Matrix pose, ref float[] kukaOut)
        {
            float A = 0;
            float B = 0;
            float C = 0;
            Matrix rotationMat = Matrix.Transpose(pose);

            B = (float)Math.Atan2(-rotationMat.M31, Math.Sqrt(rotationMat.M32 * rotationMat.M32 + rotationMat.M33 * rotationMat.M33));

            if (Math.Abs(Math.Abs(B) - 1.0 * Math.PI / 2) < 1e-6)
            {
                // Gimbal lock situation! A and C form a line of infinate solutions.
                C = 0;// (float)Math.PI / 5f;
                A = (float)Math.Atan2(Math.Sign(B) * rotationMat.M23, Math.Sign(B) * rotationMat.M13) + Math.Sign(B) * C;
            }
            else
            {
                A = (float)Math.Atan2(rotationMat.M21, rotationMat.M11);
                C = (float)Math.Atan2(rotationMat.M32, rotationMat.M33);
            }

            kukaOut[0] = pose.Translation.X;
            kukaOut[1] = pose.Translation.Y;
            kukaOut[2] = pose.Translation.Z;
            kukaOut[3] = MathHelper.ToDegrees(A);
            kukaOut[4] = MathHelper.ToDegrees(B);
            kukaOut[5] = MathHelper.ToDegrees(C);

        }

        #endregion

        

        public static double[] getAverage(double[][] arraylist)
        {
            int n = arraylist.Length;
            if (n < 1)
            {
                return  new double[] { 0, 0, 0, 0, 0, 0 };
            }
            else
            {
                double[] average = new double[] { 0, 0, 0, 0, 0, 0 };
                for (int i = 0; i < n; i++)
                {
                    for (int j = 0; j < 6; j++)
                    {
                        average[j] += arraylist[i][j];
                    }
                }
                for (int i = 0; i < 6; i++)
                {
                    average[i] = 1.0 * average[i] / n;
                }
                return average;
            }
        }

        #region Kinamatic equations, FK/IK and transformation matricies


        public static double[,] GetInverseJacobian(double[] currentAngles, Vector3 EE)
        {
            return getTipInverseJacobian(InverseJacobianWrist(currentAngles, 1e-6), EE);
        }

        static double[,] getTipInverseJacobian(double[,] InvWristJacobian, Vector3 EE)
        {
            double[,] InvSkewEE = new double[,] { { 1, 0, 0, 0, -(EE.Z), EE.Y }, { 0, 1, 0, EE.Z, 0, -EE.X }, { 0, 0, 1, -EE.Y, EE.X, 0 }, { 0, 0, 0, 1, 0, 0 }, { 0, 0, 0, 0, 1, 0 }, { 0, 0, 0, 0, 0, 1 } };
            return SF.multiplyMatrix(InvWristJacobian, InvSkewEE);
        }

        /// <summary>
        /// Computes the inverse jacobian to the WRIST where if singularity occures the result aproximates the jacobian.
        /// </summary>
        /// <param name="t"></param> 6 angles in radians.
        /// <param name="error"></param> Error for singularity detection, 1e-6 specified if greater than 1e-2.
        /// <returns></returns>
        static double[,] InverseJacobianWrist(double[] t, double error)
        {
            if (t.Length == 6)
            {
                error = (Math.Abs(error) > 1e-2) ? 1e-6 : Math.Abs(error);
                return InverseJacobianWrist(t[0], t[1], t[2], t[3], t[4], t[5], error);
            }
            else
            {
                throw new KukaException(string.Format("Incorrect number of angles supplied. Need 6 angles but {0} were given.", t.Length));
            }
        }

        /// <summary>
        /// Computes the inverse jacobian to the WRIST where if singularity occures aproximates the jacobian.
        /// Angles are in radians!!!
        /// </summary>
        /// <param name="t1"></param>
        /// <param name="t2"></param>
        /// <param name="t3"></param>
        /// <param name="t4"></param>
        /// <param name="t5"></param>
        /// <param name="t6"></param>
        /// <param name="EE"></param>
        /// <returns></returns>
        static double[,] InverseJacobianWrist(double t1, double t2, double t3, double t4, double t5, double t6, double error)
        {
            double[,] inverseJoc = new double[6, 6];
            double s1 = Math.Sin(t1);
            double c1 = Math.Cos(t1);
            double s2 = Math.Sin(t2);
            double s2p3 = Math.Sin(t2 + t3);
            double c2p3 = Math.Cos(t2 + t3);
            double c2 = Math.Cos(t2);
            double s3 = Math.Sin(t3);
            double c3 = Math.Cos(t3);
            double s3p = Math.Sin(t3 - 1.0 * Math.PI / 2);
            double c3p = Math.Cos(t3 - 1.0 * Math.PI / 2);
            double s4 = Math.Sin(t4);
            double c4 = Math.Cos(t4);
            double s5 = Math.Sin(t5);
            double c5 = Math.Cos(t5);
            double s6 = Math.Sin(t6);
            double c6 = Math.Cos(t6);
            double c234 = Math.Cos(t2 + t3 + t4);
            // These are the denominators withing the inverse Jacobian, if they tend to zero singularity is reached and velocities will tend to inf!
            double cot5 = (Math.Abs(s5) < error) ? 1.0 * c5 / error : 1.0 * c5 / s5;
            double singularity1 = (Math.Abs(103 * c2p3 + 7 * s2p3 + 112 * c2 + 5) < error) ? Math.Sign(103 * c2p3 + 7 * s2p3 + 112 * c2 + 5) * error : (103 * c2p3 + 7 * s2p3 + 112 * c2 + 5);
            double singularity2 = (Math.Abs(1960 * c3 - 28840 * s3) < error) ? Math.Sign(1960 * c3 - 28840 * s3) * error : (1960 * c3 - 28840 * s3);
            double singularity3 = (Math.Abs(7 * c3 - 103 * s3) < error) ? Math.Sign(7 * c3 - 103 * s3) * error : (7 * c3 - 103 * s3);
            double singularity4 = (Math.Abs(3920 * c3 - 57680 * s3) < error) ? Math.Sign(3920 * c3 - 57680 * s3) * error : (3920 * c3 - 57680 * s3);
            double Sing5 = (3605 * c2 * s5 - 175 * c3 * s5 - 53045 * s2 * s5 + 2575 * s3 * s5 + 57680 * c2 * s3 * s5 - 7210 * c2 * c3 * c3 * s5 + 52800 * c3 * c3 * s2 * s5 - 3920 * c2 * c3 * s5 + 52800 * c2 * c3 * s3 * s5 + 7210 * c3 * s2 * s3 * s5);
            double singularity5 = (Math.Abs(Sing5) < error) ? Math.Sign(Sing5) * error : (Sing5);
            double Sing6 = (c2p3 * c2p3 * s5 - c2p3 * c2p3 * c4 * c4 * s5 + s2p3 * c2 * s3 * s5 + s2p3 * c3 * s2 * s5 - s2p3 * c4 * c4 * s2 * s5 * s3p + c2p3 * c2 * c4 * c5 * c3p - c2p3 * c4 * c5 * s2 * s3p + s2p3 * c2 * c4 * c5 * s3p + s2p3 * c4 * c5 * c3p * s2 - c2p3 * c2 * c4 * c4 * s5 * s3p - c2p3 * c4 * c4 * c3p * s2 * s5 + s2p3 * c2 * c4 * c4 * c3p * s5 - s2p3 * c2 * c4 * c4 * s3 * s5 - s2p3 * c3 * c4 * c4 * s2 * s5);
            double singularity6 = (Math.Abs(Sing6) < error) ? Math.Sign(Sing6) * error : (Sing6);
            double singularity7 = (Math.Abs(7 * c3 * s5 - 103 * s3 * s5) < error) ? Math.Sign(7 * c3 * s5 - 103 * s3 * s5) * error : (7 * c3 * s5 - 103 * s3 * s5);
            double singularity8 = ((Math.Abs(s5) < error) ? Math.Sign(s5) * error : s5);
            double Sing9 = (5 * s5 * (721 * c2 - 35 * c3 - 10609 * s2 + 515 * s3 - 784 * c2 * c3 + 11536 * c2 * s3 - 1442 * c2 * c3 * c3 + 10560 * c3 * c3 * s2 + 1442 * c3 * s2 * s3 + 10560 * c2 * c3 * s3));
            double singularity9 = ((Math.Abs(Sing9) < error) ? Math.Sign(Sing9) * error : Sing9);
            double Sing10 = (3605 * c2 - 175 * c3 - 53045 * s2 + 2575 * s3 - 3920 * c2 * c3 + 57680 * c2 * s3 - 7210 * c2 * c3 * c3 + 52800 * c3 * c3 * s2 + 7210 * c3 * s2 * s3 + 52800 * c2 * c3 * s3);
            double singularity10 = ((Math.Abs(Sing10) < error) ? Math.Sign(Sing10) * error : Sing10);

            inverseJoc[0, 0] = -1.0 * s1 / (5 * singularity1);
            inverseJoc[0, 1] = -1.0 * c1 / (5 * singularity1);
            inverseJoc[0, 2] = 0;
            inverseJoc[0, 3] = 0;
            inverseJoc[0, 4] = 0;
            inverseJoc[0, 5] = 0;
            inverseJoc[1, 0] = -1.0 * (103 * c1 * c2 * c3 - 103 * c1 * s2 * s3 + 7 * c1 * c2 * s3 + 7 * c1 * c3 * s2) / (2 * singularity2);
            inverseJoc[1, 1] = 1.0 * (7 * c2 * s1 * s3 + 7 * c3 * s1 * s2 - 103 * s1 * s2 * s3 + 103 * c2 * c3 * s1) / (2 * singularity2);
            inverseJoc[1, 2] = -1.0 * (7 * c2p3 - 103 * s2p3) / (560 * singularity3);
            inverseJoc[1, 3] = 0;
            inverseJoc[1, 4] = 0;
            inverseJoc[1, 5] = 0;
            inverseJoc[2, 0] = 1.0 * (112 * c1 * c2 - 103 * c1 * s2 * s3 + 103 * c1 * c2 * c3 + 7 * c1 * c2 * s3 + 7 * c1 * c3 * s2) / singularity4;
            inverseJoc[2, 1] = -1.0 * (112 * c2 * s1 + 7 * c2 * s1 * s3 + 7 * c3 * s1 * s2 - 103 * s1 * s2 * s3 + 103 * c2 * c3 * s1) / singularity4;
            inverseJoc[2, 2] = -1.0 * (103 * s2p3 - 7 * c2p3 + 112 * s2) / (560 * singularity3);
            inverseJoc[2, 3] = 0;
            inverseJoc[2, 4] = 0;
            inverseJoc[2, 5] = 0;
            inverseJoc[3, 0] = -1.0 * (103 * c2 * s1 * s5 + 112 * c1 * c2 * c2 * c5 * s4 - 103 * c2 * c3 * c3 * s1 * s5 - 7 * c3 * c3 * s1 * s2 * s5 + 5 * c1 * c2 * c5 * s4 + 103 * c4 * c5 * s1 * s2 + 103 * c1 * c2 * c2 * c3 * c5 * s4 + 7 * c2 * c3 * c3 * c4 * c5 * s1 + 7 * c1 * c2 * c2 * c5 * s3 * s4 - 103 * c3 * c3 * c4 * c5 * s1 * s2 - 7 * c2 * c3 * s1 * s3 * s5 + 103 * c3 * s1 * s2 * s3 * s5 + 7 * c1 * c2 * c3 * c5 * s2 * s4 - 103 * c2 * c3 * c4 * c5 * s1 * s3 - 103 * c1 * c2 * c5 * s2 * s3 * s4 - 7 * c3 * c4 * c5 * s1 * s2 * s3) / singularity5;
            inverseJoc[3, 1] = 1.0 * (103 * c1 * c2 * c3 * c3 * s5 - 103 * c1 * c2 * s5 + 7 * c1 * c3 * c3 * s2 * s5 + 112 * c2 * c2 * c5 * s1 * s4 - 103 * c1 * c4 * c5 * s2 + 5 * c2 * c5 * s1 * s4 + 103 * c1 * c3 * c3 * c4 * c5 * s2 + 103 * c2 * c2 * c3 * c5 * s1 * s4 + 7 * c2 * c2 * c5 * s1 * s3 * s4 + 7 * c1 * c2 * c3 * s3 * s5 - 103 * c1 * c3 * s2 * s3 * s5 - 7 * c1 * c2 * c3 * c3 * c4 * c5 + 103 * c1 * c2 * c3 * c4 * c5 * s3 + 7 * c1 * c3 * c4 * c5 * s2 * s3 + 7 * c2 * c3 * c5 * s1 * s2 * s4 - 103 * c2 * c5 * s1 * s2 * s3 * s4) / singularity5;
            inverseJoc[3, 2] = -1.0 * (c5 * s2 * s4) / (5 * singularity7);
            inverseJoc[3, 3] = 1.0 * (c2p3 * c1 * c4 * c4 * s5 - c2p3 * c1 * s5 + c1 * c2 * c4 * c4 * s5 * s3p + c1 * c4 * c4 * c3p * s2 * s5 - c1 * c2 * c4 * c5 * c3p + c1 * c4 * c5 * s2 * s3p + c3 * c5 * s1 * s4 * s3p - c2 * c2 * c3 * c5 * s1 * s4 * s3p - c2 * c2 * c5 * c3p * s1 * s3 * s4 + c2p3 * c2 * c5 * s1 * s4 * s3p + c2p3 * c5 * c3p * s1 * s2 * s4 + c3 * c4 * c3p * s1 * s4 * s5 + c2 * c2 * c4 * s1 * s3 * s4 * s5 * s3p + c2p3 * c2 * c4 * c3p * s1 * s4 * s5 - c2p3 * c4 * s1 * s2 * s4 * s5 * s3p - c2 * c3 * c5 * c3p * s1 * s2 * s4 + c2 * c5 * s1 * s2 * s3 * s4 * s3p - c2 * c2 * c3 * c4 * c3p * s1 * s4 * s5 + c2 * c3 * c4 * s1 * s2 * s4 * s5 * s3p + c2 * c4 * c3p * s1 * s2 * s3 * s4 * s5) / singularity6;
            inverseJoc[3, 4] = 1.0 * (c2p3 * s1 * s5 - c2p3 * c4 * c4 * s1 * s5 - c2 * c4 * c4 * s1 * s5 * s3p - c4 * c4 * c3p * s1 * s2 * s5 + c2 * c4 * c5 * c3p * s1 + c1 * c3 * c5 * s4 * s3p - c4 * c5 * s1 * s2 * s3p + c2p3 * c1 * c2 * c5 * s4 * s3p + c2p3 * c1 * c5 * c3p * s2 * s4 + c1 * c3 * c4 * c3p * s4 * s5 - c1 * c2 * c2 * c3 * c5 * s4 * s3p - c1 * c2 * c2 * c5 * c3p * s3 * s4 + c2p3 * c1 * c2 * c4 * c3p * s4 * s5 - c2p3 * c1 * c4 * s2 * s4 * s5 * s3p - c1 * c2 * c3 * c5 * c3p * s2 * s4 + c1 * c2 * c5 * s2 * s3 * s4 * s3p - c1 * c2 * c2 * c3 * c4 * c3p * s4 * s5 + c1 * c2 * c2 * c4 * s3 * s4 * s5 * s3p + c1 * c2 * c3 * c4 * s2 * s4 * s5 * s3p + c1 * c2 * c4 * c3p * s2 * s3 * s4 * s5) / singularity6;
            inverseJoc[3, 5] = s2p3 - 1.0 * (Math.Cos(t2 + t3 + t4) * cot5) / 2 - 1.0 * (Math.Cos(t2 + t3 - t4) * cot5) / 2;
            inverseJoc[4, 0] = 1.0 * (112 * c1 * c2 * c2 * c4 - 103 * s1 * s2 * s4 + 5 * c1 * c2 * c4 + 103 * c1 * c2 * c2 * c3 * c4 + 7 * c1 * c2 * c2 * c4 * s3 - 7 * c2 * c3 * c3 * s1 * s4 + 103 * c3 * c3 * s1 * s2 * s4 + 7 * c1 * c2 * c3 * c4 * s2 - 103 * c1 * c2 * c4 * s2 * s3 + 103 * c2 * c3 * s1 * s3 * s4 + 7 * c3 * s1 * s2 * s3 * s4) / singularity10;
            inverseJoc[4, 1] = -1.0 * (103 * c1 * s2 * s4 + 112 * c2 * c2 * c4 * s1 + 5 * c2 * c4 * s1 + 7 * c1 * c2 * c3 * c3 * s4 + 103 * c2 * c2 * c3 * c4 * s1 - 103 * c1 * c3 * c3 * s2 * s4 + 7 * c2 * c2 * c4 * s1 * s3 + 7 * c2 * c3 * c4 * s1 * s2 - 103 * c1 * c2 * c3 * s3 * s4 - 103 * c2 * c4 * s1 * s2 * s3 - 7 * c1 * c3 * s2 * s3 * s4) / singularity10;
            inverseJoc[4, 2] = 1.0 * (c4 * s2) / (5 * singularity3);
            inverseJoc[4, 3] = c4 * s1 - c1 * c2 * s3 * s4 - c1 * c3 * s2 * s4;
            inverseJoc[4, 4] = c1 * c4 + c2 * s1 * s3 * s4 + c3 * s1 * s2 * s4;
            inverseJoc[4, 5] = -c2p3 * s4;
            inverseJoc[5, 0] = 1.0 * (103 * c4 * s1 * s2 + 112 * c1 * c2 * c2 * s4 + 5 * c1 * c2 * s4 + 103 * c1 * c2 * c2 * c3 * s4 + 7 * c2 * c3 * c3 * c4 * s1 + 7 * c1 * c2 * c2 * s3 * s4 - 103 * c3 * c3 * c4 * s1 * s2 + 7 * c1 * c2 * c3 * s2 * s4 - 103 * c2 * c3 * c4 * s1 * s3 - 103 * c1 * c2 * s2 * s3 * s4 - 7 * c3 * c4 * s1 * s2 * s3) / singularity9;
            inverseJoc[5, 1] = -1.0 * (5 * c2 * s1 * s4 + 112 * c2 * c2 * s1 * s4 - 103 * c1 * c4 * s2 - 7 * c1 * c2 * c3 * c3 * c4 + 103 * c1 * c3 * c3 * c4 * s2 + 103 * c2 * c2 * c3 * s1 * s4 + 7 * c2 * c2 * s1 * s3 * s4 + 103 * c1 * c2 * c3 * c4 * s3 + 7 * c1 * c3 * c4 * s2 * s3 + 7 * c2 * c3 * s1 * s2 * s4 - 103 * c2 * s1 * s2 * s3 * s4) / singularity9;
            inverseJoc[5, 2] = 1.0 * (s2 * s4) / (5 * singularity7);
            inverseJoc[5, 3] = 1.0 * (s1 * s4 + c1 * c2 * c4 * s3 + c1 * c3 * c4 * s2) / singularity8;
            inverseJoc[5, 4] = -1.0 * (c2 * c4 * s1 * s3 - c1 * s4 + c3 * c4 * s1 * s2) / singularity8;
            inverseJoc[5, 5] = 1.0 * (c2p3 * c4) / singularity8;
            return inverseJoc;
        }
        

        /// <summary>
        /// Computers the forwards kinimatics using a known EndEffector displacement aligned with T67. The angles are in DEGREES, EE is in mm
        /// </summary>
        /// <param name="angles"></param>
        /// <param name="EE"></param>
        /// <returns></returns>
        public static Pose forwardKinimatics(double[] angles, Vector3 EE)
        {
            return forwardKinimatics(angles[0], angles[1], angles[2], angles[3], angles[4], angles[5], EE);
        }

        /// <summary>
        /// Computers the forwards kinimatics using a known EndEffector displacement aligned with T67. The angles are in radians EE is in mm
        /// </summary>
        /// <param name="a1"></param>
        /// <param name="a2"></param>
        /// <param name="a3"></param>
        /// <param name="a4"></param>
        /// <param name="a5"></param>
        /// <param name="a6"></param>
        /// <param name="EE"></param>
        /// <returns></returns>
        static Pose forwardKinimatics(double a1, double a2, double a3, double a4, double a5, double a6, Vector3 EE)
        {
            double s1 = Math.Sin(a1);
            double c1 = Math.Cos(a1);
            double s2 = Math.Sin(a2);
            double c2 = Math.Cos(a2);
            double s3p = Math.Sin(a3 - 1.0 * Math.PI / 2);
            double c3p = Math.Cos(a3 - 1.0 * Math.PI / 2);
            double s4 = Math.Sin(a4);
            double c4 = Math.Cos(a4);
            double s5 = Math.Sin(a5);
            double c5 = Math.Cos(a5);
            double s6 = Math.Sin(a6);
            double c6 = Math.Cos(a6);

            double a = (c4 * s1 + s4 * (c1 * s2 * s3p - c1 * c2 * c3p));
            double b1 = (s1 * s4 - c4 * (c1 * s2 * s3p - c1 * c2 * c3p));
            double b2 = (c1 * c2 * s3p + c1 * c3p * s2);
            double b = (c5 * b1 - s5 * b2);

            double m11 = -s6 * a - c6 * b;
            double m12 = c6 * a - s6 * b;
            double m13 = -s5 * b1 - c5 * b2;
            double m14 = 25 * c1 + 560 * c1 * c2 - EE.X * (s6 * a + c6 * b) + EE.Y * (c6 * (c4 * s1 + s4 * (c1 * s2 * s3p - c1 * c2 * c3p)) - s6 * b) - (s5 * b1 + c5 * b2) * (EE.Z + 80) - 515 * c1 * c2 * s3p - 515 * c1 * c3p * s2 - 35 * c1 * s2 * s3p + 35 * c1 * c2 * c3p;

            a = (c1 * c4 + s4 * (c2 * c3p * s1 - s1 * s2 * s3p));
            b1 = (c1 * s4 - c4 * (c2 * c3p * s1 - s1 * s2 * s3p));
            b2 = (c2 * s1 * s3p + c3p * s1 * s2);
            b = (c5 * b1 + s5 * b2);
            double m21 = -s6 * a - c6 * b;
            double m22 = c6 * a - s6 * b;
            double m23 = c5 * b2 - s5 * b1;
            double m24 = EE.Y * (c6 * a - s6 * b) - 560 * c2 * s1 - EE.X * (s6 * a + c6 * b) - 25 * s1 - (s5 * b1 - c5 * b2) * (EE.Z + 80) - 35 * c2 * c3p * s1 + 515 * c2 * s1 * s3p + 515 * c3p * s1 * s2 + 35 * s1 * s2 * s3p;

            a = (c2 * s3p + c3p * s2);
            b1 = (c2 * c3p - s2 * s3p);
            b = (s5 * b1 + c4 * c5 * a);
            double m31 = c6 * b - s4 * s6 * a;
            double m32 = s6 * b + c6 * s4 * a;
            double m33 = c4 * s5 * a - c5 * b1;
            double m34 = 515 * s2 * s3p - 515 * c2 * c3p - 35 * c2 * s3p - 35 * c3p * s2 - 560 * s2 - (c5 * b1 - c4 * s5 * a) * (EE.Z + 80) + EE.X * (c6 * b - s4 * s6 * a) + EE.Y * (s6 * b + c6 * s4 * a) + 400;

            double m41 = 0;
            double m42 = 0;
            double m43 = 0;
            double m44 = 1;


            Matrix M = new Matrix((float)m11, (float)m12, (float)m13, (float)m14, (float)m21, (float)m22, (float)m23, (float)m24, (float)m31, (float)m32, (float)m33, (float)m34, (float)m41, (float)m42, (float)m43, (float)m44);
            M = Matrix.Transpose(M);
            return new Pose(M);
        }

        static double[] IK1to3(Pose des, Vector3 EE, double[] lastVal, ref ElbowPosition elbow, ref BasePosition basePos)
        {

            double wristOffset = Math.Atan2(35, 515);
            double theta1a, theta1b, theta1, theta2u, theta2d, theta2, theta3u, theta3d, theta3;
            Vector3 Wrist = des * (-EE - new Vector3(0, 0, 80));
            if (Wrist.Z < 0) throw new InverseKinematicsException("Out of workspace");
            theta1a = -Math.Atan2(Wrist.Y, Wrist.X);
            theta1b = (theta1a > 0) ? theta1a - Math.PI : theta1a + Math.PI;
            if (Math.Abs(lastVal[0] - theta1a) < Math.Abs(lastVal[0] - theta1b))
            {
                theta1 = theta1a;
                basePos = BasePosition.front;
            }
            else
            {
                theta1 = theta1b;
                basePos = BasePosition.back;
            }
            if (theta1 < -170.0 * Math.PI / 180 || theta1 > 170.0 * Math.PI / 180)
            {
                throw new InverseKinematicsException("Out of workspace");
            }
            Vector3 Base = new Vector3(25 * (float)Math.Cos(-theta1), 25 * (float)Math.Sin(-theta1), 400f);
            Vector3 LinkBW = Wrist - Base;
            if (Math.Abs(LinkBW.Length() - (560 + Math.Sqrt(515 * 515 + 35 * 35))) < 1e-6)
            {
                throw new InverseKinematicsException("Out of workspace");
            }
            if (LinkBW.Length() == (560 + Math.Sqrt(515 * 515 + 35 * 35)))
            {
                theta3 = -Math.Atan2(35, 515);
            }
            Vector3 xHat = new Vector3(LinkBW.X, LinkBW.Y, 0);
            xHat.Normalize();
            double beta = Math.Atan2(LinkBW.Z, Math.Sqrt(LinkBW.X * LinkBW.X + LinkBW.Y * LinkBW.Y));
            double gamma = Math.Acos((1.0 * LinkBW.LengthSquared() + 560 * 560 - 35 * 35 - 515 * 515) / (2 * 560 * LinkBW.Length()));
            double alpha = Math.Acos((1.0 * 560 * 560 + 35 * 35 + 515 * 515 - LinkBW.LengthSquared()) / (2.0 * 560 * Math.Sqrt(35 * 35 + 515 * 515)));
            if (double.IsNaN(gamma))
            {
                gamma = 0;
            }
            if (double.IsNaN(alpha))
            {
                alpha = Math.PI;
            }
            if (basePos == BasePosition.front)
            {
                theta2u = -(beta + gamma);
                theta2d = -(beta - gamma);
                theta3u = Math.PI + wristOffset - alpha;
                theta3d = -(Math.PI - alpha - wristOffset);
            }
            else
            {
                theta2u = -Math.PI + (beta - gamma);
                theta2d = -Math.PI + (beta + gamma);
                theta3u = Math.PI + wristOffset - alpha;
                theta3d = -Math.PI + alpha + wristOffset;
            }
            if (Math.Abs(lastVal[1] - theta2u) < Math.Abs(lastVal[1] - theta2d))
            {
                theta2 = theta2u;
                theta3 = theta3u;
                elbow = ElbowPosition.up;
            }
            else
            {
                theta2 = theta2d;
                theta3 = theta3d;
                elbow = ElbowPosition.down;
            }

            if (elbow == ElbowPosition.up)
            {
                if (theta2u > (-190.0 * Math.PI / 180) && theta2u < (1.0 * Math.PI / 4) && theta3u < (156.0 * Math.PI / 180) && theta3u > (-120.0 * Math.PI / 180))
                {
                    theta2 = theta2u;
                    theta3 = theta3u;
                }
                else if ((theta2d > (-190.0 * Math.PI / 180) && theta2d < (1.0 * Math.PI / 4)) && (theta3d < (156.0 * Math.PI / 180) && (theta3d > (-120.0 * Math.PI / 180))))
                {
                    elbow = ElbowPosition.down;
                    theta2 = theta2d;
                    theta3 = theta3d;
                }
                else
                {
                    throw new InverseKinematicsException("Out of workspace");
                }
            }
            else if (elbow == ElbowPosition.down)
            {
                if ((theta2d > (-190.0 * Math.PI / 180) && theta2d < (1.0 * Math.PI / 4)) && (theta3d < (156.0 * Math.PI / 180) && (theta3d > (-120.0 * Math.PI / 180))))
                {
                    theta2 = theta2d;
                    theta3 = theta3d;
                }
                else if (theta2u > (-190.0 * Math.PI / 180) && theta2u < (1.0 * Math.PI / 4) && theta3u < (156.0 * Math.PI / 180) && theta3u > (-120.0 * Math.PI / 180))
                {
                    elbow = ElbowPosition.up;
                    theta2 = theta2u;
                    theta3 = theta3u;
                }
                else
                {
                    throw new InverseKinematicsException("Out of workspace");
                }
            }
            else
            {

                if (theta2u > (-190.0 * Math.PI / 180) && theta2u < (1.0 * Math.PI / 4) && theta3u < (156.0 * Math.PI / 180) && theta3u > (-120.0 * Math.PI / 180))
                {
                    elbow = ElbowPosition.up;
                    theta2 = theta2u;
                    theta3 = theta3u;
                }
                else if ((theta2d > (-190.0 * Math.PI / 180) && theta2d < (1.0 * Math.PI / 4)) && (theta3d < (156.0 * Math.PI / 180) && (theta3d > (-120.0 * Math.PI / 180))))
                {
                    elbow = ElbowPosition.down;
                    theta2 = theta2d;
                    theta3 = theta3d;
                }
                else
                {
                    throw new InverseKinematicsException("Out of workspace");
                }
            }
            return new double[] { theta1, theta2, theta3 };
        }


        public static double[] IKSolver(Pose DesiredPose, Vector3 EE, double[] thetaLast, ref ElbowPosition elbow, ref BasePosition basePos)
        {

            double theta1, theta2, theta3, theta4, theta5, theta6;
            double[] angles1to3 = IK1to3(DesiredPose, EE, thetaLast, ref elbow, ref basePos);
            theta1 = angles1to3[0];
            theta2 = angles1to3[1];
            theta3 = angles1to3[2];

            double[,] r = DesiredPose.getMatrix;
            double[,] T30 = new double[,] { { Math.Sin(theta2 + theta3) * Math.Cos(theta1),     -Math.Sin(theta2 + theta3) * Math.Sin(theta1),  Math.Cos(theta2 + theta3),  -400 * Math.Cos(theta2 + theta3) - 25 * Math.Sin(theta2 + theta3) - 560 * Math.Sin(theta3) }, 
                                            { Math.Cos(theta2 + theta3) * Math.Cos(theta1),     -Math.Cos(theta2 + theta3) * Math.Sin(theta1),  -Math.Sin(theta2 + theta3), 400 * Math.Sin(theta2 + theta3) - 25 * Math.Cos(theta2 + theta3) - 560 * Math.Cos(theta3) }, 
                                            { Math.Sin(theta1), Math.Cos(theta1), 0, 0 }, { 0, 0, 0, 1 } };

            double[,] T3t = SF.multiplyMatrix(T30, r);

            if (Math.Abs(T3t[1, 1]) < 1e-6 && Math.Abs(T3t[1, 0]) < 1e-6)
            {
                // Singularity! set angles on last known theta4
                theta4 = thetaLast[3];
                theta5 = 0;
                theta6 = Math.Atan2(-T3t[0, 1], T3t[2, 1]) - thetaLast[3];
            }
            else
            {
                theta4 = Math.Atan2(-T3t[2, 2], -T3t[0, 2]);
                while (Math.Abs(thetaLast[3] - theta4) > 1.0 * Math.PI / 2)
                {
                    theta4 = (thetaLast[3] < theta4) ? theta4 - Math.PI : theta4 + Math.PI;
                }
                theta6 = Math.Atan2(-T3t[1, 1], -T3t[1, 0]);
                while (Math.Abs(thetaLast[5] - theta6) > 1.0 * Math.PI / 2)
                {
                    theta6 = (thetaLast[5] < theta6) ? (theta6 - Math.PI) : (theta6 + Math.PI);
                }
                theta5 = (Math.Abs(Math.Cos(theta4)) > Math.Abs(Math.Sin(theta4))) ? Math.Atan2((-1.0 * T3t[0, 2] / (Math.Cos(theta4))), T3t[1, 2]) : Math.Atan2((-1.0 * T3t[2, 2] / (Math.Sin(theta4))), T3t[1, 2]);
            }
            if (theta4 > (185.0 * Math.PI / 180) || theta4 < (-185.0 * Math.PI / 180))
            {
                throw new InverseKinematicsException("Out of workspace");
            }
            if (theta5 > (120.0 * Math.PI / 180) || theta5 < (-120.0 * Math.PI / 180))
            {
                throw new InverseKinematicsException("Out of workspace");
            }
            if (theta5 > (350.0 * Math.PI / 180) || theta5 < (-350.0 * Math.PI / 180))
            {
                throw new InverseKinematicsException("Out of workspace");
            }
            return new double[] { theta1, theta2, theta3, theta4, theta5, theta6 };
        }
        #endregion

    }


}
