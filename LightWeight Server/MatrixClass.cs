/*
 * Author: Syed Mehroz Alam
 * Email: smehrozalam@yahoo.com
 * URL: Programming Home "http://www.geocities.com/smehrozalam/" 
 * Date: 6/20/2004
 * Time: 4:46 PM
 *
 */

using System;

namespace Mehroz
{
	/// <summary>
	/// Classes Contained:
	/// 	Matrix (version 1.1)
	/// 	MatrixException
	/// 	Fraction (Version 2.0)
	/// 	FractionException
	/// </summary>


	/// Class name: Matrix
	/// Version: 1.1
	/// Class used: Fraction
	/// Developed by: Syed Mehroz Alam
	/// Email: smehrozalam@yahoo.com
	/// URL: Programming Home "http://www.geocities.com/smehrozalam/"
	/// 
	/// What's New in version 1.1
	/// 	*	Added DeterminentFast() method
	/// 	*	Added InverseFast() method
	/// 	*	renamed ConvertToString to (override) ToString()
	/// 	*	some minor bugs fixed
	/// 
	/// Constructors:
	/// 	( Fraction[,] ):	takes 2D Fractions array	
	/// 	( int[,] ):	takes 2D integer array, convert them to fractions	
	/// 	( double[,] ):	takes 2D double array, convert them to fractions
	/// 	( int Rows, int Cols )	initializes the dimensions, indexers may be used 
	/// 							to set individual elements' values
	/// 
	/// Properties:
	/// 	Rows: read only property to get the no. of rows in the current matrix
	/// 	Cols: read only property to get the no. of columns in the current matrix
	/// 
	/// Indexers:
	/// 	[i,j] = used to set/get elements in the form of a Fraction object
	/// 
	/// Public Methods (Description is given with respective methods' definitions)
	/// 	string ToString()
	/// 	Matrix Minor(Matrix, Row,Col)
	/// 	MultiplyRow( Row, Fraction )
	/// 	MultiplyRow( Row, integer )
	/// 	MultiplyRow( Row, double )
	/// 	AddRow( TargetRow, SecondRow, Multiple)
	/// 	InterchangeRow( Row1, Row2)
	/// 	Matrix Concatenate(Matrix1, Matrix2)
	/// 	Fraction Determinent()
	/// 	Fraction DeterminentFast()
	/// 	Matrix EchelonForm()
	/// 	Matrix ReducedEchelonForm()
	/// 	Matrix Inverse()
	/// 	Matrix InverseFast()
	/// 	Matrix Adjoint()
	/// 	Matrix Transpose()
	/// 	Matrix Duplicate()
	/// 	Matrix ScalarMatrix( Rows, Cols, K )
	/// 	Matrix IdentityMatrix( Rows, Cols )
	/// 	Matrix UnitMatrix(Rows, Cols)
	/// 	Matrix NullMatrix(Rows, Cols)
	/// 
	/// Private Methods
	/// 	Fraction GetElement(int iRow, int iCol)
	/// 	SetElement(int iRow, int iCol, Fraction value)
	/// 	Negate(Matrix)
	/// 	Add(Matrix1, Matrix2)
	/// 	Multiply(Matrix1, Matrix2)
	/// 	Multiply(Matrix1, Fraction)
	/// 	Multiply(Matrix1, integer)
	/// 
	/// Operators Overloaded Overloaded
	/// 	Unary: - (negate matrix)
	/// 	Binary: 
	/// 		+,- for two matrices
	/// 		* for two matrices or one matrix with integer or fraction or double
	/// 		/ for matrix with integer or fraction or double
	/// </summary>
	public struct Mat
	{
		/// <summary>
		/// Class attributes/members
		/// </summary>
		int m_iRows;
		int m_iCols;
		double[,] m_iElement;
		

		public Mat(int[,] elements)
		{
			m_iRows=elements.GetLength(0);
			m_iCols=elements.GetLength(1);;
            m_iElement = new double[m_iRows, m_iCols];
			for(int i=0;i<elements.GetLength(0);i++)
			{
				for(int j=0;j<elements.GetLength(1);j++)
				{
                    this[i, j] = elements[i, j];
				}
			}
		}

		public Mat(double[,] elements)
		{
			m_iRows=elements.GetLength(0);
			m_iCols=elements.GetLength(1);;
            m_iElement = new double[m_iRows, m_iCols];
			for(int i=0;i<elements.GetLength(0);i++)
			{
				for(int j=0;j<elements.GetLength(1);j++)
				{
                    this[i, j] = elements[i, j];
				}
			}
		}
		
		public Mat(int iRows, int iCols)
		{
			m_iRows=iRows;
			m_iCols=iCols;
            m_iElement = new double[iRows, iCols];
		}
	
		/// <summary>
		/// Properites
		/// </summary>
		public int Rows
		{
			get	{ return m_iRows;	}
		}	
	
		public int Cols
		{
			get	{ return m_iCols;	}
		}

        public double[,] getArray
        {
            get
            {
                double[,] array = new double[m_iRows, m_iCols];
                for (int i = 0; i < m_iRows; i++)
                {
                    for (int j = 0; j < m_iCols; j++)
                    {
                        array[i, j] = GetElement(i, j);
                    }
                }
                return array;
            }
        }

        public static double[] multiplyMatrix(double[] M, double value)
        {
            int m_iRows = M.Length;
            double[] Mout = new double[m_iRows];

            for (int i = 0; i < m_iRows; i++)
            {
                    Mout[i] = M[i] * value;
            }
            return Mout;
        }

        public static double[,] multiplyMatrix(double[,] M, double value)
        {
            int m_iRows = M.GetLength(0);
            int m_iCols = M.GetLength(1);
            double[,] Mout = new double[M.GetLength(0), M.GetLength(1)];

            for (int i = 0; i < m_iRows; i++)
            {
                for (int j = 0; j < m_iCols; j++)
                {
                    Mout[i, j] = M[i, j] * value;
                }
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
                    Mout[i] += value[j] * M[i, j];
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
                    Mout[i, j] = Mat.dotProduct(Mat.getRow(M, i), Mat.getCol(value, j));
                }
            }
            return Mout;
        }

        public double[] multiplyMatrix(double[] M)
        {
            if (m_iCols != M.Length)
                throw new MatrixException("Invalid Matrix specified, not correct size");
            double[] outM = new double[m_iRows];
            for (int i = 0; i < m_iRows; i++)
            {
                for (int j = 0; j < m_iCols; j++)
                {
                    outM[i] += M[j] * m_iElement[i, j];
                }
            }
            return outM;
        }
		
		/// <summary>
		/// Indexer
		/// </summary>
        public double this[int iRow, int iCol]		// matrix's index starts at 0,0
		{
			get	{	return GetElement(iRow,iCol);	}
			set	{	SetElement(iRow,iCol,value);	}
		}

		/// <summary>
		/// Internal functions for getting/setting values
		/// </summary>
        private double GetElement(int iRow, int iCol)
		{
				if ( iRow<0 || iRow>Rows-1 || iCol<0 || iCol>Cols-1 )
					throw new MatrixException("Invalid index specified");
				return m_iElement[iRow,iCol];
		}

        private void SetElement(int iRow, int iCol, double value)
		{
				if ( iRow<0 || iRow>Rows-1 || iCol<0 || iCol>Cols-1 )
					throw new MatrixException("Invalid index specified");
				m_iElement[iRow,iCol]=value;
		}

        public double[,] doubleArray
        {
            get
            {
                double[,] newArray = new double[m_iRows,m_iCols];
                for (int i = 0; i < m_iRows; i++)
                {
                    for (int j = 0; j < m_iCols; j++)
                    {
                        newArray[i, j] = m_iElement[i, j];
                    }
                }
                return newArray;
            }
        }
		
		/// <summary>
		/// The function returns the current Matrix object as a string
		/// </summary>
		public override string ToString()
		{
			string str="";
			for (int i=0;i<this.Rows;i++)
			{
				for (int j=0;j<this.Cols;j++)
					str+=this[i,j]+"\t";
				str+="\n";
			}
			return str;
		}
		
		
		/// <summary>
		/// The function return the Minor of element[Row,Col] of a Matrix object 
		/// </summary>
		public static Mat Minor(Mat matrix, int iRow, int iCol)
		{
			Mat minor=new Mat(matrix.Rows-1, matrix.Cols-1);
			int m=0,n=0;
			for (int i=0;i<matrix.Rows;i++)
			{
				if (i==iRow)
					continue;
				n=0;
				for (int j=0;j<matrix.Cols;j++)
				{
					if (j==iCol) 
						continue;
					minor[m,n]=matrix[i,j];
					n++;	
				}
				m++;
			}
			return minor;
		}
			
			
		/// <summary>
		/// The function multiplies the given row of the current matrix object by a Fraction 
		/// </summary>
		public void MultiplyRow(int iRow, double frac)
		{
			for (int j=0;j<this.Cols;j++)
			{
				this[iRow,j]*=frac;
			}
		}

		/// <summary>
		/// The function multiplies the given row of the current matrix object by an integer
		/// </summary>
		public void MultiplyRow(int iRow, int iNo)
		{
			this.MultiplyRow(iRow, iNo);
		}

		
		/// <summary>
		/// The function adds two rows for current matrix object
		/// It performs the following calculation:
		/// iTargetRow = iTargetRow + iMultiple*iSecondRow
		/// </summary>
		public void AddRow(int iTargetRow, int iSecondRow, double iMultiple)
		{
			for (int j=0;j<this.Cols;j++)
				this[iTargetRow,j]+=(this[iSecondRow,j]*iMultiple);
		}

		/// <summary>
		/// The function interchanges two rows of the current matrix object
		/// </summary>
		public void InterchangeRow(int iRow1, int iRow2)
		{
			for (int j=0;j<this.Cols;j++)
			{
				double temp=this[iRow1,j];
				this[iRow1,j]=this[iRow2,j];
				this[iRow2,j]=temp;
			}
		}
		
		/// <summary>
		/// The function concatenates the two given matrices column-wise
		/// it can be helpful in a equation solver class where the augmented matrix is obtained by concatenation
		/// </summary>
		public static Mat Concatenate(Mat matrix1, Mat matrix2)
		{
			if (matrix1.Rows!=matrix2.Rows)
				throw new MatrixException("Concatenation not possible");
			Mat matrix=new Mat(matrix1.Rows, matrix1.Cols+matrix2.Cols );
			for (int i=0;i<matrix.Rows;i++)
				for (int j=0;j<matrix.Cols;j++)
				{
					if ( j<matrix1.Cols )
						matrix[i,j]=matrix1[i,j];
					else 
						matrix[i,j]=matrix2[i,j-matrix1.Cols];
				}	
			return matrix;
		}

		/// <summary>
		/// The function returns the determinent of the current Matrix object as Fraction
		/// It computes the determinent by reducing the matrix to reduced echelon form using row operations
		/// The function is very fast and efficient but may raise overflow exceptions in some cases.
		/// In such cases use the Determinent() function which computes determinent in the traditional 
		/// manner(by using minors)
		/// </summary>
		public double DeterminentFast()
		{
			if (this.Rows!=this.Cols)
				throw new MatrixException("Determinent of a non-square matrix doesn't exist");
			double det=1;
			try
			{
				Mat ReducedEchelonMatrix=this.Duplicate();
				for (int i=0;i<this.Rows;i++)
				{	
					if (ReducedEchelonMatrix[i,i]==0)	// if diagonal entry is zero, 
						for (int j=i+1;j<ReducedEchelonMatrix.Rows;j++)
							if (ReducedEchelonMatrix[j,i]!=0)	 //check if some below entry is non-zero
							{
								ReducedEchelonMatrix.InterchangeRow(i,j);	// then interchange the two rows
								det*=-1;	//interchanging two rows negates the determinent
							}

					det*=ReducedEchelonMatrix[i,i];
					ReducedEchelonMatrix.MultiplyRow(i, 1.0/ReducedEchelonMatrix[i,i]);
				
					for (int j=i+1;j<ReducedEchelonMatrix.Rows;j++)
					{
						ReducedEchelonMatrix.AddRow( j, i, -ReducedEchelonMatrix[j,i]);
					}
					for (int j=i-1;j>=0;j--)
					{
						ReducedEchelonMatrix.AddRow( j, i, -ReducedEchelonMatrix[j,i]);
					}
				}
				return det;
			}
			catch(Exception)
			{
                Determinent();
				throw new MatrixException("Determinent of the given matrix could not be calculated");
			}
		}
	
		/// <summary>
		/// The function returns the determinent of the current Matrix object as Fraction
		/// It computes the determinent in the traditional way (i.e. using minors)
		/// It can be much slower(due to recursion) if the given matrix has order greater than 6
		/// Try using DeterminentFast() function if the order of matrix is greater than 6
		/// </summary>
		public double Determinent()
		{
			return Determinent(this);
		}
		
		/// <summary>
		/// The helper function for the above Determinent() method
		/// it calls itself recursively and computes determinent using minors
		/// </summary>
		private double Determinent(Mat matrix)
		{
			double det=0;
			if (matrix.Rows!=matrix.Cols)
				throw new MatrixException("Determinent of a non-square matrix doesn't exist");
			if (matrix.Rows==1)
				return matrix[0,0];
			for (int j=0;j<matrix.Cols;j++)
				det+=(matrix[0,j]*Determinent(Mat.Minor(matrix, 0,j))*(int)System.Math.Pow(-1,0+j));
			return det;
		}


		/// <summary>
		/// The function returns the Echelon form of the current matrix
		/// </summary>
		public Mat EchelonForm()
		{
			try
			{
				Mat EchelonMatrix=this.Duplicate();
				for (int i=0;i<this.Rows;i++)
				{	
					if (EchelonMatrix[i,i]==0)	// if diagonal entry is zero, 
						for (int j=i+1;j<EchelonMatrix.Rows;j++)
							if (EchelonMatrix[j,i]!=0)	 //check if some below entry is non-zero
								EchelonMatrix.InterchangeRow(i,j);	// then interchange the two rows
					if (EchelonMatrix[i,i]==0)	// if not found any non-zero diagonal entry
						continue;	// increment i;
					if ( EchelonMatrix[i,i]!=1)	// if diagonal entry is not 1 , 	
						for (int j=i+1;j<EchelonMatrix.Rows;j++)
							if (EchelonMatrix[j,i]==1)	 //check if some below entry is 1
								EchelonMatrix.InterchangeRow(i,j);	// then interchange the two rows
					EchelonMatrix.MultiplyRow(i, 1.0/EchelonMatrix[i,i]);
					for (int j=i+1;j<EchelonMatrix.Rows;j++)
						EchelonMatrix.AddRow( j, i, -EchelonMatrix[j,i]);
				}
				return EchelonMatrix;
			}
			catch(Exception)
			{
				throw new MatrixException("Matrix can not be reduced to Echelon form");
			}
		}

		/// <summary>
		/// The function returns the reduced echelon form of the current matrix
		/// </summary>
		public Mat ReducedEchelonForm()
		{
			try
			{
				Mat ReducedEchelonMatrix=this.Duplicate();
				for (int i=0;i<this.Rows;i++)
				{	
					if (ReducedEchelonMatrix[i,i]==0)	// if diagonal entry is zero, 
						for (int j=i+1;j<ReducedEchelonMatrix.Rows;j++)
							if (ReducedEchelonMatrix[j,i]!=0)	 //check if some below entry is non-zero
								ReducedEchelonMatrix.InterchangeRow(i,j);	// then interchange the two rows
					if (ReducedEchelonMatrix[i,i]==0)	// if not found any non-zero diagonal entry
						continue;	// increment i;
					if ( ReducedEchelonMatrix[i,i]!=1)	// if diagonal entry is not 1 , 	
						for (int j=i+1;j<ReducedEchelonMatrix.Rows;j++)
							if ( ReducedEchelonMatrix[j,i]==1 )	 //check if some below entry is 1
								ReducedEchelonMatrix.InterchangeRow(i,j);	// then interchange the two rows
					ReducedEchelonMatrix.MultiplyRow(i, 1.0/ReducedEchelonMatrix[i,i]);
					for (int j=i+1;j<ReducedEchelonMatrix.Rows;j++)
						ReducedEchelonMatrix.AddRow( j, i, -ReducedEchelonMatrix[j,i]);
					for (int j=i-1;j>=0;j--)
						ReducedEchelonMatrix.AddRow( j, i, -ReducedEchelonMatrix[j,i]);
				}
				return ReducedEchelonMatrix;
			}
			catch(Exception)
			{
				throw new MatrixException("Matrix can not be reduced to Echelon form");
			}
		}

		/// <summary>
		/// The function returns the inverse of the current matrix using Reduced Echelon Form method
		/// The function is very fast and efficient but may raise overflow exceptions in some cases.
		/// In such cases use the Inverse() method which computes inverse in the traditional way(using adjoint).
		/// </summary>
		public Mat InverseFast()
		{
			if ( this.DeterminentFast()==0 )
				throw new MatrixException("Inverse of a singular matrix is not possible");
			try
			{
				Mat IdentityMatrix=Mat.IdentityMatrix(this.Rows, this.Cols);
				Mat ReducedEchelonMatrix=this.Duplicate();
				for (int i=0;i<this.Rows;i++)
				{	
					if (ReducedEchelonMatrix[i,i]==0)	// if diagonal entry is zero, 
						for (int j=i+1;j<ReducedEchelonMatrix.Rows;j++)
							if (ReducedEchelonMatrix[j,i]!=0)	 //check if some below entry is non-zero
							{
								ReducedEchelonMatrix.InterchangeRow(i,j);	// then interchange the two rows
								IdentityMatrix.InterchangeRow(i,j);	// then interchange the two rows
							}
					IdentityMatrix.MultiplyRow(i, 1.0/ReducedEchelonMatrix[i,i]);
					ReducedEchelonMatrix.MultiplyRow(i, 1.0/ReducedEchelonMatrix[i,i]);
				
					for (int j=i+1;j<ReducedEchelonMatrix.Rows;j++)
					{
						IdentityMatrix.AddRow( j, i, -ReducedEchelonMatrix[j,i]);
						ReducedEchelonMatrix.AddRow( j, i, -ReducedEchelonMatrix[j,i]);
					}
					for (int j=i-1;j>=0;j--)
					{
						IdentityMatrix.AddRow( j, i, -ReducedEchelonMatrix[j,i]);
						ReducedEchelonMatrix.AddRow( j, i, -ReducedEchelonMatrix[j,i]);
					}
				}
				return IdentityMatrix;
			}
			catch(Exception)
			{
				throw new MatrixException("Inverse of the given matrix could not be calculated");
			}
		}

        /// <summary>
        /// The function returns the inverse of the current matrix using Reduced Echelon Form method
        /// The function is very fast and efficient but may raise overflow exceptions in some cases.
        /// In such cases use the Inverse() method which computes inverse in the traditional way(using adjoint).
        /// </summary>
        public Mat InverseFast(double error)
        {
            if (this.DeterminentFast() < error)
                return this.Inverse(error);
            try
            {
                Mat IdentityMatrix = Mat.IdentityMatrix(this.Rows, this.Cols);
                Mat ReducedEchelonMatrix = this.Duplicate();
                for (int i = 0; i < this.Rows; i++)
                {
                    if (ReducedEchelonMatrix[i, i] == 0)	// if diagonal entry is zero, 
                        for (int j = i + 1; j < ReducedEchelonMatrix.Rows; j++)
                            if (ReducedEchelonMatrix[j, i] != 0)	 //check if some below entry is non-zero
                            {
                                ReducedEchelonMatrix.InterchangeRow(i, j);	// then interchange the two rows
                                IdentityMatrix.InterchangeRow(i, j);	// then interchange the two rows
                            }
                    IdentityMatrix.MultiplyRow(i, 1.0/ReducedEchelonMatrix[i, i]);
                    ReducedEchelonMatrix.MultiplyRow(i, 1.0/ReducedEchelonMatrix[i, i]);

                    for (int j = i + 1; j < ReducedEchelonMatrix.Rows; j++)
                    {
                        IdentityMatrix.AddRow(j, i, -ReducedEchelonMatrix[j, i]);
                        ReducedEchelonMatrix.AddRow(j, i, -ReducedEchelonMatrix[j, i]);
                    }
                    for (int j = i - 1; j >= 0; j--)
                    {
                        IdentityMatrix.AddRow(j, i, -ReducedEchelonMatrix[j, i]);
                        ReducedEchelonMatrix.AddRow(j, i, -ReducedEchelonMatrix[j, i]);
                    }
                }
                return IdentityMatrix;
            }
            catch (Exception)
            {
                return this.Inverse(error);
            }
        }
		
		/// <summary>
		/// The function returns the inverse of the current matrix in the traditional way(by adjoint method)
		/// It can be much slower if the given matrix has order greater than 6
		/// Try using InverseFast() function if the order of matrix is greater than 6
		/// </summary>
		public Mat Inverse()
		{
			if ( this.Determinent()==0 )
				throw new MatrixException("Inverse of a singular matrix is not possible");
			return ( this.Adjoint()/this.Determinent() );
		}

        /// <summary>
        /// The function returns the inverse of the current matrix in the traditional way(by adjoint method)
        /// It can be much slower if the given matrix has order greater than 6
        /// Try using InverseFast() function if the order of matrix is greater than 6
        /// </summary>
        public Mat Inverse(double error)
        {
            double det = this.Determinent();
            return this.Adjoint() / ((det < error) ? error : det);
        }

		/// <summary>
		/// The function returns the adjoint of the current matrix
		/// </summary>
		public Mat Adjoint()
		{
			if (this.Rows!=this.Cols)
				throw new MatrixException("Adjoint of a non-square matrix does not exists");
			Mat AdjointMatrix=new Mat(this.Rows, this.Cols);
			for (int i=0;i<this.Rows;i++)
				for (int j=0;j<this.Cols;j++)
					AdjointMatrix[i,j]=Math.Pow(-1,i+j)*( Minor(this, i,j).Determinent() );
			AdjointMatrix=AdjointMatrix.Transpose();
			return AdjointMatrix;
		}
		
		/// <summary>
		/// The function returns the transpose of the current matrix
		/// </summary>
		public Mat Transpose()
		{
			Mat TransposeMatrix=new Mat(this.Cols, this.Rows);
			for (int i=0;i<TransposeMatrix.Rows;i++)
				for (int j=0;j<TransposeMatrix.Cols;j++)
					TransposeMatrix[i,j]=this[j,i];
			return TransposeMatrix;
		}
		
		/// <summary>
		/// The function duplicates the current Matrix object
		/// </summary>
		public Mat Duplicate()
		{
			Mat matrix=new Mat(Rows,Cols);
			for (int i=0;i<Rows;i++)
				for (int j=0;j<Cols;j++)
					matrix[i,j]=this[i,j];
			return matrix;
		}
	
		/// <summary>
		/// The function returns a Scalar Matrix of dimension ( Row x Col ) and scalar K
		/// </summary>
		public static Mat ScalarMatrix(int iRows, int iCols, int K)
		{
			double zero=0;
			double scalar=K;
			Mat matrix=new Mat(iRows,iCols);
			for (int i=0;i<iRows;i++)
				for (int j=0;j<iCols;j++)
				{
					if (i==j)
						matrix[i,j]=scalar;
					else
						matrix[i,j]=zero;
				}
			return matrix;
		}

		/// <summary>
		/// The function returns an identity matrix of dimensions ( Row x Col )
		/// </summary>
		public static Mat IdentityMatrix(int iRows, int iCols)
		{
			return ScalarMatrix(iRows, iCols, 1);
		}

		/// <summary>
		/// The function returns a Unit Matrix of dimension ( Row x Col )
		/// </summary>
		public static Mat UnitMatrix(int iRows, int iCols)
{
			double temp=1;
			Mat matrix=new Mat(iRows,iCols);
			for (int i=0;i<iRows;i++)
				for (int j=0;j<iCols;j++)
					matrix[i,j]=temp;
			return matrix;
		}
		
		/// <summary>
		/// The function returns a Null Matrix of dimension ( Row x Col )
		/// </summary>
		public static Mat NullMatrix(int iRows, int iCols)
		{
			double temp=0;
			Mat matrix=new Mat(iRows,iCols);
			for (int i=0;i<iRows;i++)
				for (int j=0;j<iCols;j++)
					matrix[i,j]=temp;
			return matrix;
		}
		
		/// <summary>
		/// Operators for the Matrix object
		/// includes -(unary), and binary opertors such as +,-,*,/
		/// </summary>
		public static Mat operator -(Mat matrix)
		{	return Mat.Negate(matrix);	}
		
		public static Mat operator +(Mat matrix1, Mat matrix2)
		{	return Mat.Add(matrix1, matrix2);	}
		
		public static Mat operator -(Mat matrix1, Mat matrix2)
		{	return Mat.Add(matrix1, -matrix2);	}
		
		public static Mat operator *(Mat matrix1, Mat matrix2)
		{	return Mat.Multiply(matrix1, matrix2);	}
	
		public static Mat operator *(Mat matrix1, int iNo)
		{	return Mat.Multiply(matrix1, iNo);	}
	
		public static Mat operator *(Mat matrix1, double dbl)
		{	return Mat.Multiply(matrix1, dbl);	}
	
		public static Mat operator *(int iNo, Mat matrix1)
		{	return Mat.Multiply(matrix1, iNo);	}
	
		public static Mat operator *(double dbl, Mat matrix1)
		{	return Mat.Multiply(matrix1, dbl);	}

		public static Mat operator /(Mat matrix1, int iNo)
		{	return Mat.Multiply(matrix1, 1.0/iNo);	}
		
		public static Mat operator /(Mat matrix1, double dbl)
		{	return Mat.Multiply(matrix1, 1.0/dbl);	}
		
		
		/// <summary>
		/// Internal Fucntions for the above operators
		/// </summary>
		private static Mat Negate(Mat matrix)
		{
			return Mat.Multiply(matrix,-1);
		}
		
		private static Mat Add(Mat matrix1, Mat matrix2)
		{
			if (matrix1.Rows!=matrix2.Rows || matrix1.Cols!=matrix2.Cols)
				throw new MatrixException("Operation not possible");
			Mat result=new Mat(matrix1.Rows, matrix1.Cols);
			for (int i=0;i<result.Rows;i++)
				for (int j=0;j<result.Cols;j++)
					result[i,j]=matrix1[i,j]+matrix2[i,j];
			return result;
		}
		
		private static Mat Multiply(Mat matrix1, Mat matrix2)
		{
			if ( matrix1.Cols!=matrix2.Rows )
				throw new MatrixException("Operation not possible");
			Mat result=Mat.NullMatrix(matrix1.Rows,matrix2.Cols);
			for (int i=0;i<result.Rows;i++)
				for (int j=0;j<result.Cols;j++)
					for (int k=0;k<matrix1.Cols;k++)
						result[i,j]+=matrix1[i,k]*matrix2[k,j];
			return result;						
		}
		
		private static Mat Multiply(Mat matrix, int iNo)
		{
			Mat result=new Mat(matrix.Rows,matrix.Cols);
			for (int i=0;i<matrix.Rows;i++)
				for (int j=0;j<matrix.Cols;j++)
					result[i,j]=matrix[i,j]*iNo;
			return result;
		}
		
		private static Mat Multiply(Mat matrix, double frac)
		{
			Mat result=new Mat(matrix.Rows,matrix.Cols);
			for (int i=0;i<matrix.Rows;i++)
				for (int j=0;j<matrix.Cols;j++)
					result[i,j]=matrix[i,j]*frac;
			return result;
		}
	
	}	//end class Matrix
	
	/// <summary>
	/// Exception class for Matrix class, derived from System.Exception
	/// </summary>
	public class MatrixException : Exception
	{
		public MatrixException() : base()
		{}
	
		public MatrixException(string Message) : base(Message)
		{}
		
		public MatrixException(string Message, Exception InnerException) : base(Message, InnerException)
		{}
	}	// end class MatrixException

	

    /*
	/// <summary>
	/// Class name: Fraction
	/// Developed by: Syed Mehroz Alam
	/// Email: smehrozalam@yahoo.com
	/// URL: Programming Home "http://www.geocities.com/smehrozalam/"
	/// Version: 2.0
	/// 
	/// What's new in version 2.0:
	/// 	*	Changed Numerator and Denominator from Int32(integer) to Int64(long) for increased range
	/// 	*	renamed ConvertToString() to (overloaded) ToString()
	/// 	*	added the capability of detecting/raising overflow exceptions
	/// 	*	Fixed the bug that very small numbers e.g. 0.00000001 could not be converted to fraction
	/// 	*	Other minor bugs fixed
	/// 
	/// Properties:
	/// 	Numerator: Set/Get value for Numerator
	/// 	Denominator:  Set/Get value for Numerator
	/// 	Value:  Set an integer value for the fraction
	/// 
	/// Constructors:
	/// 	no arguments:	initializes fraction as 0/1
	/// 	(Numerator, Denominator): initializes fraction with the given numerator and denominator values
	/// 	(integer):	initializes fraction with the given integer value
	/// 	(long):	initializes fraction with the given long value
	/// 	(double):	initializes fraction with the given double value
	/// 	(string):	initializes fraction with the given string value
	/// 				the string can be an in the form of and integer, double or fraction.
	/// 				e.g it can be like "123" or "123.321" or "123/456"
	/// 
	/// Public Methods (Description is given with respective methods' definitions)
	/// 	(override) string ToString(Fraction)
	/// 	Fraction ToFraction(string)
	/// 	Fraction ToFraction(double)
	/// 	double ToDouble(Fraction)
	/// 	Fraction Duplicate()
	/// 	Fraction Inverse(integer)
	/// 	Fraction Inverse(Fraction)
	/// 	ReduceFraction(Fraction)
	/// 	Equals(object)
	/// 	GetHashCode()
	/// 
	///	Private Methods (Description is given with respective methods' definitions)
	/// 	Initialize(Numerator, Denominator)
	/// 	Fraction Negate(Fraction)
	/// 	Fraction Add(Fraction1, Fraction2)
	/// 
	/// Overloaded Operators (overloaded for Fractions, Integers and Doubles)
	/// 	Unary: -
	/// 	Binary: +,-,*,/ 
	/// 	Relational and Logical Operators: ==,!=,<,>,<=,>=
	/// 
	/// Overloaded user-defined conversions
	/// 	Implicit:	From double/long/string to Fraction
	/// 	Explicit:	From Fraction to double/string
	/// </summary>
	public class Fraction
	{
		/// <summary>
		/// Class attributes/members
		/// </summary>
		long m_iNumerator;
		long m_iDenominator;
		
		/// <summary>
		/// Constructors
		/// </summary>
		public Fraction()
		{
			Initialize(0,1);
		}
	
		public Fraction(long iWholeNumber)
		{
			Initialize(iWholeNumber, 1);
		}
	
		public Fraction(double dDecimalValue)
		{
			Fraction temp=ToFraction(dDecimalValue);
			Initialize(temp.Numerator, temp.Denominator);
		}
		
		public Fraction(string strValue)
		{
			Fraction temp=ToFraction(strValue);
			Initialize(temp.Numerator, temp.Denominator);
		}
		
		public Fraction(long iNumerator, long iDenominator)
		{
			Initialize(iNumerator, iDenominator);
		}
		
		/// <summary>
		/// Internal function for constructors
		/// </summary>
		private void Initialize(long iNumerator, long iDenominator)
		{
			Numerator=iNumerator;
			Denominator=iDenominator;
			ReduceFraction(this);
		}
	
		/// <summary>
		/// Properites
		/// </summary>
		public long Denominator
		{
			get
			{	return m_iDenominator;	}
			set
			{
				if (value!=0)
					m_iDenominator=value;
				else
					throw new FractionException("Denominator cannot be assigned a ZERO Value");
			}
		}
	
		public long Numerator
		{
			get	
			{	return m_iNumerator;	}
			set
			{	m_iNumerator=value;	}
		}
	
		public long Value
		{
			set
			{	m_iNumerator=value;
				m_iDenominator=1;	}
		}
	
		
		/// <summary>
		/// The function takes a Fraction object and returns its value as double
		/// </summary>
		public static double ToDouble(Fraction frac)
		{
			return ( (double)frac.Numerator/frac.Denominator );
		}

		/// <summary>
		/// The function returns the current Fraction object as double
		/// </summary>
		public double ToDouble()
		{
			return ( (double)this.Numerator/this.Denominator );
		}

		/// <summary>
		/// The function returns the current Fraction object as a string
		/// </summary>
		public override string ToString()
		{
			string str;
			if ( this.Denominator==1 )
				str=this.Numerator.ToString();
			else
				str=this.Numerator + "/" + this.Denominator;
			return str;
		}
		/// <summary>
		/// The function takes an string as an argument and returns its corresponding reduced fraction
		/// the string can be an in the form of and integer, double or fraction.
		/// e.g it can be like "123" or "123.321" or "123/456"
		/// </summary>
		public static Fraction ToFraction(string strValue)
		{
			int i;
			for (i=0;i<strValue.Length;i++)
				if (strValue[i]=='/')
					break;
			
			if (i==strValue.Length)		// if string is not in the form of a fraction
				// then it is double or integer
				return ( ToFraction( Convert.ToDouble(strValue) ) );
			
			// else string is in the form of Numerator/Denominator
			long iNumerator=Convert.ToInt64(strValue.Substring(0,i));
			long iDenominator=Convert.ToInt64(strValue.Substring(i+1));
			return new Fraction(iNumerator, iDenominator);
		}
		
		
		/// <summary>
		/// The function takes a floating point number as an argument 
		/// and returns its corresponding reduced fraction
		/// </summary>
		public static Fraction ToFraction(double dValue)
		{
			try
			{
				checked
				{
					Fraction frac;
					if (dValue%1==0)	// if whole number
					{
						frac=new Fraction( (long) dValue );
					}
					else
					{
						double dTemp=dValue;
						long iMultiple=1;
						string strTemp=dValue.ToString();
                        while (strTemp.IndexOf("E") > 0 || strTemp.IndexOf("e") > 0)	// if in the form like 12E-9
						{
							dTemp*=10;
							iMultiple*=10;
							strTemp=dTemp.ToString();
						}
						int i=0;
						while ( strTemp[i]!='.' )
							i++;
						int iDigitsAfterDecimal=strTemp.Length-i-1;
						while ( iDigitsAfterDecimal>0  )
						{
							dTemp*=10;
							iMultiple*=10;
							iDigitsAfterDecimal--;
						}
						frac=new Fraction( (int)Math.Round(dTemp) , iMultiple );
					}
					return frac;
				}
			}
			catch(OverflowException)
			{
                return ToFraction((double)(Math.Round(dValue*1e6)*1e-6));
                throw new FractionException("Conversion not possible due to overflow of number: " + dValue.ToString() + "Rounded to 1E-6");
			}
			catch(Exception)
			{
				throw new FractionException("Conversion not possible");
			}
		}

		/// <summary>
		/// The function replicates current Fraction object
		/// </summary>
		public Fraction Duplicate()
		{
			Fraction frac=new Fraction();
			frac.Numerator=Numerator;
			frac.Denominator=Denominator;
			return frac;
		}

		/// <summary>
		/// The function returns the inverse of a Fraction object
		/// </summary>
		public static Fraction Inverse(Fraction frac1)
		{
			if (frac1.Numerator==0)
				throw new FractionException("Operation not possible (Denominator cannot be assigned a ZERO Value)");
	
			long iNumerator=frac1.Denominator;
			long iDenominator=frac1.Numerator;
			return ( new Fraction(iNumerator, iDenominator));
		}	
	

		/// <summary>
		/// Operators for the Fraction object
		/// includes -(unary), and binary opertors such as +,-,*,/
		/// also includes relational and logical operators such as ==,!=,<,>,<=,>=
		/// </summary>
		public static Fraction operator -(Fraction frac1)
		{	return ( Negate(frac1) );	}
	                              
		public static Fraction operator +(Fraction frac1, Fraction frac2)
		{	return ( Add(frac1 , frac2) );	}
	
		public static Fraction operator +(int iNo, Fraction frac1)
		{	return ( Add(frac1 , new Fraction(iNo) ) );	}
	
		public static Fraction operator +(Fraction frac1, int iNo)
		{	return ( Add(frac1 , new Fraction(iNo) ) );	}

		public static Fraction operator +(double dbl, Fraction frac1)
		{	return ( Add(frac1 , Fraction.ToFraction(dbl) ) );	}
	
		public static Fraction operator +(Fraction frac1, double dbl)
		{	return ( Add(frac1 , Fraction.ToFraction(dbl) ) );	}
	
		public static Fraction operator -(Fraction frac1, Fraction frac2)
		{	return ( Add(frac1 , -frac2) );	}
	
		public static Fraction operator -(int iNo, Fraction frac1)
		{	return ( Add(-frac1 , new Fraction(iNo) ) );	}
	
		public static Fraction operator -(Fraction frac1, int iNo)
		{	return ( Add(frac1 , -(new Fraction(iNo)) ) );	}

		public static Fraction operator -(double dbl, Fraction frac1)
		{	return ( Add(-frac1 , Fraction.ToFraction(dbl) ) );	}
	
		public static Fraction operator -(Fraction frac1, double dbl)
		{	return ( Add(frac1 , -Fraction.ToFraction(dbl) ) );	}
	
		public static Fraction operator *(Fraction frac1, Fraction frac2)
		{	return ( Multiply(frac1 , frac2) );	}
	
		public static Fraction operator *(long iNo, Fraction frac1)
		{	return ( Multiply(frac1 , new Fraction(iNo) ) );	}
	
		public static Fraction operator *(Fraction frac1, int iNo)
		{	return ( Multiply(frac1 , new Fraction(iNo) ) );	}
	
		public static Fraction operator *(double dbl, Fraction frac1)
		{	return ( Multiply(frac1 , Fraction.ToFraction(dbl) ) );	}
	
		public static Fraction operator *(Fraction frac1, double dbl)
		{	return ( Multiply(frac1 , Fraction.ToFraction(dbl) ) );	}
	
		public static Fraction operator /(Fraction frac1, Fraction frac2)
		{	return ( Multiply( frac1 , Inverse(frac2) ) );	}
	
		public static Fraction operator /(int iNo, Fraction frac1)
		{	return ( Multiply( Inverse(frac1) , new Fraction(iNo) ) );	}
	
		public static Fraction operator /(Fraction frac1, int iNo)
		{	return ( Multiply( frac1 , Inverse(new Fraction(iNo)) ) );	}
	
		public static Fraction operator /(double dbl, Fraction frac1)
		{	return ( Multiply( Inverse(frac1) , Fraction.ToFraction(dbl) ) );	}
	
		public static Fraction operator /(Fraction frac1, double dbl)
		{	return ( Multiply( frac1 , Fraction.Inverse( Fraction.ToFraction(dbl) ) ) );	}

		public static bool operator ==(Fraction frac1, Fraction frac2)
		{	return frac1.Equals(frac2);		}

		public static bool operator !=(Fraction frac1, Fraction frac2)
		{	return ( !frac1.Equals(frac2) );	}

		public static bool operator ==(Fraction frac1, int iNo)
		{	return frac1.Equals( new Fraction(iNo));	}

		public static bool operator !=(Fraction frac1, int iNo)
		{	return ( !frac1.Equals( new Fraction(iNo)) );	}
		
		public static bool operator ==(Fraction frac1, double dbl)
		{	return frac1.Equals( new Fraction(dbl));	}

		public static bool operator !=(Fraction frac1, double dbl)
		{	return ( !frac1.Equals( new Fraction(dbl)) );	}
		
		public static bool operator<(Fraction frac1, Fraction frac2)
		{	return frac1.Numerator * frac2.Denominator < frac2.Numerator * frac1.Denominator;	}

		public static bool operator>(Fraction frac1, Fraction frac2)
		{	return frac1.Numerator * frac2.Denominator > frac2.Numerator * frac1.Denominator;	}

		public static bool operator<=(Fraction frac1, Fraction frac2)
		{	return frac1.Numerator * frac2.Denominator <= frac2.Numerator * frac1.Denominator;	}
		
		public static bool operator>=(Fraction frac1, Fraction frac2)
		{	return frac1.Numerator * frac2.Denominator >= frac2.Numerator * frac1.Denominator;	}
		
		/// <summary>
		/// checks whether two fractions are equal
		/// </summary>
		public override bool Equals(object obj)
		{
			Fraction frac=(Fraction)obj;
			return ( Numerator==frac.Numerator && Denominator==frac.Denominator);
		}
		
		/// <summary>
		/// returns a hash code for this fraction
		/// </summary>
   		public override int GetHashCode()
   		{
			return ( Convert.ToInt32((Numerator ^ Denominator) & 0xFFFFFFFF) ) ;
		}

		/// <summary>
		/// internal function for negation
		/// </summary>
		private static Fraction Negate(Fraction frac1)
		{
			long iNumerator=-frac1.Numerator;
			long iDenominator=frac1.Denominator;
			return ( new Fraction(iNumerator, iDenominator) );

		}	

		/// <summary>
		/// internal functions for binary operations
		/// </summary>
		private static Fraction Add(Fraction frac1, Fraction frac2)
		{
			try
			{
				checked
				{
					long iNumerator=frac1.Numerator*frac2.Denominator + frac2.Numerator*frac1.Denominator;
					long iDenominator=frac1.Denominator*frac2.Denominator;
					return ( new Fraction(iNumerator, iDenominator) );
				}
			}
			catch(OverflowException)
			{
				throw new FractionException("Overflow occurred while performing arithemetic operation");
			}
			catch(Exception)
			{
				throw new FractionException("An error occurred while performing arithemetic operation");
			}
		}
	
		private static Fraction Multiply(Fraction frac1, Fraction frac2)
		{
			try
			{
				checked
				{
					long iNumerator=frac1.Numerator*frac2.Numerator;
					long iDenominator=frac1.Denominator*frac2.Denominator;
					return ( new Fraction(iNumerator, iDenominator) );
				}
			}
			catch(OverflowException)
			{
				throw new FractionException("Overflow occurred while performing arithemetic operation");
			}
			catch(Exception)
			{
				throw new FractionException("An error occurred while performing arithemetic operation");
			}
		}

		/// <summary>
		/// The function returns GCD of two numbers (used for reducing a Fraction)
		/// </summary>
		private static long GCD(long iNo1, long iNo2)
		{
			// take absolute values
			if (iNo1 < 0) iNo1 = -iNo1;
			if (iNo2 < 0) iNo2 = -iNo2;
			
			do
			{
				if (iNo1 < iNo2)
				{
					long tmp = iNo1;  // swap the two operands
					iNo1 = iNo2;
					iNo2 = tmp;
				}
				iNo1 = iNo1 % iNo2;
			} while (iNo1 != 0);
			return iNo2;
		}
	
		/// <summary>
		/// The function reduces(simplifies) a Fraction object by dividing both its numerator 
		/// and denominator by their GCD
		/// </summary>
		public static void ReduceFraction(Fraction frac)
		{
			try
			{
				if (frac.Numerator==0)
				{
					frac.Denominator=1;
					return;
				}
				
				long iGCD=GCD(frac.Numerator, frac.Denominator);
				frac.Numerator/=iGCD;
				frac.Denominator/=iGCD;
				
				if ( frac.Denominator<0 )	// if -ve sign in denominator
				{
					//pass -ve sign to numerator
					frac.Numerator*=-1;
					frac.Denominator*=-1;	
				}
			} // end try
			catch(Exception exp)
			{
				throw new FractionException("Cannot reduce Fraction: " + exp.Message);
			}
		}
			
	}	//end class Fraction

	/// <summary>
	/// Exception class for Fraction, derived from System.Exception
	/// </summary>
	public class FractionException : Exception
	{
		public FractionException() : base()
		{}
	
		public FractionException(string Message) : base(Message)
		{}
		
		public FractionException(string Message, Exception InnerException) : base(Message, InnerException)
		{}
	}	//end class FractionException
     * 
     * 
     * 
     * 
     * 
	*/
}	//end namespace Mehroz
