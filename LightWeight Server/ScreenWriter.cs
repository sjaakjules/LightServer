using Microsoft.Xna.Framework;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace LightWeight_Server
{
    class ScreenWriter
    {
        object dataWriteLock = new object();

        bool _isConnected;
        double[] _angles = new double[6];
        Pose _Position, _Velocity, _Acceleration, _DesiredPose;
        Vector3 _EndEffector;
        Dictionary<Guid, StringBuilder> DataStrings;
        Dictionary<Guid, string> FileName;
        readonly Guid DataLogger;
        
         

        public ScreenWriter()
        {
            DataLogger = Guid.NewGuid();
            DataStrings.Add(DataLogger, new StringBuilder());
            // Initialises the screen variables.
        }


        public void Loop()
        {
            foreach (KeyValuePair<Guid, StringBuilder> item in DataStrings)
            {
                StreamWriter file = new StreamWriter(FileName[item.Key],true);
                if (item.Key.Equals(DataLogger))
                {
                    lock (dataWriteLock)
                    {
                        file.WriteLine(DataStrings[item.Key]);
                        DataStrings[item.Key].Clear();
                    }
                }
                file.Flush();
                file.Close();
            }
        }

        /// <summary>
        /// Constructs the information to be updated giving unique ID used to populate information later.
        /// </summary>
        /// <param name="data"></param> Data type which is overlaoded for correct display settings
        /// <param name="message"></param> String message to display
        /// <returns></returns>
        public Guid displayInformation(Object data, string message)
        {
            // Sets up data to display with unique ID and populates a dictionary
            return Guid.Empty;
        }

        /// <summary>
        /// Updates information to display to screen. This must be inforamtion which is regerested at compilation
        /// </summary>
        /// <param name="data"></param>
        /// <param name="ID"></param>
        /// <returns></returns>
        public bool updateInfo(Object data, Guid ID)
        {
            // Updates data using an ID tag to know what to do with it.
            return false;
        }

        private void UpdateScreen()
        {
            // Clears and updates screen with new information
        }

        /// <summary>
        /// Public funtion to update Debug log when catch statement are triggered.
        /// </summary>
        /// <param name="Error"></param>Error trigger
        /// <param name="msg"></param>Custom string to give specific information such as values which failed and why.
        public void Debug(Exception Error, string msg)
        {
            // Writes errors with time stamp to log file.
            // Notify error occured to server and external program if required
        }

        /// <summary>
        /// Public funtion to update Error log when catch statement are triggered. 
        /// Will link important errors to server display or external server such as collisions or automatic trajectory changes.
        /// </summary>
        /// <param name="Error"></param>Error trigger
        /// <param name="msg"></param>Custom string to give specific information such as values which failed and why.
        public void Error(Exception Error, string msg)
        {
            // Writes errors with time stamp to log file.
            // Notify error occured to server and external program if required
        }

        /// <summary>
        /// Creates a stream/file writer to save data in csv format. Returns the index of the data and links with columns of the csv document.
        /// </summary>
        /// <param name="DataName"></param>String of the name of the file to be created.
        /// <param name="data"></param>Object of data to be written, TODO: overload with known types.
        /// <returns></returns>
        public int CreateData(string DataName, object data)
        {
            // Dummy method to create data writer.
            // Overload with different inputs, this will set the array and link index with columns required.
            // RETURN int of index for WriteData reference
            // TODO: Don't use -1 but use errors and event handeler.
            return -1;
        }



        /// <summary>
        /// Adds information to be printed to csv file. Returns false if data file was not created.
        /// </summary>
        /// <param name="DataName"></param>String of the name of the file to be created.
        /// <param name="time"></param>Time of the data to be added / captured. Expressed in ms and will save in bins of 4ms
        /// <param name="index"></param>ID of the data to be written as per CreateData result.
        /// <param name="data"></param>Object of data to be written, TODO: overload with known types.
        /// <returns></returns>
        public bool WriteData(string DataName, double time, int index, object data)
        {
            // Dummy method to write data to file. 
            // Overloaded with variouse inputs such as Pose, double, double[,] ect, 
            // RETURN bool if data was added,
            // TODO: Don't use bool but use errors and event handeler.
            return false;
        }
    }
}
