//----------------------------------------------------------------------------
//
// Copyright 2020 by Northern Digital Inc.
// All Rights Reserved
//
//----------------------------------------------------------------------------
// By using this Sample Code the licensee agrees to be bound by the
// provisions of the agreement found in the LICENSE.txt file.

// System
using System;
using System.Threading.Tasks;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using CsvHelper;
using CsvHelper.Configuration;
using System.IO.Ports;


// 3rd Party
using Zeroconf;

// NDI
using NDI.CapiSample;
using NDI.CapiSample.Data;
using NDI.CapiSample.Protocol;
using System.Runtime.CompilerServices;
using System.Drawing;
using Microsoft.VisualBasic.FileIO;

namespace NDI.CapiSampleApplication
{
    /// <summary>
    /// This sample program showcases a common connection process to receive pose data for Aurora and Vega products.
    /// </summary>
    class Program
    {
        static void Main(string[] args)
        {
            log("C# CAPI Sample v" + Capi.GetVersion());

            // Use the host specified if provided, otherwise:
            // 1st. try to find a serial device.
            // 2nd. Try to find a network device.
            Capi cAPI, cAPI_test;
            string host = "";
            if (args.Length > 0)
            {
                host = args[0];
            }
            else
            {
                //log("Finding serial NDI devices.");
                //string[] comPorts = CapiSerial.GetAvailableComPorts();
                //if (comPorts.Length > 0)
                //{
                //    host = comPorts[0];
                //}
                //else
                //{
                //    log("Could not find a serial device.");
                //    log("Finding an NDI device on the network.");
                //    var task = GetIP();
                //    task.Wait(5000);

                //    if (task.IsCompleted && task.Result != null)
                //    {
                //        host = task.Result;
                //    }
                //    else
                //    {
                //        log("Could not find an NDI device on the network.");
                //    }
                //}

                log("Finding an NDI device on the network.");
                var task = GetIP();
                task.Wait(5000);

                if (task.IsCompleted && task.Result != null)
                {
                    host = task.Result;
                }
                else
                {
                    log("Could not find an NDI device on the network.");
                }
            }

            if (host == "" || host == "help" || host == "-h" || host == "/?" || host == "--help")
            {
                // host argument was not provided and we couldn't find one
                if (host == "")
                {
                    log("Could not automatically detect an NDI device, please manually specify one.");
                }

                PrintHelp();
                return;
            }

            //// Create a new CAPI instance based on the connection type
            //if(host.StartsWith("COM") || host.StartsWith("/dev"))
            //{
            //    cAPI = new CapiSerial(host);
            //}
            //else
            //{
            //    cAPI = new CapiTcp(host);
            //}

            int flag1 = 0;
            int ctr = 0;
            string[] fields = { "k" };

            DateTime now = DateTime.Now;
            string filedir = @"D:\FichStuff\CTR_Fall23\";
            string date = now.ToString("yyyy-M-d");
            string time = now.ToString("hh-mm-ss");
            string filePath = filedir + date + "_" + time + ".csv";

            //string fileRead = filedir + "21_09_23-16_09_2-tubes_rotate.csv";
            //string fileRead = filedir + "06_10_23-17_44_3-tubes_in-plane-bending.csv";
            //string fileRead = filedir + "07_10_23-14_56_2-tubes_rotate.csv";
<<<<<<< HEAD
<<<<<<< HEAD
            string fileRead = filedir + "21_11_23-13_39_2-tubes_rotate.csv";
=======
            string fileRead = filedir + "20_11_23-15_31_2-tubes_rotate.csv";
>>>>>>> 1b65e17 (added fucntion to calculate the  complete jacobian function)
=======
            string fileRead = filedir + "21_11_23-13_39_2-tubes_rotate.csv";
>>>>>>> 4f3ebc2 (changes to fkin_comp.m)

            int test_flag = 0;

            while (test_flag == 0)
            {
                log("Testing tracking setup. Press y to begin and n to skip");
                string inp3 = Console.ReadLine();
                if (inp3 == "y" || inp3 == "Y")
                {
                    cAPI_test = new CapiTcp(host);
                    test_flag = Test_tracking(cAPI_test);
                    cAPI_test = null;
                }
                else
                {
                    test_flag = 1;
                }
            }



            string[] dataRead = File.ReadAllLines(fileRead);
            int dataSize = dataRead.Length;
            //log(dataSize.ToString());
            //log(dataRead[1]);

            int end_flag = 0;
            int write_flag = 0;

            SerialPort port = new SerialPort("COM4", 250000);
            TextFieldParser parser = new TextFieldParser(fileRead);

            parser.TextFieldType = FieldType.Delimited;
            parser.SetDelimiters(",");

            while (flag1 == 0 && ctr < dataSize)
            {
                // Create a new CAPI instance based on the connection type
                //if (host.StartsWith("COM") || host.StartsWith("/dev"))
                //{
                //    cAPI = new CapiSerial(host);
                //    log("Serial host connected...");
                //}
                //else
                //{
                //    cAPI = new CapiTcp(host);
                //    log("TCP host connected...");
                //}
                
                if (end_flag == 0 && write_flag == 0)
                {
                    //Processing row
                    fields = parser.ReadFields();
                    if(parser.EndOfData)
                    {
                        end_flag = 1;
                    }
                }


                port.Open();
                log("Writing to robot...");

                //port.WriteLine("G0 X0 Y0 Z0 A0 B0 C0\n");
                //port.WriteLine("G0 X0 Y0 Z0 A10 B10 C10\n");
                port.WriteLine((string)fields[0]);
                log(fields[0]);
                log("Done writing....");
                log("Collecting data point: " + ctr);


                log("Enter t to start tracking or anything else to quit");
                string inp0 = Console.ReadLine();
                if (inp0 == "t" || inp0 == "T")
                {

                    cAPI = new CapiTcp(host);
                    log("TCP host connected...");

                    //log("Collecting data point: " + ctr);
                    
                    write_flag = Run(cAPI, filePath, fields);
                    cAPI = null;
                    port.Close();
                    if (write_flag == 0)
                    {
                        ctr++;
                    }
                }
                else
                {
                    flag1 = 1;
                    port.Close();
                    break;
                }

                log("Enter s to stop or anything else collect next data point");
                string inp1 = Console.ReadLine();

                if (inp1 == "s" || inp1 == "S")
                {
                    flag1 = 1;
                    port.Close();
                }
            }

        }

        /// <summary>
        /// Print usage information
        /// </summary>
        public static void PrintHelp()
        {
            log("");
            log("usage: CAPISampleApplication.exe [<hostname>]");
            log("where:");
            log("\t<hostname>\t(optional) The measurement device's hostname, IP address, or serial port.");
            log("example hostnames:");
            log("\tConnecting to device by IP address: 169.254.8.50");
            log("\tConnecting to device by hostname: P9-B0103.local");
            log("\tConnecting to serial port varies by operating system:");
            log("\t\tCOM10 (Windows), /dev/ttyUSB0 (Linux), /dev/cu.usbserial-001014FA (Mac)");
        }

        /// <summary>
        /// Timestamped log output function.
        /// </summary>
        /// <param name="message">Message to be logged.</param>
        public static void log(string message)
        {
            string time = DateTime.UtcNow.ToString("yyyy-MM-ddTHH:mm:ss.fffZ");
            Console.WriteLine(time + " [CAPISample] " + message);
        }

        public static string inp()
        {
            string mes = Console.ReadLine();
            return mes;
        }

        /// <summary>
        /// Find the first device on the network that is broadcasting the _ndi._tcp.local. service.
        /// </summary>
        /// <returns>Task with an IP string result.</returns>
        private static async Task<string> GetIP()
        {
            IReadOnlyList<IZeroconfHost> results = await ZeroconfResolver.ResolveAsync("_ndi._tcp.local.");
            foreach(var result in results)
            {
                return result.IPAddress;
            }

            return null;
        }

        private static bool InitializePorts(Capi cAPI)
        {
            // Polaris Section
            // ---
            // Request a new tool port handle so that we can load an SROM
            Port tool1 = cAPI.PortHandleRequest();
            if (tool1 == null)
            {
                log("Could not get available port for tool.");
            }
            else if (!tool1.LoadSROM("sroms/needle_pointer.rom"))
            //8700339.rom defulat
            {
                log("Could not load SROM file for tool1.");
                return false;
            }

            Port tool2 = cAPI.PortHandleRequest();
            if (tool2 == null)
            {
                log("Could not get available port for tool.");
            }
            else if (!tool2.LoadSROM("sroms/home_frame2.rom"))
            //8700339.rom defulat
            {
                log("Could not load SROM file for tool2.");
                return false;
            }
            // ---

            // Initialize all ports not currently initialized
            var ports = cAPI.PortHandleSearchRequest(PortHandleSearchType.NotInit);
            foreach (var port in ports)
            {
                if (!port.Initialize())
                {
                    log("Could not initialize port " + port.PortHandle + ".");
                    return false;
                }

                if (!port.Enable())
                {
                    log("Could not enable port " + port.PortHandle + ".");
                    return false;
                }
            }

            // List all enabled ports
            log("Enabled Ports:");
            ports = cAPI.PortHandleSearchRequest(PortHandleSearchType.Enabled);
            foreach (var port in ports)
            {
                port.GetInfo();
                log(port.ToString());
            }

            return true;
        }

        /// <summary>
        /// Check for BX2 command support.
        /// </summary>
        /// <param name="apiRevision">API revision string returned by CAPI.GetAPIRevision()</param>
        /// <returns>True if supported.</returns>
        private static bool IsBX2Supported(string apiRevision)
        {
            // Refer to the API guide for how to interpret the APIREV response
            char deviceFamily = apiRevision[0];
            int majorVersion = int.Parse(apiRevision.Substring(2, 3));

            // As of early 2017, the only NDI device supporting BX2 is the Vega
            // Vega is a Polaris device with API major version 003
            if (deviceFamily == 'G' && majorVersion >= 3)
            {
                return true;
            }

            return false;
        }

        /// <summary>
        /// Run the CAPI sample regardless of the connection method.
        /// </summary>
        /// <param name="cAPI">The configured CAPI protocol.</param>
        private static int Run(Capi cAPI, string filePath, string[] field)
        {
            // Be Verbose
            // cAPI.LogTransit = true;

            // Use the same log output format as this sample application
            var csv = new StringBuilder();
            int write_flag = 0;

            cAPI.SetLogger(log);

            if (!cAPI.Connect())
            {
                log("Could not connect to " + cAPI.GetConnectionInfo());
                PrintHelp();
                return 0;
            }
            log("Connected");

            // Get the API Revision this will tell us if BX2 is supported.
            string revision = cAPI.GetAPIRevision();
            log("Revision:" + revision);

            if (!cAPI.Initialize())
            {
                log("Could not initialize.");
                return 0;
            }
            log("Initialized");

            // The Frame Frequency may not be possible to set on all devices, so an error response is okay.
            cAPI.SetUserParameter("Param.Tracking.Frame Frequency", "60");
            cAPI.SetUserParameter("Param.Tracking.Track Frequency", "2");

            // Read the final values
            log(cAPI.GetUserParameter("Param.Tracking.Frame Frequency"));
            log(cAPI.GetUserParameter("Param.Tracking.Track Frequency"));

            // Initialize tool ports
            if (!InitializePorts(cAPI))
            {
                return 0;
            }

            if (!cAPI.TrackingStart())
            {
                log("Could not start tracking.");
                return 0;
            }
            log("TrackingStarted");

            // Track several frames of data using the BX command.
            //for (int i = 0; i < 300; i++)
            //{
            //    if (!cAPI.IsConnected)
            //    {
            //        log("Disconnected while tracking.");
            //        break;
            //    }
            //
            //    List<Tool> tools = cAPI.SendBX();
            //    foreach (var t in tools)
            //    {
            //        log(t.ToString());
            //    }
            //}

            // Track several frames of data using the BX2 Command
            if (IsBX2Supported(revision))
            {
                for (int i = 0; i < 20; i++)
                {
                    if (!cAPI.IsConnected)
                    {
                        log("Disconnected while tracking.");
                        break;
                    }

                    foreach (var f in field)
                    {
                        csv.Append(f);
                        csv.Append(",");
                    }
                    
                    List<Tool> tools = cAPI.SendBX2("--6d=tools");
                    foreach (var t in tools)
                    {
                        //log(t.ToString());
                        log((t.transform.position).ToString());

                        var dataStr = t.ToString();

                        //csv.Append(dataStr);
                        
                        csv.Append(t.transform.status);
                        csv.Append(",");
                        csv.Append(t.transform.position);
                        csv.Append(",");
                        csv.Append(t.transform.orientation);
                        csv.Append(",");
                        csv.Append(t.transform.error);
                        csv.Append(",");
                        //foreach (var m in t.markers)
                        //{
                        //    log(m.ToString());
                        //}
                    }
                    csv.AppendLine();
                }        
            }

            if (!cAPI.TrackingStop())
            {
                log("Could not stop tracking.");
                return 0;
            }
            log("TrackingStopped");

            if (!cAPI.Disconnect())
            {
                log("Could not disconnect.");
                return 0;
            }
            log("Disconnected");

            log("Is the collected data good? (y/n)");
            string inp2 = Console.ReadLine();
            if (inp2 == "y" || inp2 == "Y")
            {
                log("wrting to csv....");
                File.AppendAllText(filePath, csv.ToString());
                write_flag = 0;
            }
            else
            {
                log("Okay let's try again...");
                write_flag = 1;
            }

            return write_flag;
        }

        private static int Test_tracking(Capi cAPI)
        {
            // Be Verbose
            // cAPI.LogTransit = true;

            // Use the same log output format as this sample application

            int test_flag = 0;
            cAPI.SetLogger(log);

            if (!cAPI.Connect())
            {
                log("Could not connect to " + cAPI.GetConnectionInfo());
                PrintHelp();
                return 0;
            }
            log("Connected");

            // Get the API Revision this will tell us if BX2 is supported.
            string revision = cAPI.GetAPIRevision();
            log("Revision:" + revision);

            if (!cAPI.Initialize())
            {
                log("Could not initialize.");
                return 0;
            }
            log("Initialized");

            // The Frame Frequency may not be possible to set on all devices, so an error response is okay.
            cAPI.SetUserParameter("Param.Tracking.Frame Frequency", "60");
            cAPI.SetUserParameter("Param.Tracking.Track Frequency", "2");

            // Read the final values
            log(cAPI.GetUserParameter("Param.Tracking.Frame Frequency"));
            log(cAPI.GetUserParameter("Param.Tracking.Track Frequency"));

            // Initialize tool ports
            if (!InitializePorts(cAPI))
            {
                return 0;
            }

            if (!cAPI.TrackingStart())
            {
                log("Could not start tracking.");
                return 0;
            }
            log("TrackingStarted");

            // Track several frames of data using the BX command.
            //for (int i = 0; i < 300; i++)
            //{
            //    if (!cAPI.IsConnected)
            //    {
            //        log("Disconnected while tracking.");
            //        break;
            //    }
            //
            //    List<Tool> tools = cAPI.SendBX();
            //    foreach (var t in tools)
            //    {
            //        log(t.ToString());
            //    }
            //}

            // Track several frames of data using the BX2 Command
            if (IsBX2Supported(revision))
            {
                for (int i = 0; i < 20; i++)
                {
                    if (!cAPI.IsConnected)
                    {
                        log("Disconnected while tracking.");
                        break;
                    }

                    List<Tool> tools = cAPI.SendBX2("--6d=tools");
                    foreach (var t in tools)
                    {
                        //log(t.ToString());
                        log((t.transform).ToString());
                        
                        //foreach (var m in t.markers)
                        //{
                        //    log(m.ToString());
                        //}
                    }
                    
                }
            }

            if (!cAPI.TrackingStop())
            {
                log("Could not stop tracking.");
                return 0;
            }
            log("TrackingStopped");

            if (!cAPI.Disconnect())
            {
                log("Could not disconnect.");
                return 0;
            }
            log("Disconnected");

            log("Is the collected data good? (y/n)");
            string inp2 = Console.ReadLine();
            if (inp2 == "y" || inp2 == "Y")
            {
                
                test_flag = 1;
            }
            else
            {
                log("Okay let's try again...");
                test_flag = 0;
            }

            return test_flag;
        }
    }
}
