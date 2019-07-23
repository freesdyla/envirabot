using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Drawing;
using System.IO.Pipes;
using System.IO;
using Flir.Atlas.Live.Remote;
using Flir.Atlas.Live.Device;
using Flir.Atlas.Live.Discovery;
using Flir.Atlas.Image;
using Flir.Atlas.Image.Palettes;
using System.Threading;
using System.Diagnostics;

namespace FlirThermoCamServer
{
    class Program
    {
        static void Main(string[] args)
        {
            ThermoCam tc = new ThermoCam();

            tc.start();

            tc.snapShot(55);

            //for (int i = 0; i < 100; i++)
            //{
            //  Console.WriteLine("Enter focus distance\n");
            // string dist = Console.ReadLine();
            // int numVal = Int32.Parse(dist);
            //System.Threading.Thread.Sleep(1000);
            //tc.snapShot(i);
            // }

            NamedPipeServerStream namedPipeServer = new NamedPipeServerStream("thermo-pipe");

            Console.WriteLine("waiting for connection...");

            namedPipeServer.WaitForConnection();

            Console.WriteLine("Connected " + namedPipeServer.IsConnected);

            while(true)
            {
                try
                {
                    int byteFromClient = namedPipeServer.ReadByte();

                    if (byteFromClient > 0)
                    {
                        int result = tc.snapShot(byteFromClient);

                        // send confirmation to client when image saved
                        if(result == 0) namedPipeServer.WriteByte(1);
                        else namedPipeServer.WriteByte(2);
                    }
                    else
                        namedPipeServer.WriteByte(2);
                }
                catch(Exception)
                {
                    namedPipeServer.Disconnect();

                    Console.WriteLine("Disconnected. Waiting for new connection...");

                    namedPipeServer.WaitForConnection();

                    Console.WriteLine("Connected " + namedPipeServer.IsConnected);
                }
            }          
        }
    }

    class ThermoCam
    {
        public ThermalCamera _camera;
        public Discovery _discovery;

        public int resetThermalCamNetworkAdapter()
        {
            //8C-AE-4C-F4-43-83 mac address
            System.Diagnostics.ProcessStartInfo psi = new System.Diagnostics.ProcessStartInfo("netsh.exe");
            psi.WindowStyle = ProcessWindowStyle.Hidden;
            psi.UseShellExecute = true;
            psi.Verb = "runas";
            psi.Arguments = "interface set interface \"\"\"Ethernet 7\"\"\" disable";
            System.Diagnostics.Process p = new System.Diagnostics.Process();
            p.StartInfo = psi;
            p.Start();

            //System.Threading.Thread.Sleep(1000);

            psi.Arguments = "interface set interface \"\"\"Ethernet 7\"\"\" enable";
            System.Diagnostics.Process p1 = new System.Diagnostics.Process();
            p1.StartInfo = psi;
            p1.Start();

            string path = @"C:\Users\lietang123\Documents\RoAdFiles\LineProfilerRobotArmTest\LineProfilerRobotArmTest\log_file.txt";

            using (StreamWriter sw = File.AppendText(path))
            {
                string msg = DateTime.Now.ToString() + ": lost thermal cam connection";
                sw.WriteLine(msg);
            }

            //System.Threading.Thread.Sleep(10000);
            return 0;
        }



        public int start()
        {
            _discovery = new Discovery();
            _camera = new ThermalCamera();

            _discovery.DeviceFound += _discovery_DeviceFound;

            _discovery.Start(Interface.Gigabit);

            int timeout_cnt = 0;

            while (!_camera.ConnectionStatus.Equals(ConnectionStatus.Connected))
            {
                System.Threading.Thread.Sleep(1000);

                if(++timeout_cnt > 60)
                {
                    Console.WriteLine("camera connection timeout");
                    return -1;
                }
            };

            Console.WriteLine("camera connected");

           // _camera.RemoteControl.Focus.SetDistance(0.3);

            //snapShot();

            return 0;
        }

        public void stop()
        {
            _camera.Disconnect();
        }

        public void _discovery_DeviceFound(Object sender, CameraDeviceInfoEventArgs e)
        {
            Console.WriteLine("found device");
            Console.WriteLine(e.CameraDevice.Name);
            _discovery.Stop();
            _camera.Connect(e.CameraDevice);
        }

        public int snapShot(int focus_dist_cm)
        {

            double focus_dist;
            bool set_focus_success = false;

            for (int i = 0; i < 5 && !set_focus_success; i++)
            {
                try
                {
                    //_camera.RemoteControl.Focus.Mode(FocusMode.Auto);
                    focus_dist = Math.Max(Math.Min((double)focus_dist_cm / 100.0, 2.0), 0.1); _camera.RemoteControl.Focus.SetDistance(focus_dist);
                    set_focus_success = true;
                    System.Threading.Thread.Sleep(500);    //important for the motor to finish
                }
                catch (Exception exception)
                {
                    Console.WriteLine(exception.ToString());
                    stop();
                    resetThermalCamNetworkAdapter();
                    start();
                    //if (start() != 0) return -1;
                }
            }

            Console.WriteLine(focus_dist_cm + " cm");

            _camera.GetImage().EnterLock();

            try
            {
                String path = "c:/users/lietang123/documents/roadfiles/flirthermocamserver/flirthermocamserver/bin/release/thermo.jpg";

                ThermalImage img = (ThermalImage)_camera.GetImage();
                img.Palette = PaletteManager.Iron;
                img.Scale.IsAutoAdjustEnabled = true;
                img.SaveSnapshot(path);
                Rectangle rectangle = new Rectangle(0, 0, 640, 480);
                double[] temperature_img = img.GetValues(rectangle);

                using (BinaryWriter writer = new BinaryWriter(File.Open("c:/users/lietang123/documents/roadfiles/flirthermocamserver/flirthermocamserver/bin/release/temperature.bin", FileMode.Create)))
                {
                    var result = new byte[temperature_img.Length * sizeof(double)];
                    Buffer.BlockCopy(temperature_img, 0, result, 0, result.Length);
                    writer.Write(result);
                }
            }
            catch (Exception exception)
            {
                Console.WriteLine(exception.ToString());
                stop();
                resetThermalCamNetworkAdapter();
                start();
                _camera.GetImage().ExitLock();
                return -1;
            }
            finally
            {
                
            }

            _camera.GetImage().ExitLock();

            return 0;
        }

    }
}
