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

namespace FlirThermoCamServer
{
    class Program
    {
        static void Main(string[] args)
        {
            ThermoCam tc = new ThermoCam();
            tc.init();

            //for (int i = 0; i < 100; i++)
            {
                
              //  Console.WriteLine("Enter focus distance\n");
               // string dist = Console.ReadLine();
               // int numVal = Int32.Parse(dist);
                tc.snapShot(50);
            }

            NamedPipeServerStream namedPipeServer = new NamedPipeServerStream("thermo-pipe");

            Console.WriteLine("waiting for connection...");

            namedPipeServer.WaitForConnection();

            Console.WriteLine("Connected " + namedPipeServer.IsConnected);

            while(true)
            {
                try
                {
                    int byteFromClient = namedPipeServer.ReadByte();

                   // if (byteFromClient != -1)
                     //   Console.WriteLine(byteFromClient);

                    if (byteFromClient > 0) // fire camera
                    {
                        tc.snapShot(byteFromClient);
                       // Console.WriteLine("snap shot");

                        // send confirmation to client when image saved
                        namedPipeServer.WriteByte(1);
                    }

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

        public int init()
        {
            _discovery = new Discovery();
            _camera = new ThermalCamera();

            _discovery.DeviceFound += _discovery_DeviceFound;

            _discovery.Start(Interface.Gigabit);

            while (!_camera.ConnectionStatus.Equals(ConnectionStatus.Connected)) ;

            Console.WriteLine("camera connected");

           // _camera.RemoteControl.Focus.SetDistance(0.3);

            //snapShot();

            return 0;
        }

        public void _discovery_DeviceFound(Object sender, CameraDeviceInfoEventArgs e)
        {
            Console.WriteLine("found device");
            Console.WriteLine(e.CameraDevice.Name);
            _discovery.Stop();
            _camera.Connect(e.CameraDevice);
        }

        public void snapShot(int focus_dist_cm)
        {
            try
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
                    }
                }

                Console.WriteLine(focus_dist_cm + " cm");

                _camera.GetImage().EnterLock();

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
            }
            finally
            {
                _camera.GetImage().ExitLock();
            }
        }

    }
}
