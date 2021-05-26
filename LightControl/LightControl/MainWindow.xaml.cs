using System;
using System.Collections.Generic;
using System.Linq;
using System.IO.Ports;
using System.Text;
using System.Threading;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Diagnostics;

namespace LightControl
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        static SerialPort _serialPort;

        public MainWindow()
        {
            InitializeComponent();
        }

        private void Connect()
        {
            _serialPort = new SerialPort(comPort.Text, 115200)
            {
                ReadTimeout = 2000,
                WriteTimeout = 2000
            };
            _serialPort.Open();
        }

        private void Disconnect()
        {
            _serialPort.Close();
            _serialPort = null;
        }

        enum Mode { Random, Pattern, Individual, Switching };

        private void SetMode(Mode mode)
        {
            var cmd = "";
            switch (mode)
            {
                case Mode.Random: cmd = "r"; break;
                case Mode.Pattern: cmd = "s";  break;
                case Mode.Individual: cmd = "i";  break;
                case Mode.Switching: cmd = "f"; /* anything not the above or p or w */ break;
            }

            _serialPort.Write(cmd);
            string back = char.ToString((char)_serialPort.ReadChar());
            Debug.Assert(cmd == back);
        }

        // how to serialize these into a thing?
        private void SetLedValues(int[] values)
        {
            // serialize the LEDs into a bytestream we can send to the FPGA
            // 24 led values, 3 bits each
            // so each 8 leds makes 3 bytes. do that 3 times to create byte array to send to com port
            byte[] led_register = new byte[9];
            int byte_idx = 0;
            int packed;
            string back;
            for (int i=0;i<3;i++)
            {
                packed = 0;

                for (int j=0;j<8;j++)
                {
                    int led = 23 - (i * 8 + j);
                    packed = (packed << 3) | values[led];
                }

                // great now we have 3 bytes worth of led values. unpack them and put them into bytes
                led_register[byte_idx++] = (byte)((packed & 0xFF0000) >> 16);
                led_register[byte_idx++] = (byte)((packed & 0xFF00) >> 8);
                led_register[byte_idx++] = (byte)(packed & 0xFF);
            }
            for (int i=0;i<9;i++)
            {
                _serialPort.Write("p");
                back = char.ToString((char)_serialPort.ReadChar());
                Debug.Assert(back == "p");

                _serialPort.Write(led_register, i, 1);
                byte backval = (byte)_serialPort.ReadByte();
                Debug.Assert(backval == led_register[i]);
            }

            _serialPort.Write("u");
            back = char.ToString((char)_serialPort.ReadChar());
            Debug.Assert(back == "u");
        }

        private void Connect_Click(object sender, RoutedEventArgs e)
        {
            if ((string)connect.Content == "Connect")
            {
                // On connect:
                // Open that com port
                // Put in individual mode
                // Clear out all values
                Connect();
                var back = _serialPort.ReadByte(); // opening will spit out a ?
                Debug.Print("got " + back + "back");

                _serialPort.Write("z");
                back = _serialPort.ReadByte();
                Debug.Print("got " + back + "back");

                Thread.Sleep(1000);

                SetMode(Mode.Individual);
                connect.Content = "Disconnect";
            }
            else
            {
                SetMode(Mode.Switching);
                Disconnect();
                connect.Content = "Connect";
            }
        }

        private int[] GetLedValue(Ellipse e)
        {
            int[] values = new int[2];

            if (e.Fill == Brushes.White)
            {
                values[0] = 0;
                values[1] = 0;
            }
            else if (e.Fill == Brushes.Red)
            {
                values[0] = 7;
                values[1] = 0;
            }
            else if (e.Fill == Brushes.Green) {
                values[0] = 0;
                values[1] = 7;
            }
            else {
                Debug.Assert(false);
            }

            return values;
        }

        private void Led_MouseLeftButtonDown(object sender, MouseButtonEventArgs unused)
        {

            Ellipse e = (Ellipse)sender;
            if (e.Fill == Brushes.White)
            {
                e.Fill = Brushes.Red;
            }
            else if (e.Fill == Brushes.Red)
            {
                e.Fill = Brushes.Green;
            }
            else if (e.Fill == Brushes.Green)
            {
                e.Fill = Brushes.White;
            }
            else
            {
                Debug.Assert(false);
            }

            List<int> values = new List<int>();
            values.AddRange(GetLedValue(led0));
            values.AddRange(GetLedValue(led1));
            values.AddRange(GetLedValue(led2));
            values.AddRange(GetLedValue(led3));
            values.AddRange(GetLedValue(led4));
            values.AddRange(GetLedValue(led5));
            values.AddRange(GetLedValue(led6));
            values.AddRange(GetLedValue(led7));
            values.AddRange(GetLedValue(led8));
            values.AddRange(GetLedValue(led9));
            values.AddRange(GetLedValue(led10));
            values.AddRange(GetLedValue(led11));

            SetLedValues(values.ToArray());
        }
    }
}
