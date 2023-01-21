using BluetoothApp.Services.Protocol;
using BluetoothApp.ViewModels;
using Plugin.BluetoothClassic.Abstractions;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Xamarin.Essentials;
using Xamarin.Forms;
using Xamarin.Forms.Xaml;
using static Xamarin.Forms.Internals.Profile;

namespace BluetoothApp.Views
{
    [XamlCompilation(XamlCompilationOptions.Compile)]
    public partial class SelectDeviceView : ContentPage
    {
        private ProtocolDecode _protocolDecode;
        private bool _isConnected;
        public Action change;
        private IBluetoothManagedConnection _currentConnection;
        private List<int> PWM;
        private int avgPWM;
        private int _setAngle;
        private int oldSetAngle;
        public SelectDeviceView()
        {
            InitializeComponent();

            BindingContext = new SelectDeviceViewModel();
            PWM = new List<int>();
            UpdateUI();

            _protocolDecode = new ProtocolDecode(0xAB, 0xCD, 0xAF, 0xCF);
            _protocolDecode.OnDataFormatedEvent += OnDataFormatted;
        }

        private void UpdateUI()
        {
            var device = BindingContext as SelectDeviceViewModel;
            var adapter = device.BluetoothAdapter;
            if (adapter != null)
            {
                if (!adapter.Enabled)
                {
                    adapter.Enable();
                    device.FindDevices();
                }
            }
        }
        private async void btnConnect_Clicked(object sender, EventArgs e)
        {
            var device = BindingContext as SelectDeviceViewModel;

            var selectedDevice = device.SelectedDevice;
            if (selectedDevice != null)
            {
                if (!_isConnected)
                {
                    var connected = await TryConnect(selectedDevice, device.BluetoothAdapter);
                    if (connected)
                    {
                        btnConnect.Text = "Desconectar";
                        _isConnected = true;
                        lvBondedDevices.IsEnabled = false;
                        //await Navigation.PushAsync(new DataMonitorView());
                        device.IsConnected = true;
                        SendCommand(true);


                    }
                }
                else
                {
                    try
                    {
                        _currentConnection.Dispose();
                        btnConnect.Text = "Conectar";
                        _isConnected = false;
                        lvBondedDevices.IsEnabled = true;
                        device.RealDisplacement = 0;
                        device.CalculateDisplacement = 0;
                        device.DyMPU = 0;
                        device.DxMPU = 0;
                        device.Command = 'f';
                        device.IsConnected = false;
                        SendCommand(true);
                    }
                    catch (Exception ex)
                    {

                        await DisplayAlert("Error", ex.Message, "Close");
                    }
                }
            }
        }

        private async Task<bool> TryConnect(BluetoothDeviceModel device, IBluetoothAdapter adapter)
        {
            _currentConnection = adapter.CreateManagedConnection(device);
            try
            {
                _currentConnection.Connect();
                _currentConnection.OnRecived += Connection_OnRecived;
                return true;
            }
            catch (Exception exception)
            {
                await DisplayAlert("Generic error", exception.Message, "Close");
                return false;
            }
        }

        private void Connection_OnRecived(object sender, RecivedEventArgs recivedEventArgs)
        {
            var data = recivedEventArgs.Buffer.ToArray();
            _protocolDecode.Add(data);
        }

        private void OnDataFormatted(IEnumerable<byte> data)
        {
            var data1 = new byte[4] { 0, 0, 0, 0 };
            data1[0] = data.ElementAt(0);
            data1[1] = data.ElementAt(1);
            var data2 = new byte[4] { 0, 0, 0, 0 };
            data2[0] = data.ElementAt(2);
            data2[1] = data.ElementAt(3);
            var data3 = new byte[4] { 0, 0, 0, 0 };
            data3[0] = data.ElementAt(4);
            data3[1] = data.ElementAt(5);
            var signalDy = data.ElementAt(6) == 0 ? 1 : -1;
            var data4 = new byte[4] { 0, 0, 0, 0 };
            data4[0] = data.ElementAt(7);
            data4[1] = data.ElementAt(8);
            var signalDx = data.ElementAt(9) == 0 ? 1 : -1;
            PWM.Add(data.ElementAt(10));
            var data5 = new byte[4] { 0, 0, 0, 0 };
            data5[0] = data.ElementAt(11);
            data5[1] = data.ElementAt(12);
            data5[2] = data.ElementAt(13);
            data5[3] = data.ElementAt(14);

            if (PWM.Count() > 0)
            {
                if ((int)PWM.Average() > avgPWM)
                {
                    avgPWM = (int)PWM.Average();
                }

            }

            var model = BindingContext as SelectDeviceViewModel;
            var avgValue = BitConverter.ToInt32(data1, 0);
            var calculateDisplacement = BitConverter.ToInt32(data2, 0);
            var dyMPU = BitConverter.ToInt32(data3, 0);
            var dxMPU = BitConverter.ToInt32(data4, 0);
            var convert = BitConverter.ToInt32(data.ToArray(), 15);
            var convert2 = BitConverter.ToInt32(data5.ToArray(), 0);
            model.AvgVel = (double)(avgValue) / 1000;
            model.CalculateDisplacement = (double)(calculateDisplacement) / 1000;
            model.DyMPU = (double)(dyMPU) / 1000 * signalDy;
            model.DxMPU = (double)(dxMPU) / 1000 * signalDx;
            model.AvgPWM = avgPWM;
            //var aux = ((double)(convert)) * 0.033 * 2 * Math.PI / 520;
            model.Encoder = convert;
            model.Encoder2 = convert2;
            model.EncoderError = convert - convert2;
            model.RealDisplacement = ((double)(convert)) * 0.033 * 2 * Math.PI / 520;
        }

       

        private int convertAngle(int angle)
        {
            int _angle = 0;
            if (angle <= 80 && angle >= -80)
            {

                if (oldSetAngle >= 90)
                {
                    if (angle >= 0)
                    {
                        _angle = angle + 90 +3;
                    }
                    else
                    {
                        _angle = angle +90 + 3;
                    }

                }
                else
                {
                    if (angle >= 0)
                    {
                        _angle = angle + 90 + 5;
                    }
                    else
                    {
                        _angle = angle + 90 +5;
                    }
                }
                oldSetAngle = _angle;


            }
            return _angle;

        }
        private void btnSend_Clicked(object sender, EventArgs e)
        {
            SendCommand(false);
        }
        public void SendCommand(bool isStop)
        {
            double _setTime = 0.0;
            double.TryParse(setTime.Text, out _setTime);
            int.TryParse(setAngle.Text, out _setAngle);

            byte[] message = BitConverter.GetBytes(((int)(_setTime * 1000)));
            byte[] message2 = BitConverter.GetBytes(convertAngle(_setAngle));
            var model = BindingContext as SelectDeviceViewModel;
            byte[] data = new byte[14];

            char command1 = isStop == true ? 's' : model.CommandList[model.SelectedCommand][0];
            char command2 = isStop == true ? 't' : model.CommandList[model.SelectedCommand][1]; ;
            data[0] = 0xAB;
            data[1] = 0xCD;
            data[2] = (byte)command1;
            data[3] = (byte)command2;
            data[4] = message[3];
            data[5] = message[2];
            data[6] = message[1];
            data[7] = message[0];
            data[8] = message2[3];
            data[9] = message2[2];
            data[10] = message2[1];
            data[11] = message2[0];
            data[12] = 0xAF;
            data[13] = 0xCF;
            PWM.Clear();
            avgPWM = 0;

            _currentConnection.Transmit(data);
        }
        private void btnStop_Clicked(object sender, EventArgs e)
        {
            SendCommand(true);
        }
    }
}