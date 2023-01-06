
using Plugin.BluetoothClassic.Abstractions;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Runtime.CompilerServices;
using System.Text;
using Xamarin.Forms;

namespace BluetoothApp.ViewModels
{
    public class SelectDeviceViewModel : INotifyPropertyChanged
    {
        private ObservableCollection<BluetoothDeviceModel> _deviceList;
        private BluetoothDeviceModel _selectedDevice;
        private double _realDisplacement;
        private bool _isConnected;
        private char _command;
        private string[] commandList = { "t1", "t2", "t3","t4", "t5", "t6", "t7", "t8", "t9", "t10"};
        private int _selectedCommand = 0;



       

        public string[] CommandList
        {
            get { return commandList; }
            set 
            {
                commandList = value;
                OnPropertyChanged(nameof(CommandList));
            }
        }


        public int SelectedCommand
        {
            get { return _selectedCommand; }
            set 
            { 
                _selectedCommand = value;
                OnPropertyChanged(nameof(SelectedCommand));
            }
        }


        public SelectDeviceViewModel()
        {
            BluetoothAdapter = DependencyService.Resolve<IBluetoothAdapter>();

            FindDevices();
        }

        public void FindDevices()
        {
            DeviceList = new ObservableCollection<BluetoothDeviceModel>();
            if (BluetoothAdapter != null)
            {
                foreach (var item in BluetoothAdapter.BondedDevices)
                {
                    DeviceList.Add(item);
                }
            }
        }

        public ObservableCollection<BluetoothDeviceModel> DeviceList
        {
            get
            {
                return _deviceList;
            }
            set
            {
                _deviceList = value;
                OnPropertyChanged(nameof(DeviceList));
            }
        }
        public BluetoothDeviceModel SelectedDevice
        {
            get
            {
                return _selectedDevice;
            }
            set
            {
                _selectedDevice = value;
                OnPropertyChanged(nameof(SelectedDevice));
            }
        }
        public double RealDisplacement
        {
            get
            {
                return _realDisplacement;
            }
            set
            {
                _realDisplacement = value;
                OnPropertyChanged(nameof(RealDisplacement));
            }
        }

        public bool IsConnected
        {
            get { return _isConnected; } 
            set
            {
                _isConnected= value;
                OnPropertyChanged(nameof(IsConnected));
            } 
        }

        public char Command
        {
            get { return _command; }
            set 
            {
                _command = value;
                OnPropertyChanged(nameof(Command));
            }
        }

        private double _avgVel;

        public double AvgVel
        {
            get { return _avgVel; }
            set 
            {
                _avgVel = value;
                OnPropertyChanged(nameof(AvgVel));
            }
        }

        private double _dyMPU;

        public double DyMPU
        {
            get { return _dyMPU; }
            set 
            {
                _dyMPU = value;
                OnPropertyChanged(nameof(DyMPU));
            }
        }
        private double _dxMPU;

        public double DxMPU
        {
            get { return _dxMPU; }
            set
            {
                _dxMPU = value;
                OnPropertyChanged(nameof(DxMPU));
            }
        }
        private double _calculateDisplacement;

        public double CalculateDisplacement 
        {
            get { return _calculateDisplacement; }
            set
            {
                _calculateDisplacement = value;
                OnPropertyChanged(nameof(CalculateDisplacement));
            }
        }
        private int _avgPWM;

        public int AvgPWM
        {
            get { return _avgPWM; }
            set 
            {
                _avgPWM = value;
                OnPropertyChanged(nameof(AvgPWM));
            }
        }


        public readonly IBluetoothAdapter BluetoothAdapter;

        public event PropertyChangedEventHandler PropertyChanged;
        private void OnPropertyChanged([CallerMemberName] string name = "")
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
        }
    }
}
