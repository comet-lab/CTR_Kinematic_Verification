%% Initializing the reading OptiTrack data in Matlab
% Please turn on the system and set Rigid body first
% And enable "stream rigid body" 

%% Connection init
% Add NatNet .NET assembly so that Matlab can access its methods, delegates, etc.
% Note : The NatNetML.DLL assembly depends on NatNet.dll, so make sure they
% are both in the same folder and/or path if you move them.
% dllPath = fullfile('.\NatNet_SDK_4.0\NatNetSDK\lib\x64\NatNetML.dll');
dllPath = fullfile('D:\FichStuff\CTR_Spring23\CTR_ControlBoard_Firmware\NatNet_SDK_4.0_edited\NatNet_SDK_4.0\NatNetSDK\lib\x64\NatNetML.dll');
% dllPath = fullfile('C:\Users\ninab\OneDrive\Documents\WPI\2022-2023\Spring Semester\RBE 598 Directed Research\CTR_ControlBoard_Firmware\NatNet_SDK_4.0_edited\NatNet_SDK_4.0\NatNetSDK\lib\x64\NatNetML.dll');
assemblyInfo = NET.addAssembly(dllPath);
theClient = NatNetML.NatNetClientML(0); % Input = iConnectionType: 0 = Multicast, 1 = Unicast

% Connect to an OptiTrack server (Motive)
HostIP = char('130.215.211.19');
theClient.Initialize(HostIP, HostIP);

disp('OptiTrack NatNet connected!')