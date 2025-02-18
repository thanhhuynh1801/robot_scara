
function varargout = robot_gui(varargin)
% ROBOT_GUI MATLAB code for robot_gui.fig
%      ROBOT_GUI, by itself, creates a new ROBOT_GUI or raises the existing
%      singleton*.
%
%      H = ROBOT_GUI returns the handle to a new ROBOT_GUI or the handle to
%      the existing singleton*.
%
%      ROBOT_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ROBOT_GUI.M with the given input arguments.
%
%      ROBOT_GUI('Property','Value',...) creates a new ROBOT_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before robot_gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to robot_gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help robot_gui

% Last Modified by GUIDE v2.5 28-Nov-2023 15:43:14

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @robot_gui_OpeningFcn, ...
                   'gui_OutputFcn',  @robot_gui_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before robot_gui is made visible.
function robot_gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to robot_gui (see VARARGIN)

% Choose default command line output for robot_gui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes robot_gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = robot_gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on slider movement.
function slider_theta1_Callback(hObject, eventdata, handles)
% hObject    handle to slider_theta1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

theta1 = (pi/180)*(get(handles.slider_theta1,'Value'));
set(handles.edit_theta1,'String',theta1*180/pi);

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider_theta1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_theta1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider_theta2_Callback(hObject, eventdata, handles)
% hObject    handle to slider_theta2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

theta2 = (pi/180)*(get(handles.slider_theta2,'Value'));
set(handles.edit_theta2,'String',theta2*180/pi);

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider_theta2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_theta2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider_d3_Callback(hObject, eventdata, handles)
% hObject    handle to slider_d3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

d3 = get(handles.slider_d3,'Value');
set(handles.edit_d3,'String',d3);
    
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider_d3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_d3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider_theta4_Callback(hObject, eventdata, handles)
% hObject    handle to slider_theta4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
theta4 = (pi/180)*(get(handles.slider_theta4,'Value'));
set(handles.edit_theta4,'String',theta4*180/pi);
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider_theta4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_theta4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in forward.
function forward_Callback(hObject, eventdata, handles)
% hObject    handle to forward (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global theta1_pre;
global theta2_pre;
global d3_pre;
global theta4_pre; 
global opa_pre;

t1     = str2double(get(handles.edit_theta1,'String'));
t2     = str2double(get(handles.edit_theta2,'String'));
d3     = str2double(get(handles.edit_d3,'String'));
t4     = str2double(get(handles.edit_theta4,'String'));
set(handles.slider_theta1, 'value',t1);
set(handles.slider_theta2, 'value',t2);
set(handles.slider_theta4, 'value',t4);
set(handles.slider_d3, 'value',d3);

theta1 = (pi/180)*(get(handles.slider_theta1,'Value'));  
theta2 = (pi/180)*(get(handles.slider_theta2,'Value'));
theta4 = (pi/180)*(get(handles.slider_theta4,'Value'));    
d3 = get(handles.slider_d3,'Value');
    
[T10 T20 T30 T40] = forward(theta1, theta2, d3, theta4);
Px=T40(1,4);
Py=T40(2,4);
Pz=T40(3,4);
set(handles.edit_posx,'string',num2str(Px));
set(handles.edit_posy,'string',num2str(Py));
set(handles.edit_posz,'string',num2str(Pz));
%ROLL-PITCH-YAW
roll  = atan2(T40(2,3),T40(3,3));  %roll
pitch  = asin(-T40(3,1)); %pitch
yaw  = atan2(T40(2,1),T40(1,1));  %yaw
set(handles.edit_roll,'String',roll);
set(handles.edit_pitch,'String',pitch);
set(handles.edit_yaw,'String',yaw);

% alpha= atan2(-Z2, Z3)
% beta= asin(Z1)
% gamma = atan2(-Y1,X1)

opa=opa_pre;
%drawrobot(theta1,theta2,d3,theta4,opa);
the1= theta1_pre;
the2= theta1_pre;
the4= theta1_pre;
set(handles.forward, 'Enable', 'off');
numbers = [abs(theta1-the1)/0.05, abs(theta2-the2)/0.05, abs(theta4-the4)/0.05,15];
N = max(numbers); 

for k=1:N 
     Theta_1_temp = theta1_pre + (theta1-theta1_pre)*(k)/N;
     Theta_2_temp = theta2_pre + (theta2-theta2_pre)*(k)/N;
     d_3_temp = d3_pre+ (d3-d3_pre)*(k)/N;
     Theta_4_temp = theta4_pre+ (theta4-theta4_pre)*(k)/N;
     
     drawrobot(Theta_1_temp,Theta_2_temp,d_3_temp, Theta_4_temp,opa,handles);
     pause(0.2);
end
set(handles.forward, 'Enable', 'on');
theta1_pre = theta1;
theta2_pre = theta2;
d3_pre=d3;
theta4_pre = theta4; 


% --- Executes on button press in inverse.
function inverse_Callback(hObject, eventdata, handles)
% hObject    handle to inverse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global theta1_pre;
global theta2_pre;
global d3_pre;
global theta4_pre; 
global opa_pre;

x = str2double(handles.edit_posx.String);
y = str2double(handles.edit_posy.String);
z = str2double(handles.edit_posz.String);
yaw= str2double(handles.edit_yaw.String);

[theta1, theta2,d3,theta4]= Inverse(x,y,z,yaw);
set(handles.slider_theta1, 'value',(theta1*180/pi));
set(handles.slider_theta2, 'value',(theta2*180/pi));
set(handles.slider_theta4, 'value',(theta4*180/pi));
set(handles.slider_d3, 'value',d3);

set(handles.edit_theta1,'string',num2str(theta1*180/pi));
set(handles.edit_theta2,'string',num2str(theta2*180/pi));
set(handles.edit_theta4,'string',num2str(theta4*180/pi));
set(handles.edit_d3,'string',num2str(d3));

opa=opa_pre;
%drawrobot(theta1,theta2,d3,theta4,opa);
the1= theta1_pre;
the2= theta1_pre;
the4= theta1_pre;
set(handles.inverse, 'Enable', 'off');
numbers = [abs(theta1-the1)/0.05, abs(theta2-the2)/0.05, abs(theta4-the4)/0.05,10];
N = max(numbers); 

for k=1:N 
     Theta_1_temp = theta1_pre + (theta1-theta1_pre)*(k)/N;
     Theta_2_temp = theta2_pre + (theta2-theta2_pre)*(k)/N;
     d_3_temp = d3_pre+ (d3-d3_pre)*(k)/N;
     Theta_4_temp = theta4_pre+ (theta4-theta4_pre)*(k)/N;
     
     drawrobot(Theta_1_temp,Theta_2_temp,d_3_temp, Theta_4_temp,opa,handles);
     pause(0.2);
end
set(handles.inverse, 'Enable', 'on');
theta1_pre = theta1;
theta2_pre = theta2;
d3_pre=d3;
theta4_pre = theta4; 

function edit_posx_Callback(hObject, eventdata, handles)
% hObject    handle to edit_posx (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_posx as text
%        str2double(get(hObject,'String')) returns contents of edit_posx as a double


% --- Executes during object creation, after setting all properties.
function edit_posx_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_posx (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_posy_Callback(hObject, eventdata, handles)
% hObject    handle to edit_posy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_posy as text
%        str2double(get(hObject,'String')) returns contents of edit_posy as a double


% --- Executes during object creation, after setting all properties.
function edit_posy_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_posy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_posz_Callback(hObject, eventdata, handles)
% hObject    handle to edit_posz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_posz as text
%        str2double(get(hObject,'String')) returns contents of edit_posz as a double


% --- Executes during object creation, after setting all properties.
function edit_posz_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_posz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit9_Callback(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit9 as text
%        str2double(get(hObject,'String')) returns contents of edit9 as a double


% --- Executes during object creation, after setting all properties.
function edit9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit10_Callback(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit10 as text
%        str2double(get(hObject,'String')) returns contents of edit10 as a double


% --- Executes during object creation, after setting all properties.
function edit10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit11_Callback(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit11 as text
%        str2double(get(hObject,'String')) returns contents of edit11 as a double


% --- Executes during object creation, after setting all properties.
function edit11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit12_Callback(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit12 as text
%        str2double(get(hObject,'String')) returns contents of edit12 as a double


% --- Executes during object creation, after setting all properties.
function edit12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_theta1_Callback(hObject, eventdata, handles)
% hObject    handle to edit_theta1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_theta1 as text
%        str2double(get(hObject,'String')) returns contents of edit_theta1 as a double


% --- Executes during object creation, after setting all properties.
function edit_theta1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_theta1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_theta2_Callback(hObject, eventdata, handles)
% hObject    handle to edit_theta2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_theta2 as text
%        str2double(get(hObject,'String')) returns contents of edit_theta2 as a double


% --- Executes during object creation, after setting all properties.
function edit_theta2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_theta2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_d3_Callback(hObject, eventdata, handles)
% hObject    handle to edit_d3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_d3 as text
%        str2double(get(hObject,'String')) returns contents of edit_d3 as a double


% --- Executes during object creation, after setting all properties.
function edit_d3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_d3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_theta4_Callback(hObject, eventdata, handles)
% hObject    handle to edit_theta4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_theta4 as text
%        str2double(get(hObject,'String')) returns contents of edit_theta4 as a double


% --- Executes during object creation, after setting all properties.
function edit_theta4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_theta4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in run.
function run_Callback(hObject, eventdata, handles)
% hObject    handle to run (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global theta1_pre;
global theta2_pre;
global d3_pre;
global theta4_pre; 

theta1 = (pi/180)*(get(handles.slider_theta1,'Value'));
set(handles.edit_theta1,'String',theta1*180/pi);
theta2 = (pi/180)*(get(handles.slider_theta2,'Value'));
set(handles.edit_theta2,'String',theta2*180/pi);
theta4 = (pi/180)*(get(handles.slider_theta4,'Value'));
set(handles.edit_theta4,'String',theta4*180/pi);
d3 = get(handles.slider_d3,'Value');
set(handles.edit_d3,'String',d3);
opa=0.5;

drawrobot(theta1,theta2,d3,theta4,opa, handles);
set(handles.run, 'Enable', 'off');
theta1_pre = theta1;
theta2_pre = theta2;
d3_pre=d3;
theta4_pre = theta4;



function edit18_Callback(hObject, eventdata, handles)
% hObject    handle to edit18 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit18 as text
%        str2double(get(hObject,'String')) returns contents of edit18 as a double


% --- Executes during object creation, after setting all properties.
function edit18_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit18 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit19_Callback(hObject, eventdata, handles)
% hObject    handle to edit19 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit19 as text
%        str2double(get(hObject,'String')) returns contents of edit19 as a double


% --- Executes during object creation, after setting all properties.
function edit19_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit19 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit20_Callback(hObject, eventdata, handles)
% hObject    handle to edit20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit20 as text
%        str2double(get(hObject,'String')) returns contents of edit20 as a double


% --- Executes during object creation, after setting all properties.
function edit20_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_roll_Callback(hObject, eventdata, handles)
% hObject    handle to edit_roll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_roll as text
%        str2double(get(hObject,'String')) returns contents of edit_roll as a double


% --- Executes during object creation, after setting all properties.
function edit_roll_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_roll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_pitch_Callback(hObject, eventdata, handles)
% hObject    handle to edit_pitch (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_pitch as text
%        str2double(get(hObject,'String')) returns contents of edit_pitch as a double


% --- Executes during object creation, after setting all properties.
function edit_pitch_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_pitch (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_yaw_Callback(hObject, eventdata, handles)
% hObject    handle to edit_yaw (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_yaw as text
%        str2double(get(hObject,'String')) returns contents of edit_yaw as a double


% --- Executes during object creation, after setting all properties.
function edit_yaw_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_yaw (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in checkbox_c0.
function checkbox_c0_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_c0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global theta1_pre;
global theta2_pre;
global d3_pre;
global theta4_pre;
global opa_pre;
opa =get(handles.slider_opa,'Value');

drawrobot(theta1_pre,theta2_pre,d3_pre,theta4_pre,opa,handles);
opa_pre=opa;
% Hint: get(hObject,'Value') returns toggle state of checkbox_c0


% --- Executes on button press in checkbox_c1.
function checkbox_c1_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_c1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global theta1_pre;
global theta2_pre;
global d3_pre;
global theta4_pre;
global opa_pre;
opa =get(handles.slider_opa,'Value');

drawrobot(theta1_pre,theta2_pre,d3_pre,theta4_pre,opa,handles);
opa_pre=opa;
% Hint: get(hObject,'Value') returns toggle state of checkbox_c1


% --- Executes on button press in checkbox_c2.
function checkbox_c2_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_c2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox_c2
global theta1_pre;
global theta2_pre;
global d3_pre;
global theta4_pre;
global opa_pre;
opa =get(handles.slider_opa,'Value');

drawrobot(theta1_pre,theta2_pre,d3_pre,theta4_pre,opa,handles);
opa_pre=opa;

% --- Executes on button press in checkbox_c3.
function checkbox_c3_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_c3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox_c3
global theta1_pre;
global theta2_pre;
global d3_pre;
global theta4_pre;
global opa_pre;
opa =get(handles.slider_opa,'Value');

drawrobot(theta1_pre,theta2_pre,d3_pre,theta4_pre,opa,handles);
opa_pre=opa;

% --- Executes on button press in checkbox_c4.
function checkbox_c4_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_c4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox_c4
global theta1_pre;
global theta2_pre;
global d3_pre;
global theta4_pre;
global opa_pre;
opa =get(handles.slider_opa,'Value');

drawrobot(theta1_pre,theta2_pre,d3_pre,theta4_pre,opa,handles);
opa_pre=opa;

% --- Executes on slider movement.
function slider_opa_Callback(hObject, eventdata, handles)
% hObject    handle to slider_opa (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global theta1_pre;
global theta2_pre;
global d3_pre;
global theta4_pre;
global opa_pre;
opa =get(handles.slider_opa,'Value');
set(handles.edit_opa,'String',opa);

drawrobot(theta1_pre,theta2_pre,d3_pre,theta4_pre,opa,handles);
opa_pre=opa;

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider_opa_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_opa (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit24_Callback(hObject, eventdata, handles)
% hObject    handle to edit24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit24 as text
%        str2double(get(hObject,'String')) returns contents of edit24 as a double


% --- Executes during object creation, after setting all properties.
function edit24_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_opa_Callback(hObject, eventdata, handles)
% hObject    handle to edit_opa (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_opa as text
%        str2double(get(hObject,'String')) returns contents of edit_opa as a double


% --- Executes during object creation, after setting all properties.
function edit_opa_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_opa (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in checkbox_workspace.
function checkbox_workspace_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_workspace (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global theta1_pre;
global theta2_pre;
global d3_pre;
global theta4_pre;
global opa_pre;
opa =get(handles.slider_opa,'Value');

drawrobot(theta1_pre,theta2_pre,d3_pre,theta4_pre,opa,handles);
opa_pre=opa;

% Hint: get(hObject,'Value') returns toggle state of checkbox_workspace



function edit_Xm_Callback(hObject, eventdata, handles)
% hObject    handle to edit_Xm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_Xm as text
%        str2double(get(hObject,'String')) returns contents of edit_Xm as a double


% --- Executes during object creation, after setting all properties.
function edit_Xm_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_Xm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_Ym_Callback(hObject, eventdata, handles)
% hObject    handle to edit_Ym (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_Ym as text
%        str2double(get(hObject,'String')) returns contents of edit_Ym as a double


% --- Executes during object creation, after setting all properties.
function edit_Ym_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_Ym (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_Zm_Callback(hObject, eventdata, handles)
% hObject    handle to edit_Zm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_Zm as text
%        str2double(get(hObject,'String')) returns contents of edit_Zm as a double


% --- Executes during object creation, after setting all properties.
function edit_Zm_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_Zm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in MoveL.
function MoveL_Callback(hObject, eventdata, handles)
% hObject    handle to MoveL (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global theta1_pre;
global theta2_pre;
global d3_pre;
global theta4_pre; 
global opa_pre;
opa= opa_pre;

currentX = str2double(handles.edit_posx.String);
currentY = str2double(handles.edit_posy.String);
currentZ = str2double(handles.edit_posz.String);
yaw_0= str2double(handles.edit_yaw.String);
yaw_1= str2double(handles.edit_yawm.String);

targetx = str2double(handles.edit_Xm.String);
targety = str2double(handles.edit_Ym.String);
targetz = str2double(handles.edit_Zm.String);

a1 = 450; a2= 400;
desiredVector = [targetx - currentX, targety - currentY, targetz - currentZ];
normDesiredVector = norm(desiredVector);

qMax = normDesiredVector;  
formula = @(stdVec) stdVec * desiredVector + [currentX, currentY, currentZ];

aMax = str2double(handles.edit_amax.String);
vMax = sqrt(qMax*aMax);
vmax_set= str2double(handles.edit_vmax.String);
if (vmax_set< vMax)
    vMax=vmax_set;
end

t1      = vMax/aMax;
tm      = (qMax - aMax*t1^2)/vMax;
tmax    = 2*t1 + tm;
t2      = tmax - t1;

t       = 0:0.2:tmax;
lengthT = length(t);
a       = zeros(lengthT,1);
v       = zeros(lengthT,1);
q       = zeros(lengthT,1);
Th_1=zeros(lengthT,1);
Th_2=zeros(lengthT,1);
d_3=zeros(lengthT,1);
Th_4=zeros(lengthT,1);
x_pre=currentX;
y_pre=currentY;
z_pre=currentZ;
yaw_pre=yaw_0;
X=[];
Y=[];
Z=[];
for i = 1:1:lengthT
    if (t(i) < t1)
        a(i) = aMax;
        v(i) = aMax*t(i);
        q(i) = 0.5*aMax*t(i)^2;
    elseif (t(i) < t2)
        a(i) = 0;
        v(i) = vMax;
        q(i) = 0.5*aMax*t1^2 + vMax*(t(i)-t1);
    else
        a(i) = -aMax;
        v(i) = vMax - aMax*(t(i)-t2);
        q(i) = qMax - 0.5*aMax*(tmax-t(i))^2;
    end

    qStd = q/qMax;
    desiredPos = formula(qStd(i));
    yaw = yaw_0+(q(i)/qMax)*(yaw_1-yaw_0);
    X=[X, desiredPos(1)];
    Y=[Y, desiredPos(2)];
    Z=[Z, desiredPos(3)];
    [Theta_1, Theta_2, d3, Theta_4] = Inverse(desiredPos(1), desiredPos(2),desiredPos(3), yaw);
    Th_1(i)=Theta_1;
    Th_2(i)=Theta_2;
    Th_4(i)=Theta_4;
    d_3(i)=d3;

    v_end=[((desiredPos(1)-x_pre)/0.2);
           ((desiredPos(2)-y_pre)/0.2);
           ((desiredPos(3)-z_pre)/0.2);
           ((yaw-yaw_pre)/0.2)];
    Jacobian_Matrix=[   -a2*sin(Theta_1+Theta_2)-a1*sin(Theta_1)    -a2*sin(Theta_1+Theta_2)   0   0;
                         a2*cos(Theta_1+Theta_2)+a1*cos(Theta_1)     a2*cos(Theta_1+Theta_2)   0   0;
                         0                                           0                         1   0;
                         1                                           1                         0   1];
    v_joint=(Jacobian_Matrix)\v_end;
    v_th1(i)=v_joint(1,1);
    v_th2(i)=v_joint(2,1);
    v_d3(i) =v_joint(3,1);
    v_th4(i)=v_joint(4,1);
    x_pre=desiredPos(1);
    y_pre=desiredPos(2);
    z_pre=desiredPos(3);
    yaw_pre=yaw;

    axes(handles.axes2);
    cla(handles.axes2);
    plot(t(1:i), q(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('mm');
    title('q(t)');
    grid on;
    axes(handles.axes3);
    cla(handles.axes3);
    plot(t(1:i), v(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('mm/s');
    title('v(t)');
    grid on;
    axes(handles.axes4);
    cla(handles.axes4);
    plot(t(1:i), a(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('mm/s2');
    title('a(t)');
    grid on;

    set(handles.slider_theta1, 'value',(Theta_1*180/pi));
    set(handles.slider_theta2, 'value',(Theta_2*180/pi));
    set(handles.slider_theta4, 'value',(Theta_4*180/pi));
    set(handles.slider_d3, 'value',d3);

    set(handles.edit_theta1,'string',num2str(Theta_1*180/pi));
    set(handles.edit_theta2,'string',num2str(Theta_2*180/pi));
    set(handles.edit_theta4,'string',num2str(Theta_4*180/pi));
    set(handles.edit_d3,'string',num2str(d3));

    
    drawrobot(Theta_1,Theta_2,d3, Theta_4,opa,handles);
    axes(handles.axes1);
    scatter3(X, Y, Z, 'r', 'filled');
    pause(0.02);
    set(handles.edit_posx,'string',num2str(desiredPos(1)));
    set(handles.edit_posy,'string',num2str(desiredPos(2)));
    set(handles.edit_posz,'string',num2str(desiredPos(3)));
    set(handles.edit_yaw,'string',num2str(yaw));
    theta1_pre = Theta_1;
    theta2_pre = Theta_2;
    d3_pre=d3;
    theta4_pre = Theta_4; 

end
axes(handles.axes5);
cla(handles.axes5);
plot(t(1:i), Th_1(1:i), 'LineWidth', 2);
xlabel('s');
ylabel('rad');
title('Theta1(t)');
grid on;
axes(handles.axes6);
cla(handles.axes6);
plot(t(1:i), Th_2(1:i), 'LineWidth', 2);
xlabel('s');
ylabel('rad');
title('theta2(t)');
grid on;
axes(handles.axes7);
cla(handles.axes7);
plot(t(1:i), d_3(1:i), 'LineWidth', 2);
xlabel('s');
ylabel('mm');
title('d3(t)');
grid on;
axes(handles.axes8);
cla(handles.axes8);
plot(t(1:i), Th_4(1:i), 'LineWidth', 2);
xlabel('s');
ylabel('rad');
title('Theta4(t)');
grid on;
axes(handles.axes9);
cla(handles.axes9);
plot(t(1:i), v_th1(1:i), 'LineWidth', 2);
xlabel('s');
ylabel('rad/s');
title('v_th1(t)');
grid on;
axes(handles.axes10);
cla(handles.axes10);
plot(t(1:i), v_th2(1:i), 'LineWidth', 2);
xlabel('s');
ylabel('rad/s');
title('v_th2(t)');
grid on;
axes(handles.axes11);
cla(handles.axes11);
plot(t(1:i), v_d3(1:i), 'LineWidth', 2);
xlabel('s');
ylabel('mm/s');
title('v_d3(t)');
grid on;
axes(handles.axes12);
cla(handles.axes12);
plot(t(1:i), v_th4(1:i), 'LineWidth', 2);
xlabel('s');
ylabel('rad/s');
title('v_th4(t)');
grid on;


% --- Executes on button press in MoveC.
function MoveC_Callback(hObject, eventdata, handles)
% hObject    handle to MoveC (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global theta1_pre;
global theta2_pre;
global d3_pre;
global theta4_pre; 
global opa_pre;
opa=opa_pre;
a1 = 450; a2= 400;
x_0 = str2double(handles.edit_posx.String);
y_0 = str2double(handles.edit_posy.String);
z_0 = str2double(handles.edit_posz.String);
yaw_0= str2double(handles.edit_yaw.String);
yaw_1= str2double(handles.edit_yawm.String);

x_1 = str2double(handles.edit_x1.String);
y_1 = str2double(handles.edit_y1.String);
z_1 = str2double(handles.edit_z1.String);

x_2 = str2double(handles.edit_x2.String);
y_2 = str2double(handles.edit_y2.String);
z_2 = str2double(handles.edit_z2.String);

%%%%%%%%%%%%%%
AC = [x_2-x_0;y_2-y_0;z_2-z_0];
AB = [x_1-x_0;y_1-y_0;z_1-z_0];
n = cross(AC,AB);
d = n'*[x_0;y_0;z_0];
% Tam duong tron
M = [n';...
    2*(x_1-x_0) 2*(y_1-y_0) 2*(z_1-z_0);...
    2*(x_2-x_0) 2*(y_2-y_0) 2*(z_2-z_0)];
N = [d;x_1^2+y_1^2+z_1^2-x_0^2-y_0^2-z_0^2;x_2^2+y_2^2+z_2^2-x_0^2-y_0^2-z_0^2];
O = M\N;
R= sqrt((x_1-O(1))^2+(y_1-O(2))^2 );

%%%%%%%%%%%%
alpha_0=acos((2*R*R-(x_0-O(1)-R)^2-(y_0-O(2))^2)/(2*R*R));
alpha_1=acos((2*R*R-(x_1-O(1)-R)^2-(y_1-O(2))^2)/(2*R*R));
alpha_2=acos((2*R*R-(x_2-O(1)-R)^2-(y_2-O(2))^2)/(2*R*R));
if (y_0<O(2))
    alpha_0=2*pi-alpha_0;
end
if (y_1<O(2))
    alpha_1=2*pi-alpha_1;
end
if (y_2<O(2))
    alpha_2=2*pi-alpha_2;
end
alpha= alpha_2-alpha_0;
if (alpha>0)
    dau=-1;
    dentaalpha=2*pi-alpha;
else
    dau=1;
    dentaalpha=alpha;
end

qMax    = R*abs(dentaalpha);

aMax = str2double(handles.edit_amax.String);
vMax = sqrt(qMax*aMax);
vmax_set= str2double(handles.edit_vmax.String);
if (vmax_set< vMax)
    vMax=vmax_set;
end

t1      = vMax/aMax;
tm      = (qMax - aMax*t1^2)/vMax;
tmax    = 2*t1 + tm;
t2      = tmax - t1;

t       = 0:0.2:tmax;
lengthT = length(t);
a       = zeros(lengthT,1);
v       = zeros(lengthT,1);
q       = zeros(lengthT,1);
Th_1=zeros(lengthT,1);
Th_2=zeros(lengthT,1);
d_3=zeros(lengthT,1);
Th_4=zeros(lengthT,1);
x_pre=x_0;
y_pre=y_0;
z_pre=z_0;
yaw_pre=yaw_0;
X=[];
Y=[];
Z=[];
for i = 1:1:lengthT
    if (t(i) < t1)
        a(i) = aMax;
        v(i) = aMax*t(i);
        q(i) = 0.5*aMax*t(i)^2;
    elseif (t(i) < t2)
        a(i) = 0;
        v(i) = vMax;
        q(i) = 0.5*aMax*t1^2 + vMax*(t(i)-t1);
    else
        a(i) = -aMax;
        v(i) = vMax - aMax*(t(i)-t2);
        q(i) = qMax - 0.5*aMax*(tmax-t(i))^2;
    end

    x = O(1)+R*cos(alpha_0+(q(i)/qMax)*dentaalpha*dau);
    y = O(2)+R*sin(alpha_0+(q(i)/qMax)*dentaalpha*dau);
    z = -x*n(1)/n(3)-y*n(2)/n(3)+d/n(3)-(q(i)/qMax)*(abs(z_0-z_1));
    yaw = yaw_0+(q(i)/qMax)*(yaw_1-yaw_0);
    X=[X, x];
    Y=[Y, y];
    Z=[Z, z];
    [Theta_1, Theta_2, d3, Theta_4] = Inverse(x, y,z, yaw);
    Th_1(i)=Theta_1;
    Th_2(i)=Theta_2;
    Th_4(i)=Theta_4;
    d_3(i)=d3;

    v_end=[((x-x_pre)/0.2);
           ((y-y_pre)/0.2);
           ((z-z_pre)/0.2);
           ((yaw-yaw_pre)/0.2)];
    Jacobian_Matrix=[   -a2*sin(Theta_1+Theta_2)-a1*sin(Theta_1)    -a2*sin(Theta_1+Theta_2)   0   0;
                         a2*cos(Theta_1+Theta_2)+a1*cos(Theta_1)     a2*cos(Theta_1+Theta_2)   0   0;
                         0                                           0                         1   0;
                         1                                           1                         0   1];
    v_joint=(Jacobian_Matrix)\v_end;
    v_th1(i)=v_joint(1,1);
    v_th2(i)=v_joint(2,1);
    v_d3(i) =v_joint(3,1);
    v_th4(i)=v_joint(4,1);
    x_pre=x;
    y_pre=y;
    z_pre=z;
    yaw_pre=yaw;

    axes(handles.axes2);
    cla(handles.axes2);
    plot(t(1:i), q(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('mm');
    title('q(t)');
    grid on;
    axes(handles.axes3);
    cla(handles.axes3);
    plot(t(1:i), v(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('mm/s');
    title('v(t)');
    grid on;
    axes(handles.axes4);
    cla(handles.axes4);
    plot(t(1:i), a(1:i), 'LineWidth', 2);
    xlabel('s');
    ylabel('mm/s2');
    title('a(t)');
    grid on;

    set(handles.slider_theta1, 'value',(Theta_1*180/pi));
    set(handles.slider_theta2, 'value',(Theta_2*180/pi));
    set(handles.slider_theta4, 'value',(Theta_4*180/pi));
    set(handles.slider_d3, 'value',d3);

    set(handles.edit_theta1,'string',num2str(Theta_1*180/pi));
    set(handles.edit_theta2,'string',num2str(Theta_2*180/pi));
    set(handles.edit_theta4,'string',num2str(Theta_4*180/pi));
    set(handles.edit_d3,'string',num2str(d3));

    
    drawrobot(Theta_1,Theta_2,d3, Theta_4,opa,handles);
    axes(handles.axes1);
    scatter3(X, Y, Z, 'r', 'filled');
    pause(0.02);
    set(handles.edit_posx,'string',num2str(x));
    set(handles.edit_posy,'string',num2str(y));
    set(handles.edit_posz,'string',num2str(z));
    set(handles.edit_yaw,'string',num2str(yaw));
    theta1_pre = Theta_1;
    theta2_pre = Theta_2;
    d3_pre=d3;
    theta4_pre = Theta_4; 

end
axes(handles.axes5);
cla(handles.axes5);
plot(t(1:i), Th_1(1:i), 'LineWidth', 2);
xlabel('s');
ylabel('rad');
title('Theta1(t)');
grid on;
axes(handles.axes6);
cla(handles.axes6);
plot(t(1:i), Th_2(1:i), 'LineWidth', 2);
xlabel('s');
ylabel('rad');
title('theta2(t)');
grid on;
axes(handles.axes7);
cla(handles.axes7);
plot(t(1:i), d_3(1:i), 'LineWidth', 2);
xlabel('s');
ylabel('mm');
title('d3(t)');
grid on;
axes(handles.axes8);
cla(handles.axes8);
plot(t(1:i), Th_4(1:i), 'LineWidth', 2);
xlabel('s');
ylabel('rad');
title('Theta4(t)');
grid on;
axes(handles.axes9);
cla(handles.axes9);
plot(t(1:i), v_th1(1:i), 'LineWidth', 2);
xlabel('s');
ylabel('rad/s');
title('v_th1(t)');
grid on;
axes(handles.axes10);
cla(handles.axes10);
plot(t(1:i), v_th2(1:i), 'LineWidth', 2);
xlabel('s');
ylabel('rad/s');
title('v_th2(t)');
grid on;
axes(handles.axes11);
cla(handles.axes11);
plot(t(1:i), v_d3(1:i), 'LineWidth', 2);
xlabel('s');
ylabel('mm/s');
title('v_d3(t)');
grid on;
axes(handles.axes12);
cla(handles.axes12);
plot(t(1:i), v_th4(1:i), 'LineWidth', 2);
xlabel('s');
ylabel('rad/s');
title('v_th4(t)');
grid on;




% pointss = [];
% for k=1:30 
%     x = O(1)+R*cos(alpha_0+(k/30)*dentaalpha*a);
%     y = O(2)+R*sin(alpha_0+(k/30)*dentaalpha*a);
%     z = -x*n(1)/n(3)-y*n(2)/n(3)+d/n(3);
%     point = [x, y, z];  % Tạo một điểm mới có tọa độ (x, y, z)
%     pointss = [pointss; point];  % Thêm điểm vào mảng points
% end
% disp(pointss);
% opa=opa_pre;
% for i=1:30
%     [theta1, theta2,d3,theta4]= Inverse(pointss(i,1),pointss(i,2),pointss(i,3),yaw);
%     set(handles.slider_theta1, 'value',(theta1*180/pi));
%     set(handles.slider_theta2, 'value',(theta2*180/pi));
%     set(handles.slider_theta4, 'value',(theta4*180/pi));
%     set(handles.slider_d3, 'value',d3);
% 
%     set(handles.edit_theta1,'string',num2str(theta1*180/pi));
%     set(handles.edit_theta2,'string',num2str(theta2*180/pi));
%     set(handles.edit_theta4,'string',num2str(theta4*180/pi));
%     set(handles.edit_d3,'string',num2str(d3));
% 
%     
%     drawrobot(theta1,theta2,d3, theta4,opa,handles);
%     pause(0.2);
%     set(handles.edit_posx,'string',num2str(pointss(i,1)));
%     set(handles.edit_posy,'string',num2str(pointss(i,2)));
%     set(handles.edit_posz,'string',num2str(pointss(i,3)));
%     theta1_pre = theta1;
%     theta2_pre = theta2;
%     d3_pre=d3;
%     theta4_pre = theta4; 
% end


function edit30_Callback(hObject, eventdata, handles)
% hObject    handle to edit30 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit30 as text
%        str2double(get(hObject,'String')) returns contents of edit30 as a double


% --- Executes during object creation, after setting all properties.
function edit30_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit30 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit31_Callback(hObject, eventdata, handles)
% hObject    handle to edit31 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit31 as text
%        str2double(get(hObject,'String')) returns contents of edit31 as a double


% --- Executes during object creation, after setting all properties.
function edit31_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit31 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit32_Callback(hObject, eventdata, handles)
% hObject    handle to edit32 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit32 as text
%        str2double(get(hObject,'String')) returns contents of edit32 as a double


% --- Executes during object creation, after setting all properties.
function edit32_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit32 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_x1_Callback(hObject, eventdata, handles)
% hObject    handle to edit_x1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_x1 as text
%        str2double(get(hObject,'String')) returns contents of edit_x1 as a double


% --- Executes during object creation, after setting all properties.
function edit_x1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_x1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_y1_Callback(hObject, eventdata, handles)
% hObject    handle to edit_y1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_y1 as text
%        str2double(get(hObject,'String')) returns contents of edit_y1 as a double


% --- Executes during object creation, after setting all properties.
function edit_y1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_y1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_z1_Callback(hObject, eventdata, handles)
% hObject    handle to edit_z1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_z1 as text
%        str2double(get(hObject,'String')) returns contents of edit_z1 as a double


% --- Executes during object creation, after setting all properties.
function edit_z1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_z1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_x2_Callback(hObject, eventdata, handles)
% hObject    handle to edit_x2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_x2 as text
%        str2double(get(hObject,'String')) returns contents of edit_x2 as a double


% --- Executes during object creation, after setting all properties.
function edit_x2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_x2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_y2_Callback(hObject, eventdata, handles)
% hObject    handle to edit_y2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_y2 as text
%        str2double(get(hObject,'String')) returns contents of edit_y2 as a double


% --- Executes during object creation, after setting all properties.
function edit_y2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_y2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_z2_Callback(hObject, eventdata, handles)
% hObject    handle to edit_z2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_z2 as text
%        str2double(get(hObject,'String')) returns contents of edit_z2 as a double


% --- Executes during object creation, after setting all properties.
function edit_z2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_z2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit39_Callback(hObject, eventdata, handles)
% hObject    handle to edit39 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit39 as text
%        str2double(get(hObject,'String')) returns contents of edit39 as a double


% --- Executes during object creation, after setting all properties.
function edit39_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit39 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit40_Callback(hObject, eventdata, handles)
% hObject    handle to edit40 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit40 as text
%        str2double(get(hObject,'String')) returns contents of edit40 as a double


% --- Executes during object creation, after setting all properties.
function edit40_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit40 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit41_Callback(hObject, eventdata, handles)
% hObject    handle to edit41 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit41 as text
%        str2double(get(hObject,'String')) returns contents of edit41 as a double


% --- Executes during object creation, after setting all properties.
function edit41_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit41 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit42_Callback(hObject, eventdata, handles)
% hObject    handle to edit42 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit42 as text
%        str2double(get(hObject,'String')) returns contents of edit42 as a double


% --- Executes during object creation, after setting all properties.
function edit42_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit42 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit43_Callback(hObject, eventdata, handles)
% hObject    handle to edit43 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit43 as text
%        str2double(get(hObject,'String')) returns contents of edit43 as a double


% --- Executes during object creation, after setting all properties.
function edit43_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit43 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit44_Callback(hObject, eventdata, handles)
% hObject    handle to edit44 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit44 as text
%        str2double(get(hObject,'String')) returns contents of edit44 as a double


% --- Executes during object creation, after setting all properties.
function edit44_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit44 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_vmax_Callback(hObject, eventdata, handles)
% hObject    handle to edit_vmax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_vmax as text
%        str2double(get(hObject,'String')) returns contents of edit_vmax as a double


% --- Executes during object creation, after setting all properties.
function edit_vmax_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_vmax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_amax_Callback(hObject, eventdata, handles)
% hObject    handle to edit_amax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_amax as text
%        str2double(get(hObject,'String')) returns contents of edit_amax as a double


% --- Executes during object creation, after setting all properties.
function edit_amax_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_amax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit47_Callback(hObject, eventdata, handles)
% hObject    handle to edit47 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit47 as text
%        str2double(get(hObject,'String')) returns contents of edit47 as a double


% --- Executes during object creation, after setting all properties.
function edit47_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit47 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit48_Callback(hObject, eventdata, handles)
% hObject    handle to edit48 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit48 as text
%        str2double(get(hObject,'String')) returns contents of edit48 as a double


% --- Executes during object creation, after setting all properties.
function edit48_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit48 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_yawm_Callback(hObject, eventdata, handles)
% hObject    handle to edit_yawm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_yawm as text
%        str2double(get(hObject,'String')) returns contents of edit_yawm as a double


% --- Executes during object creation, after setting all properties.
function edit_yawm_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_yawm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit50_Callback(hObject, eventdata, handles)
% hObject    handle to edit50 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit50 as text
%        str2double(get(hObject,'String')) returns contents of edit50 as a double


% --- Executes during object creation, after setting all properties.
function edit50_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit50 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
