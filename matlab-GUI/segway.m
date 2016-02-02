% Copyright (C) 2015 Paulo Costa
% 
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% 
% GNU General Public License for more details.
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.


function varargout = segway(varargin)
% SEGWAY MATLAB code for segway.fig
%      SEGWAY, by itself, creates a new SEGWAY or raises the existing
%      singleton*.
%
%      H = SEGWAY returns the handle to a new SEGWAY or the handle to
%      the existing singleton*.
%
%      SEGWAY('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SEGWAY.M with the given input arguments.
%
%      SEGWAY('Property','Value',...) creates a new SEGWAY or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before segway_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to segway_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help segway

% Last Modified by GUIDE v2.5 16-Dec-2015 19:09:29

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @segway_OpeningFcn, ...
                   'gui_OutputFcn',  @segway_OutputFcn, ...
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


% --- Executes just before segway is made visible.
function segway_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to segway (see VARARGIN)

% Choose default command line output for segway
handles.output = hObject;
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes segway wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = segway_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in plot.
function plot_Callback(hObject, eventdata, handles)
% hObject    handle to plot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

s = handles.comport;
st = s.status
if(strcmp(st,'closed'))
    fopen(s);
    disp('opened');
end
if(s.BytesAvailable > 0)
    out = fread(s,s.BytesAvailable);
end

axes(handles.axes3);
 
% Plot parameters
numOfPoints = 100;
pointsPerCycle  = 2;
numcols = 3;

% Storage vectors
valvec = zeros(numOfPoints, numcols);
% Sample axis
x = 0:1:numOfPoints-1;

% Set up plot
% h = plot(x,val1vec,'red', x, val2vec, 'green', ...
%     x, val3vec, 'blue');
h = plot(x,valvec);

xlabel('Samples');
title('Mobile Inverted Pendulum Control');
ylim([-10,10]);
xlim([0,numOfPoints-1]);
set(h,'LineWidth',1.8);
i = 1;


handles.finishp = 0;
guidata(hObject,handles);
set(handles.finishp,'UserData',0);

p = get(handles.finishp,'UserData');

while(~p)
   
    % Serial input << MUST >> be in the form x,y,z
   % for the scans to be meaningful. No results will
   % be obtained if there is not an input
   % Make sure the COM port has a valid serial connection
   % and that it follows the format above.
   p = get(handles.finishp,'UserData');
   %disp(p);
   
  
   % Get string
   
   C = zeros(pointsPerCycle,numcols);
   fmt = repmat('%f',1,numcols);
   % Transform string values into floats
   
   for k = 1:pointsPerCycle
       out = fscanf(s,fmt);
       sout = size(out);
       if(sout(1) == numcols && sout(2) == 1)
           C(k,:) = out;
       end
   end
   
   nBlocks = floor(numOfPoints/pointsPerCycle);
   if (i < nBlocks)
       place = (i-1)*pointsPerCycle+1;
       for n=1:numcols
            valvec(place:place+pointsPerCycle-1,n) = C(:,n);
       end
   else
        for j = 1:nBlocks-1
            place = (j-1)*pointsPerCycle+1;
            place2= j*pointsPerCycle+1;
            for n=1:numcols
                valvec(place:place+pointsPerCycle-1,n) = valvec(place2:place2+pointsPerCycle-1,n);
            end
        end
        place = (nBlocks-1)*pointsPerCycle + 1;
        for n=1:numcols
            valvec(place:place+pointsPerCycle-1,n) = C(:,n);
        end
   end
   
   place = valvec(:,3);
   valvec(:,3) = 0.08*place;
   totalVec = num2cell(valvec',2);
   valvec(:,3) = place;
   
   i=i+1;
   
   set(h,'XData',x,{'YData'},totalVec);
   drawnow;   % Updates the graph faster
   
end
fclose(s);
   
   
   
%    val1 = 0;
%    val2 = 0;
%    val3 = 0;
%    if(rxl >= 1)
%     val1 = str2double(C(1));
%    if(rxl >= 2)
%    val2  = str2double(C(2));
%    if (rxl >= 3)
%    val3 = str2double(C(3));
%    end
%    end
%    end
%    
%    
%    
%    %% Plot
%    if(i < numOfPoints)
%       val1vec(i) = val1;
%       val2vec(i) = val2;
%       val3vec(i) = val3;
%    else
%       for j = 1:numOfPoints-1
%          val1vec(j) = val1vec(j+1); 
%          val2vec(j) = val2vec(j+1);
%          val3vec(j) = val3vec(j+1);
%       end
%       val1vec(numOfPoints) = val1;
%       val2vec(numOfPoints) = val2;
%       val3vec(numOfPoints) = val3;
%    end
%    totalVec = num2cell([val1vec ; val3vec; val2vec],2);
%    
%    
%    if(mod(i,50) == 0) 
%        set(h,'XData',x,{'YData'},totalVec);
%        drawnow;   % Updates the graph faster
%    end
%    i = i +  1;
%     
% end

% set(handles.text1,'BackgroundColor','red');
% %s = handles.comport;
% fclose(s);
% delete(s);
% clear('s');
% delete(instrfindall);

function comport_text_Callback(hObject, eventdata, handles)
% hObject    handle to comport_text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.comport_text = get(hObject,'String');
guidata(hObject,handles);

% Hints: get(hObject,'String') returns contents of comport_text as text
%        str2double(get(hObject,'String')) returns contents of comport_text as a double


% --- Executes during object creation, after setting all properties.
function comport_text_CreateFcn(hObject, eventdata, handles)
% hObject    handle to comport_text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

set(handles.text1,'BackgroundColor','red');
s = handles.comport;
fclose(s);
clear s;
delete(instrfindall);

% --- Executes on button press in connect.
function connect_Callback(hObject, eventdata, handles)
% hObject    handle to connect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% 


val =get(handles.popupmenu1,'Value');
comname = get(handles.popupmenu1,'String');
comname = comname{val};
disp(val);
disp(comname);

s = serial(comname);
set(s,'Baudrate',115200); %later change to 38400
fopen(s);

if(isvalid(s) == 1) 
    set(handles.text1,'BackgroundColor', 'Green');
end
   
handles.comport = s;
guidata(hObject,handles);

% --- Executes on key press with focus on connect and none of its controls.
function connect_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to connect (see GCBO)
% eventdata  structure with the following fields (see UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

set(handles.finishp,'UserData',1);

% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over connect.
function connect_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to connect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object creation, after setting all properties.
function axes3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes3


% --- Executes on mouse press over axes background.
function axes3_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to axes3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1




% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
delete(instrfindall);

avai = instrhwinfo('serial');
avaiport=avai.AvailableSerialPorts;

set(hObject ,'String', avaiport);


% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on key press with focus on pushbutton6 and none of its controls.
function pushbutton6_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  structure with the following fields (see UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object creation, after setting all properties.
function pushbutton6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called



function edit7_Callback(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit7 as text
%        str2double(get(hObject,'String')) returns contents of edit7 as a double


% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit8_Callback(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit8 as text
%        str2double(get(hObject,'String')) returns contents of edit8 as a double


% --- Executes during object creation, after setting all properties.
function edit8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
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


% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
kp = get(handles.edit7,'String');
ki = get(handles.edit8,'String');
kd = get(handles.edit9,'String');
tref = get(handles.edit11,'String');
Tms  = get(handles.edit12,'String');

s = handles.comport;
st = s.status
strcmp(st,'closed')
try
    fopen(s);
catch
    disp('oops');
end
if(isvalid(s))
   text_to_send = [kp,',',ki,',',kd,',',tref,',',Tms];
   try
       fprintf(s,text_to_send,'async');
       pause(0.05);
       stopasync(s)
   catch
       disp('oops');
   end
   fclose(s);
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
