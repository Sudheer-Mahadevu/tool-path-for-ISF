
%% xyz to rapid generator
% This code achieved the following :
% 1) formats the xyz point data into the RAPID code 
% 2) splits routine based on the number of lines limits in the controller

%% Steps
% import the xyz excel file 
% enter the robot configuration details and generate the ABB code
% to know the initial quatranians and workobject details along with tool
% def- create a module/routine/MoveL command using the Teachpendent and use
% the details to format the code.
%  MoveL
%  [[0,40,0],[0,-0.597172,-0.802113,0],[-1,-1,1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],
%  v30, z0,T2_R2\WObj:=KanR2; Example MoveL command form teach pendent
%%
%%Import data from excel files %%%%%%%%%
clear all; clc;
% % specify the folder for excel file
% myFolder_xyz="D:\Roboforming\DHI project\Pyramid\Pyramid_26-6-23";
% % read the name of the file to import from
% xyz_excel= input('Please enter the name of the excel file to import xyz:',"s");
% % read the raw data in the xcel file 
 %T = readmatrix(myFolder_xyz+xyz_excel+'.xlsx');
T = readmatrix('parts\part3\DirectFormingFinal.csv');
% import x y z columns
x = round(T(:,1),2);
y = T(:,2);
z = T(:,3);
% remove NAN values 
x(isnan(x))= [];
y(isnan(y))= [];
z(isnan(z))= [];
xyz =[x y z];
grid on
plot3(x,y,z,"k")

%%%%%%%%%%%%%%%%%%
% create program for Robo 2
% Enter the robot 2 configuration details 
% RAPID FORMAT

module = "DHI";
task = "xyz";
move = "MoveJ";
q = "[9.73406E-05,-0.156597,-0.987662,-8.78936E-05]";
a = "[-1,-1,-3,1]";
b = "[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]";
vel =50;
zz = "z0";
Tool = "T2_R2";
str = "\";
Workobject = "DHI_WO_R2";

%%
% check the length of the xyz file
max =6000;
l = fix(numel(x)/max);
j = 0;  
if l > 1
    fprintf("The code is long and need to be split")

%% create folder for storing RAPID modules

dateStr = datestr(now, 'yyyy-mm-dd_HH-MM');
foldername = ['OutPut_RAPIDCode_R2', dateStr];
mkdir(foldername); % Create a folder with today's date

%%
% create the modules and routine files 
    for m = 1:l
        fid = fopen(sprintf("xyz_R2%d.mod",m),"w+t");
        fprintf(fid,"MODULE xyz_R2%d\n",m);
        fprintf(fid,"    PROC xyz%d()\n",m);
            for i = 1+j*max:max+j*max
              fprintf(fid,"     %s [[%.3f,%.3f,%0.3f],%s,%s,%s],v%.0f,%s,%s%sWObj:=%s;\n",move,x(i),y(i),z(i),q,a,b,vel,zz,Tool,str,Workobject);
            end 

        fprintf(fid,"    ENDPROC\n");
        fprintf(fid,"ENDMODULE\n");
        fclose(fid);
  j = j+1;
    end

% create the final file 
n = i+1;
e = rem(numel(x),max);
fid = fopen(sprintf("xyz_R2%d_end.mod",l+1),"w+t");
        fprintf(fid,"MODULE xyz_R2%d\n",l+1);
        fprintf(fid,"    PROC xyz%d()\n",l+1);
            for k = n:e+n
                %list = T(n+1:e,:);
              fprintf(fid,"     %s [[%0.3f,%.3f,%.3f],%s,%s,%s],v%.0f,%s,%s%sWObj:=%s;\n",move,x(k),y(k),z(k),q,a,b,vel,zz,Tool,str,Workobject);
            end 

        fprintf(fid,"    ENDPROC\n");
        fprintf(fid,"ENDMODULE\n");  
       
        
    fclose(fid);
        
else 
    name = "file";
    fprintf("The total number of line are %d",length(x));
    fid = fopen(sprintf("%s.mod",name),"w+t");
        fprintf(fid,"MODULE xyz_R2%d\n",m);
        fprintf(fid,"    PROC %s()\n",task);
            for t = 1:length(x)
                %list = T(n+1:e,:);
              fprintf(fid,"     %s [[%.3f,%.3f,%.3f],%s,%s,%s],v%.0f,%s,%s%sWObj:=%s;\n",move,x(t),y(t),z(t),q,a,b,vel,zz,Tool,str,Workobject);
            end 

        fprintf(fid,"    ENDPROC\n");
        fprintf(fid,"ENDMODULE\n");  
       
     fclose(fid);

end
% Create Main module code :
% program format
% Load \Dynamic, "Home:/ABBCode/xyz_R21.mod";
 %   %"xyz1"% ;
   % UnLoad  "Home:/ABBCode/xyz_R21.mod";
  location ='\Dynamic, "Home:/ABBCode/';
   routineNumber = '   %"xyz';
   fid = fopen(sprintf("xyz_R2%dMainModule.txt",1),"w+t");
     for m = 1:l
              fprintf(fid,'Load %sxyz_R2%d.mod";\n',location,m);
              fprintf(fid,'%s%d"% ; \n',routineNumber,m);
              fprintf(fid,"Unload %sxyz_R2%d.mod;\n",location,m);
     end 
        fclose(fid);
 
 
%%
%%%%%%%%%%%%%%%%%%#############################################################
% create program for Robo1
% ##########################################################################
clc;clear all;
T = readmatrix('Flat_updatedR1.xlsx');
% import x y z columns
x = round(T(:,1),2);
y = T(:,2);
z = T(:,3);
% remove NAN values 
x(isnan(x))= [];
y(isnan(y))= [];
z(isnan(z))= [];
% update the x axis to -negative as the coordinate frame is mirrored 
xyz =[x y z];
grid on
%plot3(x(1079:end),y(1079:end),z(1079:end),"k")
% format the R1 toolpath


%%%%%%%%%%%%%%%%%%
% create program for Robo1 
% Enter the robot 1 configuration details 
% RAPID FORMAT

module = "DHI_R1";
task = "xyz";
move = "MoveL";
q = "[4.59201E-06,-0.255073,-0.966922,-3.25308E-06]";
a = "[0,2,-1,0]";
b = "[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]";
vel =50;
zz = "z0";
Tool = "T2_R1";
str = "\";
Workobject = "DHI_WO_R1";

% Enter the robot 1 configuration details 
% RAPID FOrmat

%%
% check the length of the xyz file
max =2000;
l = fix(numel(x)/max);
j = 0;  
%% create folder for storing RAPID modules
dateStr = datestr(now, 'yyyy-mm-dd_HH-MM');
foldername = ['OutPut_RAPIDCode_R1', dateStr];
mkdir(foldername); % Create a folder with today's date

if l > 1
    fprintf("The code is long and need to be split")
%%  
% create the modules and routine files 
    for m = 1:l
        fid = fopen(sprintf("xyz_R1%d.mod",m),"w+t");
        fprintf(fid,"MODULE xyz_R1%d\n",m);
        fprintf(fid,"    PROC xyzR1%d()\n",m);
            for i = 1+j*max:max+j*max
              fprintf(fid,"     %s [[%.3f,%.3f,%0.3f],%s,%s,%s],v%.0f,%s,%s%sWObj:=%s;\n",move,x(i),y(i),z(i),q,a,b,vel,zz,Tool,str,Workobject);
            end 

        fprintf(fid,"    ENDPROC\n");
        fprintf(fid,"ENDMODULE\n");
        fclose(fid);
  j = j+1;
    end

% create the final file 
n = i+1;
e = rem(numel(x),max);
fid = fopen(sprintf("xyz_R1%d_end.mod",l+1),"w+t");
        fprintf(fid,"MODULE xyz_R1%d\n",l+1);
        fprintf(fid,"    PROC xyzR1%d()\n",l+1);
            for k = n:e+n
                %list = T(n+1:e,:);
              fprintf(fid,"     %s [[%0.3f,%.3f,%.3f],%s,%s,%s],v%.0f,%s,%s%sWObj:=%s;\n",move,x(k),y(k),z(k),q,a,b,vel,zz,Tool,str,Workobject);
            end 

        fprintf(fid,"    ENDPROC\n");
        fprintf(fid,"ENDMODULE\n");  
       
        
    fclose(fid);
        
else 
 
    fprintf("The total number of line are %d",length(x));
    fid = fopen(sprintf("xyz_R1%d.mod",1),"w+t");
        fprintf(fid,"MODULE xyz_R1%d\n",1);
        fprintf(fid,"    PROC xyzR1%d()\n",1);
            for t = 1:length(x)
                %list = T(n+1:e,:);
              fprintf(fid,"     %s [[%.3f,%.3f,%.3f],%s,%s,%s],v%.0f,%s,%s%sWObj:=%s;\n",move,x(t),y(t),z(t),q,a,b,vel,zz,Tool,str,Workobject);
            end 

        fprintf(fid,"    ENDPROC\n");
        fprintf(fid,"ENDMODULE\n");  
       
     fclose(fid);

end
