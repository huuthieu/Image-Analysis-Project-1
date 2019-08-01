function varargout = my_guide_3(varargin)
% Last Modified by GUIDE v2.5 03-Dec-2018 00:48:59
% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @my_guide_3_OpeningFcn, ...
                   'gui_OutputFcn',  @my_guide_3_OutputFcn, ...
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


% --- Executes just before my_guide_3 is made visible.
function my_guide_3_OpeningFcn(hObject, eventdata, handles, varargin)
global A B C D E F stt
F={'STT'};
A={'MSSV'};
B={'Rubric 1'};
C={'Rublic 2'};
D={'Rublic 3'};
E={'Diem tong'};
stt=0;

% Choose default command line output for my_guide_3
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes my_guide_3 wait for user response (see UIRESUME)
% uiwait(handles.figure1);
% --- Outputs from this function are returned to the command line.

function varargout = my_guide_3_OutputFcn(hObject, eventdata, handles) 
% Get default command line output from handles structure
varargout{1} = handles.output;

function pushbutton1_Callback(hObject, eventdata, handles)
%clc
%clear all;
[a b]=uigetfile({'*.*','All Files'});
ScreneImage1=imread([b a]);
%imshow(img);
TEXTImage1 = imread('mau.JPG');
TEXTImagegr1 = rgb2gray(TEXTImage1);
%figure;
%imshow(TEXTImagegr1);
%title('Image of Bird');
%ScreneImage1 = imread('4.jpg');
ScreneImagegr1 = rgb2gray(ScreneImage1);
%figure;
%imshow(ScreneImagegr1);
%title('Image of Screne');

%detect feature points
TEXTPoints1 = detectSURFFeatures(TEXTImagegr1);
ScrenePoints1 = detectSURFFeatures(ScreneImagegr1);
%figure;
%imshow(TEXTImagegr1);
%title('100 strong points of box MSSV');
%hold on;
%plot(selectStrongest(TEXTPoints1,100));

%figure;
%imshow(ScreneImagegr1);
%title('300 strong points of PICTURE');
%hold on;
%plot(selectStrongest(ScrenePoints1,300));

[boxFeatures1,TEXTPoints1] = extractFeatures(TEXTImagegr1,TEXTPoints1);
[sceneFeatures1,ScrenePoints1] = extractFeatures(ScreneImagegr1,ScrenePoints1);

boxPairs1 = matchFeatures(boxFeatures1, sceneFeatures1);

matchedBoxPoints1 = TEXTPoints1(boxPairs1(:,1),:);
matchedScenePoints1 = ScrenePoints1(boxPairs1(:,2),:);

%figure;
%showMatchedFeatures(TEXTImagegr1, ScreneImagegr1, matchedBoxPoints1,matchedScenePoints1,'montage');
%title('Result');

[tform1, inlierBoxPoints1, inlierScenePoints1] = estimateGeometricTransform(matchedBoxPoints1,matchedScenePoints1,'affine');
%figure;
%showMatchedFeatures(TEXTImagegr1, ScreneImagegr1, inlierBoxPoints1,inlierScenePoints1,'montage');
%title('matched points');

boxPolygon1 = [1,1;
    size(TEXTImagegr1,2),1;
    size(TEXTImagegr1,2),size(TEXTImagegr1,1);
    1,size(TEXTImagegr1,1);
    1,1];
newBoxPolygon1 = transformPointsForward(tform1,boxPolygon1);
%figure;
%imshow(ScreneImagegr1);
%hold on;
%line(newBoxPolygon1(:,1),newBoxPolygon1(:,2),'Color','r','LineWidth',2);
%title('Detected Box');
a1=newBoxPolygon1(1,1);
b1=newBoxPolygon1(1,2);
c1=(newBoxPolygon1(3,1)-newBoxPolygon1(1,1));
d1=(newBoxPolygon1(3,2)-newBoxPolygon1(1,2));
imagen1=imcrop(ScreneImage1,[a1 b1 c1 d1]);
%figure('name','result final');
%imshow(imagen1);


%% Nhan dang chu
TEXTImage = imread('5_2.JPG');
TEXTImagegr = rgb2gray(TEXTImage);
%figure;
%imshow(TEXTImagegr);
%title('Image of Bird');
ScreneImage = imagen1;
ScreneImagegr = rgb2gray(ScreneImage);
%figure;
%imshow(ScreneImagegr);
%title('Image of Screne');

%detect feature points
TEXTPoints = detectSURFFeatures(TEXTImagegr);
ScrenePoints = detectSURFFeatures(ScreneImagegr);
%figure;
%imshow(TEXTImagegr);
%title('100 strong points of box MSSV');
%hold on;
%plot(selectStrongest(TEXTPoints,1000));

%figure;
%imshow(ScreneImagegr);
%title('300 strong points of PICTURE');
%hold on;
%plot(selectStrongest(ScrenePoints,1000));

[boxFeatures,TEXTPoints] = extractFeatures(TEXTImagegr,TEXTPoints);
[sceneFeatures,ScrenePoints] = extractFeatures(ScreneImagegr,ScrenePoints);

boxPairs = matchFeatures(boxFeatures, sceneFeatures);

matchedBoxPoints = TEXTPoints(boxPairs(:,1),:);
matchedScenePoints = ScrenePoints(boxPairs(:,2),:);

%figure;
%showMatchedFeatures(TEXTImagegr, ScreneImagegr, matchedBoxPoints,matchedScenePoints,'montage');
%title('Result');

[tform, inlierBoxPoints, inlierScenePoints] = estimateGeometricTransform(matchedBoxPoints,matchedScenePoints,'affine');
%figure;
%showMatchedFeatures(TEXTImagegr, ScreneImagegr, inlierBoxPoints,inlierScenePoints,'montage');
%title('matched points');

boxPolygon = [1,1;
    size(TEXTImagegr,2),1;
    size(TEXTImagegr,2),size(TEXTImagegr,1);
    1,size(TEXTImagegr,1);
    1,1];
newBoxPolygon = transformPointsForward(tform,boxPolygon);
%figure;
%imshow(ScreneImagegr);
%hold on;
%line(newBoxPolygon(:,1),newBoxPolygon(:,2),'Color','b','LineWidth',2);
%title('Detected Text');
a=newBoxPolygon(3,1)*0.75;
b=(newBoxPolygon(3,2)-newBoxPolygon(1,2))*0.059;
c=(newBoxPolygon(3,1)-newBoxPolygon(1,1))*0.6;
d=(newBoxPolygon(3,2)-newBoxPolygon(1,2))*0.28;
imagen=imcrop(ScreneImage,[a b c d]);
%figure('name','anh MSSV')
imshow(imagen)

%% Nhan dang cat chu tu anh
threshold = graythresh(imagen);
imagen =~im2bw(imagen,threshold);
%figure('name','label_1');
%imshow(imagen)
se = strel('square',2);
imagen = imdilate(imagen,se);
imagen = bwareaopen(imagen,70);
imwrite(imagen,'MSSV_bw.jpg')
[L, Ne]=bwlabel(imagen);
propied=regionprops(L,'BoundingBox');
hold on
for n=1:size(propied,1)
    rectangle('Position',propied(n).BoundingBox,'EdgeColor','g','LineWidth',1)
end
hold off
%pause (1)   

%combine
Error='Ready'
set(handles.text2,'String',Error); 
img=imagen1;
mysize=size(img);
img_hsv=rgb2hsv(img);

%tach kenh mau
img_h=img_hsv(:,:,1);
img_s=img_hsv(:,:,2);
img_v=img_hsv(:,:,3);

% tao anh tach' nhung diem mau` do? (h=0.95->1)
[r,c,v] = find(((img_h>=0.95 & img_h<=1)|(img_h>=0 & img_h<=0.02)) & img_s>=0.25 & img_s<=1); 
numind = size(r,1);
O = img;
BW = zeros(mysize(1),mysize(2)); 
for i=1:numind
    O(r(i),c(i),:)=[1 1 1];
    BW(r(i),c(i)) = 255;
end
imwrite(BW,'diemthanhphan_1.jpg')
se = strel('square',2);
img_erode = imerode(BW,se);
se = strel('square',2);
img_erode = imerode(img_erode,se);
%gian? no? 1 lan` de? de~ nhan. dang.
se=strel('square',8)
img_erode = imdilate(img_erode,se);
se=strel('line',20,10)
for i=1:1    
img_erode = imdilate(img_erode,se);
end

BW_2=im2bw(img_erode, 0.5);
imwrite(BW_2,'diemthanhphan_2.jpg')
C=[];           % chua toa do trong tam
stats = regionprops(BW_2,'Area','Centroid')
for i=1: length(stats)
    C=[C;round(stats(i).Centroid,0)];
end
[C_2,ia,ic]=unique(C(:,2));     %  C_2 = C(ia) and C = C_2(ic). ia cho biet trong C, so hang dang xét cua C_2 dung vi trí thu may
txtstr=cell(length(C_2)-1,1);  %tao. khoi' la` chuoi~ 3 hang`, chua' gia' tri diem cua cau 1,2,3
                               %C(ia(i),1) la` hoan`h do. cua trong tam contour. mysize(2)=hoanh do. cua?

%khung diem
if length(C_2)==4   
for i=1:3
    if(C(ia(i),1)/mysize(2))<0.25
    elseif (C(ia(i),1)/mysize(2))<=0.33 % 1d:0.25->0.33
            txtstr{i}=num2str(1);
            img=insertText(img,C(ia(i),:),'1','FontSize',18,'TextColor','black');
    elseif (C(ia(i),1)/mysize(2))<=0.41
            txtstr{i}=num2str(2);
            img=insertText(img,C(ia(i),:),'2','FontSize',18,'TextColor','black');
    elseif (C(ia(i),1)/mysize(2))<=0.49
            txtstr{i}=num2str(3);
            img=insertText(img,C(ia(i),:),'3','FontSize',18,'TextColor','black');
    elseif (C(ia(i),1)/mysize(2))<=0.56
            txtstr{i}=num2str(4);
            img=insertText(img,C(ia(i),:),'4','FontSize',18,'TextColor','black');
    elseif (C(ia(i),1)/mysize(2))<=0.65
            txtstr{i}=num2str(5);
            img=insertText(img,C(ia(i),:),'5','FontSize',18,'TextColor','black');
    end
end
else
    Error= 'Error:Rubric!Try again!'
    set(handles.text2,'String',Error);
    txtstr{:}=13
end

%cat khung diem tong
a_2=C(ia(4),1)-round(0.032*mysize(2),0);
b_2=C(ia(4),2)-round(0.018*mysize(2),0);
c_2=round(0.07*mysize(2),0);
d_2=round(0.04*mysize(2),0);
I2 = imcrop(img,[a_2 b_2 c_2 d_2]);

%tach kenh mau lan 2
mysize2=size(I2);
img_hsv=rgb2hsv(I2);
img_h=img_hsv(:,:,1);
img_s=img_hsv(:,:,2);
img_v=img_hsv(:,:,3);
[r,c,v] = find(((img_h>=0.95 & img_h<=1)|(img_h>=0 & img_h<=0.02)) & img_s>=0.25 & img_s<=1); 
numind = size(r,1);
O = img;
BW = zeros(mysize2(1),mysize2(2)); 
for i=1:numind
    O(r(i),c(i),:)=[1 1 1];
    BW(r(i),c(i)) = 255;
end
se = strel('square',2);
BW = imdilate(BW,se);
se = strel('square',2);
BW = imerode(BW,se);
se = strel('square',3);
BW = imdilate(BW,se);
se = strel('square',3);
BW = imerode(BW,se);
se = strel('square',2);
BW = imdilate(BW,se);
imagen_2 = bwareaopen(BW,30);
[L_2, Ne_2]=bwlabel(imagen_2);
propied=regionprops(L,'BoundingBox');

%nhandang
load nhandangchuso_4.mat;
sz = net.Layers(1).InputSize ;

%nhan dang diem tong
label_3=int8([]);
biendemdiemtong=0;
for n=1:Ne_2
    biendemdiemtong=biendemdiemtong+1
    [r_2,c_2] = find(L_2==n);
    n1=imagen_2(min(r_2):max(r_2),min(c_2):max(c_2));
    n1 = uint8(255 * n1);
    s1=size(n1);
    n1 = padarray(n1,[8 round(abs((s1(1)-s1(2))/2))],0,'both');
    I_read= imresize(n1, [28 28]);
    ten_diemtong=strcat('testdiemtong_',num2str(biendemdiemtong),'.png')
    imwrite(I_read,ten_diemtong)
    I_read = I_read(1:sz(1),1:sz(2),1:sz(3));
    label_tempt = int8(classify(net, I_read))-1;
    label_3=[label_3;label_tempt]
end

if length(label_3)==1
my_diemtong=int32(label_3(1));
elseif length(label_3)==2
my_diemtong=int32(label_3(1))*10+int32(label_3(2));
elseif length(label_3)==3
my_diemtong=int32(label_3(1))*10+int32(label_3(3));
my_diemtong=double(my_diemtong)/10
else 
    Error='Error:final point.!Try again!'
    set(handles.text2,'String',Error);
    my_diemtong=130
end
if my_diemtong>10
    my_diemtong=double(my_diemtong)/10
end

%nhan dang mssv
label_2=int8([]);
biendemmssv=0
for n=1:Ne
    biendemmssv=biendemmssv+1
    [r,c] = find(L==n);
    n1=imagen(min(r):max(r),min(c):max(c));
    s2=size(n1);
    if s2(2)<2*s2(1)
        n1 = uint8(255 * n1);
        n1 = padarray(n1,[8 round(abs((s2(1)-s2(2))/2))],0,'both');
        I_read= imresize(n1, [28 28]);
        ten_mssv=strcat('testmssv_',num2str(biendemmssv),'_.png')
        imwrite(I_read,ten_mssv)
        I_read = I_read(1:sz(1),1:sz(2),1:sz(3));
        label_tempt = int8(classify(net, I_read))-1;
        label_2=[label_2;label_tempt]
    end
end
if length(label_2)==7
    my_mssv=int32(label_2(1))*10^6+int32(label_2(2))*10^5+int32(label_2(3))*10^4+int32(label_2(4))*10^3+int32(label_2(5))*10^2+int32(label_2(6))*10^1+int32(label_2(7));
else
    Error='Error:MSSV!Try again!'
    my_mssv=13
    set(handles.text2,'String',Error);
end
img=insertText(img,C(ia(4),:)-30,num2str(my_diemtong),'FontSize',28,'TextColor','black');
img=insertText(img,[a b],num2str(my_mssv),'FontSize',28,'TextColor','black');
%hold on
imshow(img)

global A B C D E F stt
A{end+1,:}=my_mssv;
B{end+1,:}=str2double(txtstr{1});
C{end+1,:}=str2double(txtstr{2});
D{end+1,:}=str2double(txtstr{3});
E{end+1,:}=my_diemtong;
stt=stt+1;
F{end+1,:}=stt;

global T
T=table(F,A,B,C,D,E)
my_table=get(handles.my_table,'data');
my_table{end+1,1}=stt;
my_table{end,2}=my_mssv;
my_table{end,3}=str2double(txtstr{1});
my_table{end,4}=str2double(txtstr{2});
my_table{end,5}=str2double(txtstr{3});
my_table{end,6}=my_diemtong;
set(handles.my_table,'data',my_table);
Error='Finished'
set(handles.text2,'String',Error);

% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
filename='thien_excel.xlsx';
global T
writetable(T,filename);

% --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(hObject, eventdata, handles)
axis off

function my_table_CreateFcn(hObject, eventdata, handles)
set(hObject, 'Data', cell(1));
