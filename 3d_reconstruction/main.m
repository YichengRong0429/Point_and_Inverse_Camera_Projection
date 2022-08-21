%%load video information
%%please clear the value everytime you run it
load('Subject4-Session3-Take4_mocapJoints.mat');
load('vue2CalibInfo.mat');
load('vue4CalibInfo.mat');
videofile1 = 'Subject4-Session3-24form-Full-Take4-Vue2.mp4';
videofile2 = 'Subject4-Session3-24form-Full-Take4-Vue4.mp4';
vue2infor = VideoReader(videofile1);
vue4infor = VideoReader(videofile2);
mocapFnum = 20000; 
x = mocapJoints(mocapFnum,:,1); 
y = mocapJoints(mocapFnum,:,2); 
z = mocapJoints(mocapFnum,:,3); 
x1 =mocapJoints(mocapFnum,:,1); 
y1 =mocapJoints(mocapFnum,:,2); 
z1 = mocapJoints(mocapFnum,:,3); 
conf = mocapJoints(mocapFnum,:,4); 
conf1 = mocapJoints(mocapFnum,:,4);
vue2infor.CurrentTime = (mocapFnum-1)*(50/100)/vue2infor.FrameRate;
video2F = readFrame(vue2infor);
vue4infor.CurrentTime = (mocapFnum-1)*(50/100)/vue4infor.FrameRate;
video4F = readFrame(vue4infor);




%%3d to 2d left camera
pk = vue2.Kmat*vue2.Pmat*[x;y;z;1,1,1,1,1,1,1,1,1,1,1,1];
figure(1);
image(video2F);
hold on;

tdx1 = zeros(1,12);
tdy1 = zeros(1,12);
tdx1(1) = pk(1,1)/pk(3,1);
tdy1(1) = pk(2,1)/pk(3,1);

tdx1(2) = pk(1,2)/pk(3,2);
tdy1(2) = pk(2,2)/pk(3,2);

tdx1(3) = pk(1,3)/pk(3,3);
tdy1(3) = pk(2,3)/pk(3,3);

tdx1(4) = pk(1,4)/pk(3,4);
tdy1(4) = pk(2,4)/pk(3,4);

tdx1(5) = pk(1,5)/pk(3,5);
tdy1(5) = pk(2,5)/pk(3,5);

tdx1(6) = pk(1,6)/pk(3,6);
tdy1(6) = pk(2,6)/pk(3,6);

tdx1(7) = pk(1,7)/pk(3,7);
tdy1(7) = pk(2,7)/pk(3,7);

tdx1(8) = pk(1,8)/pk(3,8);
tdy1(8) = pk(2,8)/pk(3,8);

tdx1(9) = pk(1,9)/pk(3,9);
tdy1(9) = pk(2,9)/pk(3,9);

tdx1(10) = pk(1,10)/pk(3,10);
tdy1(10) = pk(2,10)/pk(3,10);

tdx1(11) = pk(1,11)/pk(3,11);
tdy1(11) = pk(2,11)/pk(3,11);

tdx1(12) = pk(1,12)/pk(3,12);
tdy1(12) = pk(2,12)/pk(3,12);

%%draw the shape
stohx1=(tdx1(1)+tdx1(4))/2;
stohy1=(tdx1(10)+tdx1(7))/2;
stohx2=(tdy1(1)+tdy1(4))/2;
stohy2=(tdy1(10)+tdy1(7))/2;
plot([stohx1,stohy1],[stohx2,stohy2]);
plot([tdx1(6),tdx1(5),tdx1(4),tdx1(1),tdx1(2),tdx1(3)],[tdy1(6),tdy1(5),tdy1(4),tdy1(1),tdy1(2),tdy1(3)]);
plot([tdx1(12),tdx1(11),tdx1(10),tdx1(7),tdx1(8),tdx1(9)],[tdy1(12),tdy1(11),tdy1(10),tdy1(7),tdy1(8),tdy1(9)]);
hold off;





%%3d to 2d right camera
pk = vue4.Kmat*vue4.Pmat*[x;y;z;1,1,1,1,1,1,1,1,1,1,1,1];
tdx2 = zeros(1,12);
tdy2 = zeros(1,12);
figure(2);
image(video4F);
hold on;
tdx2(1) = pk(1,1)/pk(3,1);
tdy2(1) = pk(2,1)/pk(3,1);

tdx2(2) = pk(1,2)/pk(3,2);
tdy2(2) = pk(2,2)/pk(3,2);

tdx2(3) = pk(1,3)/pk(3,3);
tdy2(3) = pk(2,3)/pk(3,3);

tdx2(4) = pk(1,4)/pk(3,4);
tdy2(4) = pk(2,4)/pk(3,4);

tdx2(5) = pk(1,5)/pk(3,5);
tdy2(5) = pk(2,5)/pk(3,5);

tdx2(6) = pk(1,6)/pk(3,6);
tdy2(6) = pk(2,6)/pk(3,6);

tdx2(7) = pk(1,7)/pk(3,7);
tdy2(7) = pk(2,7)/pk(3,7);

tdx2(8) = pk(1,8)/pk(3,8);
tdy2(8) = pk(2,8)/pk(3,8);

tdx2(9) = pk(1,9)/pk(3,9);
tdy2(9) = pk(2,9)/pk(3,9);

tdx2(10) = pk(1,10)/pk(3,10);
tdy2(10) = pk(2,10)/pk(3,10);

tdx2(11) = pk(1,11)/pk(3,11);
tdy2(11) = pk(2,11)/pk(3,11);

tdx2(12) = pk(1,12)/pk(3,12);
tdy2(12) = pk(2,12)/pk(3,12);
%%draw the shape
stohx1=(tdx2(1)+tdx2(4))/2;
stohy1=(tdx2(10)+tdx2(7))/2;
stohx2=(tdy2(1)+tdy2(4))/2;
stohy2=(tdy2(10)+tdy2(7))/2;
plot([stohx1,stohy1],[stohx2,stohy2]);
plot([tdx2(6),tdx2(5),tdx2(4),tdx2(1),tdx2(2),tdx2(3)],[tdy2(6),tdy2(5),tdy2(4),tdy2(1),tdy2(2),tdy2(3)]);
plot([tdx2(12),tdx2(11),tdx2(10),tdx2(7),tdx2(8),tdx2(9)],[tdy2(12),tdy2(11),tdy2(10),tdy2(7),tdy2(8),tdy2(9)]);
hold off;





%%2d to 3d
ptrans1 = vue2.Kmat*vue2.Pmat;
ptrans2 = vue4.Kmat*vue4.Pmat;
P1row1 = ptrans1(1,:);
P1row2 = ptrans1(2,:);
P1row3 = ptrans1(3,:);
P2row1 = ptrans2(1,:);
P2row2 = ptrans2(2,:);
P2row3 = ptrans2(3,:);

x3d = zeros(1,12);
y3d = zeros(1,12);
z3d = zeros(1,12);
for i = 1:12
    if conf(i) > 0
    newMR1 = tdy1(i)*P1row3-P1row2;
    newMR2 = P1row1-tdx1(i)*P1row3;
    newMR3 = tdy2(i)*P2row3-P2row2;
    newMR4 = P2row1-tdx2(i)*P2row3;
    A = [newMR1;newMR2;newMR3;newMR4];
    %%[a,b,answer] = svds(A,4);
    [answer,~] = eigs(A' *A);
    x3d(i) = answer(1,4)/answer(4,4);
    y3d(i) = answer(2,4)/answer(4,4);
    z3d(i) = answer(3,4)/answer(4,4);
    end
end
%%draw the shape
stohx1=(x3d(1)+x3d(4))/2;
stohy1=(x3d(10)+x3d(7))/2;
stohx2=(y3d(1)+y3d(4))/2;
stohy2=(y3d(10)+y3d(7))/2;
ztoz1 = (z3d(1)+z3d(4))/2;


ztoz2 = (z3d(10)+z3d(7))/2;
stohx12=(x1(1)+x1(4))/2;
stohy12=(x1(10)+x1(7))/2;
stohx22=(y1(1)+y1(4))/2;
stohy22=(y1(10)+y1(7))/2;
ztoz12 = (z1(1)+z1(4))/2;
ztoz22 = (z1(10)+z1(7))/2;
figure(3);
  plot3([x1(6),x1(5),x1(4),x1(1),x1(2),x1(3)],[y1(6),y1(5),y1(4),y1(1),y1(2),y1(3)],[z1(6),z1(5),z1(4),z1(1),z1(2),z1(3)],'Color','b');
  hold on;
  plot3([x1(12),x1(11),x1(10),x1(7),x1(8),x1(9)],[y1(12),y1(11),y1(10),y1(7),y1(8),y1(9)],[z1(12),z1(11),z1(10),z1(7),z1(8),z1(9)],'Color','b');
  hold on;
  plot3([stohx12,stohy12],[stohx22,stohy22],[ztoz12,ztoz22],'Color','b');
  hold on;



 plot3([x3d(6),x3d(5),x3d(4),x3d(1),x3d(2),x3d(3)],[y3d(6),y3d(5),y3d(4),y3d(1),y3d(2),y3d(3)],[z3d(6),z3d(5),z3d(4),z3d(1),z3d(2),z3d(3)],'Color','g');
 hold on;
 plot3([x3d(12),x3d(11),x3d(10),x3d(7),x3d(8),x3d(9)],[y3d(12),y3d(11),y3d(10),y3d(7),y3d(8),y3d(9)],[z3d(12),z3d(11),z3d(10),z3d(7),z3d(8),z3d(9)],'Color','g');
 hold on;
 plot3([stohx1,stohy1],[stohx2,stohy2],[ztoz1,ztoz2],'Color','g');
 hold on;









%%Epipole
kinverse = inv(vue2.Kmat);
kinverseTrans = inv(vue4.Kmat)';
zerosandone = [0 0 0 1];
newvue2 = [vue2.Pmat;zerosandone];
newvue4 = [vue4.Pmat;zerosandone];



newm = newvue2*inv(newvue4);
m1 = newm(1:3,1:3);
m2 = newm(1:3,4);
m3 = [0 -1*m2(3) m2(2);m2(3) 0 -1*m2(1);-1*m2(2) m2(1) 0];



Ematrix = inv(m1)*m3;
Fmatrix = kinverseTrans*Ematrix*kinverse;
figure(4);
%%right camera
subplot(1,2,1);
image(video2F);
hold on;
for i = 1:12
line = Fmatrix'*[tdx2(i);tdy2(i);1];
xP=1:2000;
yP=(-1*line(3)-line(1)*xP)/line(2);
scatter(xP,yP);
hold on;
scatter(tdx1(i),tdy1(i));
end
%%left camera
subplot(1,2,2);
image(video4F);
hold on;
for i = 1:12
line = Fmatrix*[tdx1(i);tdy1(i);1];
xP=1:2000;
yP=(-1*line(3)-line(1)*xP)/line(2);
scatter(xP,yP);
hold on;
scatter(tdx2(i),tdy2(i));
end




%%Evaluation
countd1 = zeros(21614,1);
countd2 = zeros(21614,1);
countd3 = zeros(21614,1);

countd4 = zeros(21614,1);
countd5 = zeros(21614,1);
countd6 = zeros(21614,1);

countd7 = zeros(21614,1);
countd8 = zeros(21614,1);
countd9 = zeros(21614,1);

countd10 = zeros(21614,1);
countd11 = zeros(21614,1);
countd12 = zeros(21614,1);
count1 = 1;
count2 = 1;
count3 = 1;
count4 = 1;
count5 = 1;
count6 = 1;
count7 = 1;
count8 = 1;
count9 = 1;
count10 = 1;
count11 = 1;
count12 = 1;
L = zeros(21614,12);
final = zeros(21614,1);
final2 = zeros(5,12);
stat = zeros(12:5);
for i=1:21614
mocapFnum = i;
x = mocapJoints(mocapFnum,:,1);
y = mocapJoints(mocapFnum,:,2);
z = mocapJoints(mocapFnum,:,3);
conf = mocapJoints(mocapFnum,:,4);
if min(conf)>0
%%left camera 3d to 2d
pk = vue2.Kmat*vue2.Pmat*[x;y;z;1,1,1,1,1,1,1,1,1,1,1,1];
tdx1 = zeros(1,12);
tdy1 = zeros(1,12);
tdx1(1) = pk(1,1)/pk(3,1);
tdy1(1) = pk(2,1)/pk(3,1);

tdx1(2) = pk(1,2)/pk(3,2);
tdy1(2) = pk(2,2)/pk(3,2);

tdx1(3) = pk(1,3)/pk(3,3);
tdy1(3) = pk(2,3)/pk(3,3);

tdx1(4) = pk(1,4)/pk(3,4);
tdy1(4) = pk(2,4)/pk(3,4);

tdx1(5) = pk(1,5)/pk(3,5);
tdy1(5) = pk(2,5)/pk(3,5);

tdx1(6) = pk(1,6)/pk(3,6);
tdy1(6) = pk(2,6)/pk(3,6);

tdx1(7) = pk(1,7)/pk(3,7);
tdy1(7) = pk(2,7)/pk(3,7);

tdx1(8) = pk(1,8)/pk(3,8);
tdy1(8) = pk(2,8)/pk(3,8);

tdx1(9) = pk(1,9)/pk(3,9);
tdy1(9) = pk(2,9)/pk(3,9);

tdx1(10) = pk(1,10)/pk(3,10);
tdy1(10) = pk(2,10)/pk(3,10);

tdx1(11) = pk(1,11)/pk(3,11);
tdy1(11) = pk(2,11)/pk(3,11);

tdx1(12) = pk(1,12)/pk(3,12);
tdy1(12) = pk(2,12)/pk(3,12);



%%right camera 3d to 2d

pk = vue4.Kmat*vue4.Pmat*[x;y;z;1,1,1,1,1,1,1,1,1,1,1,1];
tdx2 = zeros(1,12);
tdy2 = zeros(1,12);
tdx2(1) = pk(1,1)/pk(3,1);
tdy2(1) = pk(2,1)/pk(3,1);

tdx2(2) = pk(1,2)/pk(3,2);
tdy2(2) = pk(2,2)/pk(3,2);

tdx2(3) = pk(1,3)/pk(3,3);
tdy2(3) = pk(2,3)/pk(3,3);

tdx2(4) = pk(1,4)/pk(3,4);
tdy2(4) = pk(2,4)/pk(3,4);

tdx2(5) = pk(1,5)/pk(3,5);
tdy2(5) = pk(2,5)/pk(3,5);

tdx2(6) = pk(1,6)/pk(3,6);
tdy2(6) = pk(2,6)/pk(3,6);

tdx2(7) = pk(1,7)/pk(3,7);
tdy2(7) = pk(2,7)/pk(3,7);

tdx2(8) = pk(1,8)/pk(3,8);
tdy2(8) = pk(2,8)/pk(3,8);

tdx2(9) = pk(1,9)/pk(3,9);
tdy2(9) = pk(2,9)/pk(3,9);

tdx2(10) = pk(1,10)/pk(3,10);
tdy2(10) = pk(2,10)/pk(3,10);

tdx2(11) = pk(1,11)/pk(3,11);
tdy2(11) = pk(2,11)/pk(3,11);

tdx2(12) = pk(1,12)/pk(3,12);
tdy2(12) = pk(2,12)/pk(3,12);



%%2d to 3d
ptrans1 = vue2.Kmat*vue2.Pmat;
ptrans2 = vue4.Kmat*vue4.Pmat;
P1row1 = ptrans1(1,:);
P1row2 = ptrans1(2,:);
P1row3 = ptrans1(3,:);
P2row1 = ptrans2(1,:);
P2row2 = ptrans2(2,:);
P2row3 = ptrans2(3,:);

x3d = zeros(1,12);
y3d = zeros(1,12);
z3d = zeros(1,12);

for j = 1:12
    if conf(j) > 0
    newMR1 = (tdy1(j)*P1row3)-P1row2;
    newMR2 = P1row1-(tdx1(j)*P1row3);
    newMR3 = (tdy2(j)*P2row3)-P2row2;
    newMR4 = P2row1-(tdx2(j)*P2row3);
    A = [newMR1;newMR2;newMR3;newMR4];
    %%[answer,~] = eigs(A' *A);
    [a,b,answer] = svds(A,4);
    x3d(j) = answer(1,4)/answer(4,4);
    y3d(j) = answer(2,4)/answer(4,4);
    z3d(j) = answer(3,4)/answer(4,4);
    end
end


for k = 1:12
    xsq = (x(k)-x3d(k))^2;
    ysq = (y(k)-y3d(k))^2;
    zsq = (z(k)-z3d(k))^2;
    L(i,k) = sqrt(xsq + ysq + zsq);
    final(i,1) = final(i,1) + L(i,k);
    if k==1
        countd1(count1,1)=L(i,k);
        count1=count1+1;
    end  
    if k==2
        countd2(count2,1)=L(i,k);
        count2=count2+1;
    end 
    if k==3
        countd3(count3,1)=L(i,k);
        count3=count3+1;
    end 
    if k==4
        countd4(count4,1)=L(i,k);
        count4=count4+1;
    end 
    if k==5
        countd5(count5,1)=L(i,k);
        count5=count5+1;
    end 
    if k==6
        countd6(count6,1)=L(i,k);
        count6=count6+1;
    end 
    if k==7
        countd7(count7,1)=L(i,k);
        count7=count7+1;
    end 
    if k==8
        countd8(count8,1)=L(i,k);
        count8=count8+1;
    end 
    if k==9
        countd9(count9,1)=L(i,k);
        count9=count9+1;
    end
    if k==10
        countd10(count10,1)=L(i,k);
        count10=count10+1;
    end 
    if k==11
        countd11(count11,1)=L(i,k);
        count11=count11+1;
    end 
    if k==12
        countd12(count12,1)=L(i,k);
        count12=count12+1;
    end 
end
end    
end

for k=1:12
    if k==1
        final2(1,k) = median(countd1);
    end  
    if k==2
        final2(1,k) = median(countd2);
    end 
    if k==3
        final2(1,k) = median(countd3);
    end 
    if k==4
        final2(1,k) = median(countd4);
    end 
    if k==5
        final2(1,k) = median(countd5);
    end 
    if k==6
        final2(1,k) = median(countd6);
    end 
    if k==7
        final2(1,k) = median(countd7);
    end 
    if k==8
        final2(1,k) = median(countd8);
    end 
    if k==9
        final2(1,k) = median(countd9);
    end
    if k==10
        final2(1,k) = median(countd10);
    end 
    if k==11
        final2(1,k) = median(countd11);
    end 
    if k==12
        final2(1,k) = median(countd12);
    end 
end

for k=1:12
    if k==1
        final2(2,k) = mean(countd1);
    end  
    if k==2
        final2(2,k) = mean(countd2);
    end 
    if k==3
        final2(2,k) = mean(countd3);
    end 
    if k==4
        final2(2,k) = mean(countd4);
    end 
    if k==5
        final2(2,k) = mean(countd5);
    end 
    if k==6
        final2(2,k) = mean(countd6);
    end 
    if k==7
        final2(2,k) = mean(countd7);
    end 
    if k==8
        final2(2,k) = mean(countd8);
    end 
    if k==9
        final2(2,k) = mean(countd9);
    end
    if k==10
        final2(2,k) = mean(countd10);
    end 
    if k==11
        final2(2,k) = mean(countd11);
    end 
    if k==12
        final2(2,k) = mean(countd12);
    end 
end

for k=1:12
    if k==1
        final2(3,k) = std(countd1);
    end  
    if k==2
        final2(3,k) = std(countd2);
    end 
    if k==3
        final2(3,k) = std(countd3);
    end 
    if k==4
        final2(3,k) = std(countd4);
    end 
    if k==5
        final2(3,k) = std(countd5);
    end 
    if k==6
        final2(3,k) = std(countd6);
    end 
    if k==7
        final2(3,k) = std(countd7);
    end 
    if k==8
        final2(3,k) = std(countd8);
    end 
    if k==9
        final2(3,k) = std(countd9);
    end
    if k==10
        final2(3,k) = std(countd10);
    end 
    if k==11
        final2(3,k) = std(countd11);
    end 
    if k==12
        final2(3,k) = std(countd12);
    end 
end

for k=1:12
    if k==1
        final2(4,k) = min(countd1);
    end  
    if k==2
        final2(4,k) = min(countd2);
    end 
    if k==3
        final2(4,k) = min(countd3);
    end 
    if k==4
        final2(4,k) = min(countd4);
    end 
    if k==5
        final2(4,k) = min(countd5);
    end 
    if k==6
        final2(4,k) = min(countd6);
    end 
    if k==7
        final2(4,k) = min(countd7);
    end 
    if k==8
        final2(4,k) = min(countd8);
    end 
    if k==9
        final2(4,k) = min(countd9);
    end
    if k==10
        final2(4,k) = min(countd10);
    end 
    if k==11
        final2(4,k) = min(countd11);
    end 
    if k==12
        final2(4,k) = min(countd12);
    end 
end

for k=1:12
    if k==1
        final2(5,k) = max(countd1);
    end  
    if k==2
        final2(5,k) = max(countd2);
    end 
    if k==3
        final2(5,k) = max(countd3);
    end 
    if k==4
        final2(5,k) = max(countd4);
    end 
    if k==5
        final2(5,k) = max(countd5);
    end 
    if k==6
        final2(5,k) = max(countd6);
    end 
    if k==7
        final2(5,k) = max(countd7);
    end 
    if k==8
        final2(5,k) = max(countd8);
    end 
    if k==9
        final2(5,k) = max(countd9);
    end
    if k==10
        final2(5,k) = max(countd10);
    end 
    if k==11
        final2(5,k) = max(countd11);
    end 
    if k==12
        final2(5,k) = max(countd12);
    end 
end

len=1:1:length(final);
figure(5);
plot(len,final); 
xlabel('frame');
ylabel('error');
profile viewer;







