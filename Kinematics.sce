function[x1,y1,x2,y2,x3,y3,gam,xe,ye,xp,yp] = JointPosition2EffectorPose(theta1,theta2,theta3)


gam = theta1+theta2+theta3

x1 = a12*cos(theta1)

y1 = a12*sin(theta1)

x2 = a12*cos(theta1)+a23*cos(theta1+theta2)

y2 = a12*sin(theta1)+a23*sin(theta1+theta2)

x3 = x2+a3Tx1*cos(gam)

y3 = y2+a3Tx1*sin(gam)

xe = x3 + a3Ty1*sin(gam)
ye = y3 - a3Ty1*cos(gam)

xp = xe + a3Tx2*cos(gam)
yp = ye + a3Tx2*sin(gam)


endfunction





function[theta1,theta2,theta3] = EffectorPose2JointPosition(x,y,gam)


//First we change the position of the end effector to the axis of the joint 3.

x = x-a3Ty1*sin(gam)
y = y+a3Ty1*cos(gam)


//Obtain theta2

d = 2*a12*a23
f = (x-(a3Tx1+a3Tx2)*cos(gam))^2 + (y-(a3Tx1+a3Tx2)*sin(gam))^2-a12^2-a23^2
theta2 = acos(f/d)

//Obtain theta1

A = a12+a23*cos(theta2)
B = a23*sin(theta2)
E = x-(a3Tx1+a3Tx2)*cos(gam)
F = y-(a3Tx1+a3Tx2)*sin(gam)

s1 = (A*F-E*B)/(A^2+B^2)
c1 = (F*B+E*A)/(A^2+B^2)

theta1 = atan(s1,c1)

//obtain theta 3


theta3 = gam - theta2 - theta1

endfunction



function[J] = Jacobiano(theta1,theta2,theta3)


a12 = 0.62
a23 = 0.57
ax34 = 0.4
ay34 = -0.2


gam = theta1+theta2+theta3

x1 = 0
y1 = 0

x2 = a12*cos(theta1)
y2 = a12*sin(theta1)

x3 = x2+a23*cos(theta1+theta2)
y3 = y2+a23*sin(theta1+theta2)


J = [y1 y2 y3;-x1 -x2 -x3;1 1 1]

endfunction


function[w1,w2,w3] = EffectorVelocity2JointVelocity(J)

R = inv(J)*Twist'

w1 = R(1)
w2 = R(2)
w3 = R(3)

endfunction



function[w1,w2,w3,Tw] = Track()


w1 = 0
w2 = 0
w3 = 0
n = 1
time(1) = sample_time
X = X0
Y = Y0

while(X-XT > 0.00001 || Y-YT > 0.00001)

    
    
    [theta1(n),theta2(n),theta3(n)] = EffectorPose2JointPosition(X,Y,Gamm)

    //All the position of the joints are computed to draw the robot
    [x1(n),y1(n),x2(n),y2(n),x3(n),y3(n),gam(n),xe(n),ye(n),xp(n),yp(n)] = JointPosition2EffectorPose(theta1(n),theta2(n),theta3(n))

    J = Jacobiano(theta1(n),theta2(n),theta3(n))
    [w1(n),w2(n),w3(n)] = EffectorVelocity2JointVelocity(J)

    //Updating the Tool Position
    X = X + V0x * sample_time
    Y = Y + V0y * sample_time 

    //Updating the iteration variables
    n = n + 1
    time(n) = time(n-1)+sample_time
    

    

end



//PLOTS

m = 1

//Drawing the Angular Velocities
figure('backgroundColor',[1 1 1])
xname("Angular Velocities")
plot(time(1:n-1),w1)
plot(time(1:n-1),w2,"r")
plot(time(1:n-1),w3,"g")


figure('BackgroundColor',[1 1 1])
xname("Robot Simulation")
plot2d(0+0.05,0-0.05,-1,"031"," ",[-0.4,-0.7,1.1,0.1])
//joint 1
xfarc(0+0.1,0-0.1,0.1,0.1,0,360*64)
//Joint 2
xfarc(x1(m)+0.1,y1(m)-0.1,0.1,0.1,0,360*64)
//Joint 3
xfarc(x2(m)-0.1,y2(m)+0.1,0.1,0.1,0,360*64)
//Tool
xfarc(x3(m)-0.1,y3(m)+0.1,0.1,0.1,0,360*64)
a = gca()
m = m + 1

while(m<n)
    
    delete(a.children)
    //joint 1
    xfarc(0-0.05,0+0.05,0.1,0.1,0,360*64)
    //Joint 2
    xfarc(x1(m)-0.05,y1(m)+0.05,0.1,0.1,0,360*64)
    plot([0 x1(m)],[0 y1(m)])
    //Joint 3
    xfarc(x2(m)-0.05,y2(m)+0.05,0.1,0.1,0,360*64)
    plot([x1(m) x2(m)],[y1(m) y2(m)])
    //Tool
    xfarc(xp(m)-0.05,yp(m)+0.05,0.1,0.1,0,360*64)
    plot([x2(m) x3(m)],[y2(m) y3(m)])
    plot([x3(m) xe(m)],[y3(m) ye(m)])
    plot([xe(m) xp(m)],[ye(m) yp(m)])
    m = m + 1
    //disp(theta2(m))
    sleep(200)

end



endfunction

//PROGRAMA
exec("Configuration.sce")
Track()

