import web
import time
import timeit
from collections import deque
from sys import exit, exc_info, argv
import math
import os
localport = os.environ.get('PORT', 8080)

from cStringIO import StringIO

clock = timeit.default_timer;

web.config.debug = False;
urls = (
		'/u','controller',
		'/u2','controller2',
        '/error','controller3',
		'/stop','closecontroller'
		)
app = web.application(urls, globals())

def sign(val):
        return 1 if val >= 0 else -1

class closecontroller:
    def GET(self):
        return exit(0)
    def POST(self):
        return exit(0)

class controller:
    def GET(self):
        return self.process();
    def POST(self):
        return self.process();
    def process(self):
        i = web.input();#print i
        f = "-1929291"
		
        try:
			if hasattr(i, 'data'):
				PC = plantcontroller(i.data.split())
				#time.sleep(5)
				f = PC.evaluate()[0];
        except:
			print exc_info(), i
		
        web.header("Content-Type", "text/plain") # Set the Header
        return str(f)

class controller2:
    def GET(self):
        return self.process();
    def POST(self):
        return self.process();
    def process(self):
        i = web.input();#print i
        f = "-1929291"
		
        try:
			if hasattr(i, 'data'):
				PC = plantcontroller(i.data.split())
				#time.sleep(5)
				f = PC.evaluate();
        except:
			print exc_info(), i
		
        web.header("Content-Type", "text/plain") # Set the Header
        return str(PC.file_str.getvalue())

class controller3:
    def GET(self):
        return self.process();
    def POST(self):
        return self.process();
    def process(self):
        i = web.input();#print i
        f = "-1929291"
        
        try:
            if hasattr(i, 'data'):
                PC = plantcontroller(i.data.split())
                #time.sleep(5)
                f = PC.evaluate()[1];
        except:
            print exc_info(), i
        
        web.header("Content-Type", "text/plain") # Set the Header
        return str(f)


#set up the best clock that can be accessed on this machine
clock = timeit.default_timer;
#get the current time (time the controller was started).
time0 = clock();


class plantcontroller:
	def __init__(self, data):
		try:
			self.duration= float(data[0])
			self.h= float(data[1]);
			KpR= float(data[2]);
			KiR= float(data[3]);
			KdR= float(data[4]);
			KpM= float(data[5]);
			KiM= float(data[6]);
			KdM= float(data[7]);
		except:
			print exc_info()
			self.duration= 0;
			self.h= .02;
			(KpM,KiM,KdM,KpR,KiR,KdR)=(0,0,0,0,0,0)

		KiM,KiR=(0,0)
		self.R_ref = .5
		self.w=2*3.14159*1/100.*1;

		self.CumulativeError = 0.
		self.Count = 0.

		self.R =	deque([ (0,0), (0,0), (0,0)],10);
		self.Theta = deque([ (0,0), (0,0), (0,0)],10);
		self.StateTime = 0;

		self.angle_max = 3.14/180.0*(32+20)
		'''
		#------------------------------------------
		#PID controller _0 for ball!
		# http://www.forkosh.com/mimetex.cgi?P(s)=\frac{Y(s)}{X(s)}=\frac{a_2s^2+a_1 s+a_0}{b_2s^2+b_1s+b_0}
		a2_0 = KdR;
		a1_0 = KpR;
		a0_0 = KiR;

		b2_0 =  0;
		b1_0 =  1;
		b0_0 =  0;
		#------------------------------------------

		A2_0 =   a2_0 + a1_0 * self.h +  a0_0 * self.h * self.h;
		A1_0 =-2*a2_0 - a1_0 * self.h;
		A0_0 =   a2_0;

		B2_0 =   b2_0 + b1_0 * self.h +  b0_0 * self.h * self.h;
		B1_0 =-2*b2_0 - b1_0 * self.h;
		B0_0 =   b2_0;

		self.eta0_0 = -B0_0/B2_0;
		self.eta1_0 = -B1_0/B2_0;
		self.gamma0_0 = A0_0/B2_0;
		self.gamma1_0 = A1_0/B2_0;
		self.gamma2_0 = A2_0/B2_0;

		self.Y0 =	deque([ (0,0), (0,0), (0,0)],3);
		self.X0 =	deque([ (0,0), (0,0), (0,0)],3);

		#------------------------------------------
		#PID controller _1 for beam!
		# http://www.forkosh.com/mimetex.cgi?P(s)=\frac{Y(s)}{X(s)}=\frac{a_2s^2+a_1 s+a_0}{b_2s^2+b_1s+b_0}
		a2_1 = KdM;
		a1_1 = KpM;
		a0_1 = KiM;

		b2_1 =  0;
		b1_1 =  1;
		b0_1 =  0;
		#------------------------------------------

		A2_1 =   a2_1 + a1_1 * self.h +  a0_1 * self.h * self.h;
		A1_1 =-2*a2_1 - a1_1 * self.h;
		A0_1 =   a2_1;

		B2_1 =   b2_1 + b1_1 * self.h +  b0_1 * self.h * self.h;
		B1_1 =-2*b2_1 - b1_1 * self.h;
		B0_1 =   b2_1;

		self.eta0_1 = -B0_1/B2_1;
		self.eta1_1 = -B1_1/B2_1;
		self.gamma0_1 = A0_1/B2_1;
		self.gamma1_1 = A1_1/B2_1;
		self.gamma2_1 = A2_1/B2_1;

		self.X1 =	deque([ (0,0), (0,0), (0,0)],3);
		self.Y1 =	deque([ (0,0), (0,0), (0,0)],3);				    
		'''

		self.AR= KpR;
		self.BR= KdR/self.h;
                self.Y0 =       deque([ (0,0), (0,0), (0,0)],3);
                self.X0 =       deque([ (0,0), (0,0), (0,0)],3);

		self.AM= KpM;
		self.BM= KdM/self.h;
                self.X1 =       deque([ (0,0), (0,0), (0,0)],3);
                self.Y1 =       deque([ (0,0), (0,0), (0,0)],3);

		m = 0.111;
		R = 0.015;
		g = -9.8;
		L = 1.0;
		d = 0.03;
		J = 9.99e-6;
		H = -m*g*d/L/(J/R*R+m);

		#http://www.forkosh.com/mimetex.cgi?P(s)=\frac{-m*g*d/L/(J/R^2+m)}{s^2}
		#http://www.forkosh.com/mimetex.cgi?s=\frac{z-1}{zh}
		#http://www.forkosh.com/mimetex.cgi?r[n]=2r[n-1]-r[n-2]+Hh^2\theta[n]

		self.Dist =	deque([ (0,0), (0,0), (0,0)],10);
		self.Theta_plant = deque([ (0,0), (0,0), (0,0)],10);
		self.U =	deque([ (0,0), (0,0), (0,0)]);
		self.h_plant =	self.h/10;
		self.Count_plant = 0;

		#http://www.forkosh.com/mimetex.cgi?P(s)=\frac{\Theta(z)}{V_{in}(z)}=\frac{A_2^2z^2}{B_2^2z^2 + B_1z + B_0}

		alpha=0.01176
		beta=0.58823
		#http://www.forkosh.com/mimetex.cgi?P(s)=\frac{\Theta(s)}{V_{in}(s)}=\frac{1}{s(\alpha s+\beta)}
		A12=self.h_plant*self.h_plant
		B12=alpha
		B11=(beta*self.h_plant- 2*alpha)
		B10=alpha
		self.P=A12/B12
		self.Q=B11/B12
		self.R=B10/B12
		self.theta_high = 3.14/180.0*(32+20);
		self.r_high = 1.1;


		#2.2.6
		#http://www.forkosh.com/mimetex.cgi?P(s)=\frac{X(s)}{A(s)}=\frac{-7}{s^2}

		A22=-7*self.h_plant*self.h_plant
		B22=1
		B21=-2
		B20=1
		self.L=A22/B22
		self.M=B21/B22
		self.N=B20/B22

		self.file_str = StringIO()

	#in the future could base the time on the plant, and pass it as a parameter to this method
	def referencex(self,t):
		return self.R_ref*sign(math.cos(self.w*t));

	def referencey(self,t):
		return self.R_ref*sign(math.sin(self.w*t));

	def updatePID2(self):
		#global X0, X1, Y0, Y1, Theta, StateTime, CumulativeError, Count

		# Update the time and iteration number
		self.Count += 1
		t = self.Count*self.h;     
		
	        try:
        	        self.X0[-3]=self.X0[-2];self.X0[-2]=self.X0[-1];self.X0[-1]=(self.Dist[-1][0], self.Dist[-1][1])
                	self.X1[-3]=self.X1[-2];self.X1[-2]=self.X1[-1];self.X1[-1]=self.Theta_plant[-1]
        	        StateTime= self.Count_plant*self.h_plant;
	        except:
                	print exc_info(), "error"
                	self.CumulativeError = self.CumulativeError +  10 #/(duration/(h*h))

		'''
			Determine the desired beam angle based on the ball position
		'''
		x_d = self.referencex(t);#ref(t1,xkernel ,xamplitude ,xfrequency)
		e_x = x_d - self.X0[-1][0];
		angle_d = self.AR * (e_x) + self.BR * (self.X0[-1][0]-self.X0[-2][0]);

		if angle_d > self.angle_max: angle_d=self.angle_max; 
		elif angle_d < -self.angle_max: angle_d=-self.angle_max; 
		u_x = self.AM*(angle_d*16 - self.X1[-1][0]) + self.BM * (self.X1[-1][0]-self.X1[-2][0])

		y_d = self.referencey(t);#ref(t1,ykernel,yamplitude,yfrequency)
		e_y = y_d - self.X0[-1][1];
		angle_d1 = self.AR * (e_y) + self.BR * (self.X0[-1][1]-self.X0[-2][1]);

		if angle_d1 > self.angle_max: angle_d1=self.angle_max; 
		elif angle_d1 < -self.angle_max: angle_d1=-self.angle_max; 
		u_y = self.AM*(angle_d1*16 - self.X1[-1][1]) + self.BM * (self.X1[-1][1]-self.X1[-2][1])
		self.Y1[-3]=self.Y1[-2];self.Y1[-2]=self.Y1[-1];self.Y1[-1]=(u_x,u_y,);

		self.file_str.write("%s %s %s 0\n"%("p",self.Dist[-1][0], self.Dist[-1][1]))
        
		self.CumulativeError = self.CumulativeError + abs(e_x) #/(duration/h)
		self.CumulativeError = self.CumulativeError + abs(e_y) #/(duration/h)
		

	def updatePID(self):
		#global X0, X1, Y0, Y1, Theta, StateTime, CumulativeError, Count
		self.Count += 1
		t = self.Count*self.h;     
		
		'''
			Determine the desired beam angle based on the ball position
			'''
		
		self.CumulativeError = self.CumulativeError + abs(self.X0[-1][0]) #/(duration/h)
		self.CumulativeError = self.CumulativeError + abs(self.X0[-1][1]) #/(duration/h)
		
		angle0 = self.eta1_0 * self.Y0[-1][0] + self.eta0_0 * self.Y0[-2][0] + self.gamma2_0 * self.X0[-1][0] + self.gamma1_0 * self.X0[-2][0] + self.gamma0_0 * self.X0[-3][0];
		angle1 = self.eta1_0 * self.Y0[-1][1] + self.eta0_0 * self.Y0[-2][1] + self.gamma2_0 * self.X0[-1][1] + self.gamma1_0 * self.X0[-2][1] + self.gamma0_0 * self.X0[-3][1];
		
		#probably should get the old values of Y0 from X1.... Right now I'm remembering the requested values 
		self.Y0[-3]=self.Y0[-2];self.Y0[-2]=self.Y0[-1];self.Y0[-1]=(angle0, angle1);
		
		if angle0 > self.angle_max: angle0=self.angle_max;
		elif angle0 < -self.angle_max: angle0=-self.angle_max;
		if angle1 > self.angle_max: angle1=self.angle_max;
		elif angle1 < -self.angle_max: angle1=-self.angle_max;
		
		for i in range(3):
			self.X1[i]=(self.X1[i][0]-angle0*16,self.X1[i][1]-angle1*16);
		
		'''
			Determine the desired control value based on the current (and the desired) beam angle
			'''
		
		u0u2 = self.eta1_1 * self.Y1[-1][0] + self.eta0_1 * self.Y1[-2][0] + self.gamma2_1 * self.X1[-1][0] + self.gamma1_1 * self.X1[-2][0] + self.gamma0_1 * self.X1[-3][0];
		u1u2 = self.eta1_1 * self.Y1[-1][1] + self.eta0_1 * self.Y1[-2][1] + self.gamma2_1 * self.X1[-1][1] + self.gamma1_1 * self.X1[-2][1] + self.gamma0_1 * self.X1[-3][1];
		self.Y1[-3]=self.Y1[-2];self.Y1[-2]=self.Y1[-1];self.Y1[-1]=(-u0u2,-u1u2);
		
		try:
			self.X0[-3]=self.X0[-2];self.X0[-2]=self.X0[-1];self.X0[-1]=(self.Dist[-1][0]-self.referencex(t), self.Dist[-1][1]-self.referencey(t))
			self.X1[-3]=self.X1[-2];self.X1[-2]=self.X1[-1];self.X1[-1]=self.Theta_plant[-1]
			StateTime= self.Count_plant*self.h_plant;
		except:
			print exc_info(), "error"
			self.CumulativeError = self.CumulativeError +  10 #/(duration/(h*h))

	def updatePlant(self):
		#global Theta_plant, Dist, U, Count_plant
		self.Count_plant += 1;
		self.U.append(self.Y1[-1]);
		theta0 =  self.P * self.U[-1][0] - self.Q * self.Theta_plant[-1][0] - self.R * self.Theta_plant[-2][0]
		if theta0 > self.theta_high: theta0 = self.theta_high
		elif theta0 < -self.theta_high: theta0 = -self.theta_high
		
		theta1 =  self.P * self.U[-1][1] - self.Q * self.Theta_plant[-1][1] - self.R * self.Theta_plant[-2][1]
		if theta1 > self.theta_high: theta1 = self.theta_high
		elif theta1 < -self.theta_high: theta1 = -self.theta_high
		
		self.Theta_plant.append((theta0,theta1));
		x0 =  self.L * self.Theta_plant[-1][0]/16.0 - self.M * self.Dist[-1][0] - self.N * self.Dist[-2][0]; #alpha = theta/16 eqn 2.2.2 EEE490
		
		if x0 > self.r_high: x0 = self.r_high;
		elif x0 < -self.r_high: x0 = -self.r_high;
		
		x1 =  self.L * self.Theta_plant[-1][1]/16.0 - self.M * self.Dist[-1][1] - self.N * self.Dist[-2][1]; #alpha = theta/16 eqn 2.2.2 EEE490
		
		if x1 > self.r_high: x1 = self.r_high;
		elif x1 < -self.r_high: x1 = -self.r_high;
		
		self.Dist.append((x0,x1));
	#print str(repr(Count_plant*h_plant)) + ","+ str(Dist[-1][0]) +","+ str(Dist[-1][1]) +","+ str(Theta[-1][0]) +","+ str(Theta[-1][1]) +","+ str(U[-1][0]) +","+ str(U[-1][1]) +","+ str(repr(Count*h))+ ","+ str(repr((Count_plant-1)*h_plant))+",sekou"

	def evaluate(self):
		value = -99;
		try:
			if self.h < self.duration/3.0 and self.h > 0.001:
				for i in range(int(self.duration/0.001)):
					if i%int(self.h/.001) == 0:
						self.updatePID2();
					if i%int(self.h_plant/.001) == 0:
						self.updatePlant();
				if 0.0000000000000001 != self.CumulativeError:
					value= 1/pow(self.CumulativeError/self.Count,5.0);
		except:
			print exc_info()
			print "syntax is duration STEPSIZE KPball KIball KDball KPbeam KIbeam KDbeam"
			
		return [value,self.CumulativeError/self.Count]


if __name__ == "__main__":
	wsgifunc = app.wsgifunc()
	wsgifunc = web.httpserver.StaticMiddleware(wsgifunc)
	server = web.httpserver.WSGIServer(("0.0.0.0", int(localport)),wsgifunc)
	print "http://%s:%s/" % ("0.0.0.0", localport)
	try:
		server.start()
	except (KeyboardInterrupt, SystemExit):
		server.stop()
		print "Shutting down service"
