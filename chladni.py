# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""
import serial
import threading
import Queue
import string
import numpy
import time
from matplotlib import pyplot as p

class Vibrating_Plate:
    def __init__(self):
        self.radial = 0
        self.angular = 0
        self.handle = False
        for i in range(10):
            try:
                device = "/dev/ttyACM%d" % (i)
                print device
                self.handle = serial.Serial(device,115200)
            except:
                print "except"
                continue
            break;
        if not self.handle:
            print "Oh nooes"
        print "Got device %s" % (device)
        self.queue = Queue.Queue()
        t = threading.Thread(target=self.reader)
        t.daemon = True
        t.start()
        print "thread started"
        while False == self.timeout_for("reset",timeout=1):
            print "resetting"
            self.handle.write("reset\n");
    def reader(self):
        print "reader starts"
        while True:
            str = self.handle.readline()
            self.queue.put(str)
    def timeout_for(self,str,timeout):
        while True:
            try:
                resp = self.queue.get(block=True,timeout=timeout)
                print "after get"
                self.queue.task_done()
                rc = string.find(resp,str)
                if 0 == rc : return resp
                print "Unmatched: %s" % (resp)
            except Queue.Empty:
                print "exception"
                return False

    def wait_for(self,str,timeout=None):
        while True:
            resp = self.queue.get()
            self.queue.task_done()
            rc = string.find(resp,str)
            if 0 == rc : return resp
            print "Unmatched: %s" % (resp)
    def noise(self):
        self.handle.write("noise\n")
        self.wait_for("noise")
    def silent(self):
        self.handle.write("silent\n")
        self.wait_for("silent")
    def sine(self,freq):
        command = "sine %f\n" % (freq)
        self.handle.write(command)
        resp = self.wait_for("sine")
        return string.atof(string.replace(resp,"sine ",""))
    def get(self):
        self.handle.write("get\n")
        resp = self.wait_for("get")
        words = resp.split(" ")
        return float(words[1]),float(words[2]),float(words[3])
    def angular_set(self,dt):
        command = "a_set %f\n" % (dt)
        self.handle.write(command);
        self.wait_for("a_set")
    def radial_set(self,dt):
        command = "r_set %f\n" % (dt)
        self.handle.write(command);
        self.wait_for("r_set")
    def angular_go(self,steps):
        #Protect wire by going opposite direction if has already gone two full circles past
        #calibrated zero point
        if self.angular * numpy.sign(steps) > 1440:
            steps = -numpy.sign(steps) * (720 - steps) 
        command = "a_go %d\n" % (steps)
        self.handle.write(command);
        self.wait_for("a_go")
        self.angular += steps
        return True
    def radial_go(self,steps):
        command = "r_go %d\n" % (steps)
        self.handle.write(command);
        self.wait_for("r_go")
        self.radial += steps
        return True
    def angular_idle(self):
        self.handle.write("a_idle\n")
        resp = self.wait_for("a_idle")
        return "true" == resp[7:11]
    def radial_idle(self):
        self.handle.write("r_idle\n")
        resp = self.wait_for("r_idle")
        return "true" == resp[7:11]
    def go_and_wait(self,angular,radial):
        while not (self.angular_idle() and self.radial_idle()): True
        if (angular != 0): self.angular_go(angular)
        if (radial != 0): self.radial_go(radial)
        while not (self.angular_idle() and self.radial_idle()): True
        return True
    def get_samples(self):
        self.handle.write("get_samples\n")
        resp = self.wait_for("get_samples")
        n = int(resp[11:])
        values = numpy.zeros(n)
        for i in range(n):
            values[i] = int(self.queue.get())
            self.queue.task_done()
        return values
    def angular_record(self,steps):
        self.handle.write("m_sample\n");
        self.wait_for("m_sample")
        self.angular_go(steps)
        while not self.angular_idle(): time.sleep(1)
        values = self.get_samples()
        if abs(steps) != numpy.size(values,axis=0):
            print "Odd, samples count (%d) is different from steps taken (%d)" % (numpy.size(values,axis=0),abs(steps))
        return values
    def radial_record(self,steps):
        self.handle.write("m_sample\n");
        self.wait_for("m_sample")
        self.radial_go(steps)
        while not self.radial_idle(): time.sleep(1)
        values = self.get_samples()
        if abs(steps) != numpy.size(values,axis=0):
            print "Odd, samples count (%d) is different from steps taken (%d)" % (numpy.size(values,axis=0),abs(steps))
        return values
    def raw_record(self):
        self.handle.write("t_sample\n");
        self.wait_for("t_sample")
        time.sleep(1)
        return self.get_samples()
    def calibrate(self):
        self.radial = 0
        self.angular = 0

def demo_motors(c):
    npts = 7
    max_r = 7000/numpy.sqrt(2)
    x = numpy.linspace(-max_r,max_r,npts)
    y = numpy.linspace(-max_r,max_r,npts)
    last_r = 0
    last_a = 0
    c.radial_set(1.25e-3) # speed up motor a bit from default 2 ms
    c.angular_set(4.5e-3) # speed up motor a bit from default 5 ms
    points = []
    for i in range(npts):     # generate angluar and radial steps from rect coords
        for j in range(npts):
            r = numpy.sqrt(x[i]**2+y[j]**2)
            a = numpy.arctan2(y[j],x[i])*360/numpy.pi
            points.append((a,r))
    points = sorted(points,key=lambda p: p[0]) # sort by zeroth (angle), commment this line to see worse performance
    for pt in points:
        a = pt[0]
        r = pt[1]
        c.go_and_wait(a-last_a,r - last_r)
        last_a = a
        last_r = r
    c.go_and_wait(-last_a,-last_r) # return to "origin"

def demo_spectra(c):
    c.noise()
    time.sleep(2)
    y = c.raw_record()
    c.silent()
    mag = numpy.absolute(numpy.fft.rfft(y))
    mag[0] = 0 # get rid of that massive DC offset
    f = numpy.linspace(0,24e3,numpy.size(mag)) # 24e3 = 1/2 f_sample = 48kHz
    p.plot(f,mag)

def demo_freqscan(c):
    npts = 100
    dc = numpy.zeros(npts)
    amp = numpy.zeros(npts)
    phase = numpy.zeros(npts)
    f = numpy.linspace(90,110,npts)
    for i in range(npts):
        f[i] = c.sine(f[i])
        time.sleep(1)
        dc[i],amp[i],phase[i] = c.get()
        print "f=%f, dc=%f, amp=%f, phase=%f" % (f[i],dc[i],amp[i],phase[i])
    p.figure()
    p.plot(f,amp)
    p.figure()
    p.plot(f,phase)

def demo_m_sample(c):
    c.radial_set(1.1e-3)
    y = c.radial_record(7000)
    p.figure()
    p.plot(y)
    c.go_and_wait(0,-7000)

#Go to a position on the plate where r is the radius, in steps
#and theta is the angle in degrees.
def go_to_polar(c, r=None, theta=None):
    if r = None:
        r = c.radial
    if theta = None:
        theta = c.angular
    radial_steps = r - c.radial
    angular_steps =numpy.rint(theta * 2 - c.angular)
    c.angular_go(angular_steps)
    c.radial_go(radial_steps)

#Go to x,y position where x,y are in steps, plate is ~14000 in width
def go_to_cart(c, x, y):
    r = numpy.rint(numpy.sqrt(x**2 + y**2))
    theta = numpy.degrees(numpy.arctan2(y,x))
    if r > 7000:
        go_to_polar(c,0,0)
    go_to_polar(c,r,theta)

plate = Vibrating_Plate()

