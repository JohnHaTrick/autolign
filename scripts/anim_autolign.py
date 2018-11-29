#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot    as plt
import matplotlib.animation as animation
import plot_autolign         as plot
pi = np.pi

def drawCog(x,y,psi,radius):
    res = 6 # resolution of polygon
    artists = []
    alpha = np.linspace(0, 90, res)/180.0*pi + psi 
    cosval = np.insert(np.append(radius*np.cos(alpha),0),0,0) # append 0's at 1st & end to np.ndarray
    sinval = np.insert(np.append(radius*np.sin(alpha),0),0,0) # append 0's at 1st & end to np.ndarray
    artists.extend(plt.fill(x*np.ones(res+2) + cosval, y*np.ones(res+2) + sinval, 'k'))
    alpha = np.linspace(90, 180, res)/180.0*pi + psi 
    artists.extend(plt.plot(x*np.ones(res)+radius*np.cos(alpha), y*np.ones(res)+radius*np.sin(alpha),'k'))
    alpha = np.linspace(180, 270, res)/180.0*pi + psi 
    cosval = np.insert(np.append(radius*np.cos(alpha),0),0,0)
    sinval = np.insert(np.append(radius*np.sin(alpha),0),0,0)
    artists.extend(plt.fill(x*np.ones(res+2)+cosval, y*np.ones(res+2)+ sinval, 'k'))
    alpha = np.linspace(270, 360, res)/180.0*pi + psi 
    artists.extend(plt.plot(x*np.ones(res)+radius*np.cos(alpha), y*np.ones(res)+radius*np.sin(alpha),'k'))
    return artists

def trans2d(Zin, Zoom, Move, theta):
    # Zin = [xin; yin]
    # Zoom = [zoomX; zoomY]
    # Move = [moveX; moveY]
    # xin, yin : row vector
    zoomX = Zoom[0]
    zoomY = Zoom[1]
    moveX = Move[0]
    moveY = Move[1]
    ONE = np.ones([1,Zin.shape[1]])
    Zin = np.r_[Zin, ONE]
    #print theta
    Rot = np.array(
        [[zoomX*np.cos(theta), zoomY*(-np.sin(theta)), 0],
        [zoomX*np.sin(theta), zoomY*np.cos(theta), 0],
        [0, 0, 1]]) # rotation matrix
    
    Z1 = np.matmul(Rot,Zin)
    Move = np.array([[1, 0, moveX],
            [0, 1, moveY],
            [0, 0, 1]])
    Zout = np.matmul(Move,Z1)

    Zout = Zout[:2,:]
    return Zout


def plotVehicle(east, north, psi, delta1, delta2, delta3, delta4):
    
    # Param from VehicleParam
    #-------------------------------------------------------
    a = 1.50              # Front length          [m]
    b = 1.37              # Rear length           [m]
    d = 0.82              # Half Tread            [m]
    tw = 0.15             # Tire half width       [m]
    tl = 0.4              # Tire half length      [m]
    FORCE_ZOOM_X = 1.0e-3   # Long. force indicator [-]
    FORCE_ZOOM_Y = 1.0e-3   # Lat. force indicator  [-]
    #-------------------------------------------------------
    # plot vehicle chassis
    # tire matrix
    tire = np.array([[tl, tl, -tl, -tl, tl], [tw, -tw, -tw, tw, tw]]) 
        
    psi = psi + pi/2
    tread_f = d
    tread_r = d 
    fm = 1.5*tw  # force margin to plot
    #tread_r2 = tw+0.1 
    
    # cog
    gpos = [east, north]
    fpos = trans2d(np.array([[a],[0]]),[1,1], gpos, psi)
    flpos = trans2d(np.array([[a],[tread_f]]),[1,1], gpos, psi)
    frpos = trans2d(np.array([[a],[-tread_f]]),[1,1], gpos, psi)
    f1pos = trans2d(np.array([[a],[tread_f + fm]]),[1,1], gpos, psi)
    f2pos = trans2d(np.array([[a],[-(tread_f + fm)]]),[1,1], gpos, psi)
    
    rpos = trans2d(np.array([[-b],[0]]),[1,1], gpos, psi)
    rlpos = trans2d(np.array([[-b],[tread_r]]),[1,1], gpos, psi)
    rrpos = trans2d(np.array([[-b],[-tread_r]]),[1,1], gpos, psi)
    f3pos = trans2d(np.array([[-b],[tread_r + fm]]),[1,1], gpos, psi)
    f4pos = trans2d(np.array([[-b],[-(tread_r + fm)]]),[1,1], gpos, psi)
    
    tire1 = trans2d(tire,[1,1],flpos,psi+delta1)
    tire2 = trans2d(tire,[1,1],frpos,psi+delta2)
    tire3 = trans2d(tire,[1,1],rlpos,psi+delta3)
    tire4 = trans2d(tire,[1,1],rrpos,psi+delta4)
    
    # chassis center
    artists = []
    artists.extend(plt.plot([gpos[0],fpos[0]],[gpos[1],fpos[1]],'k',linewidth=1))
    artists.extend(plt.plot([gpos[0],rpos[0]],[gpos[1],rpos[1]],'k',linewidth=1))
    
    
    # chassis front
    artists.extend(plt.plot([flpos[0],frpos[0]],[flpos[1],frpos[1]],'k',linewidth=1))
    artists.extend(plt.plot([flpos[0],frpos[0]],[flpos[1],frpos[1]],'k',linewidth=1))
    
    # chassis rear
    artists.extend(plt.plot([rlpos[0],rrpos[0]],[rlpos[1],rrpos[1]],'k',linewidth=1))
    artists.extend(plt.plot([rlpos[0],rrpos[0]],[rlpos[1],rrpos[1]],'k',linewidth=1))
    
    # Front tire
    
    artists.extend(plt.plot(tire1[0,:],tire1[1,:],'k',linewidth=1))
    artists.extend(plt.fill(tire1[0,:],tire1[1,:],color=[.4,.4,.4]))
    artists.extend(plt.plot(tire2[0,:],tire2[1,:],'k',linewidth=1))
    artists.extend(plt.fill(tire2[0,:],tire2[1,:],color=[.4,.4,.4]))
    
    # Rear Tire
    artists.extend(plt.plot(tire3[0,:],tire3[1,:],'k',linewidth=1))
    artists.extend(plt.fill(tire3[0,:],tire3[1,:],color=[.4,.4,.4]))
    artists.extend(plt.plot(tire4[0,:],tire4[1,:],'k',linewidth=1))
    artists.extend(plt.fill(tire4[0,:],tire4[1,:],color=[.4,.4,.4]))
                       
    # draw_cog
    artists.extend(drawCog(east,north,psi+pi/2,0.2))

    return artists

def plotVehicleWithForce(east, north, psi, delta1, delta2, delta3, delta4, fx1, fx2, fx3, fx4, fy1, fy2, fy3, fy4):
    
    # Param from VehicleParam
    #-------------------------------------------------------
    a = 1.50              # Front length          [m]
    b = 1.37              # Rear length           [m]
    d = 0.82              # Half Tread            [m]
    tw = 0.15             # Tire half width       [m]
    tl = 0.4              # Tire half length      [m]
    FORCE_ZOOM_X = 1.0e-3   # Long. force indicator [-]
    FORCE_ZOOM_Y = 1.0e-3   # Lat. force indicator  [-]
    #-------------------------------------------------------
    # plot vehicle chassis
    # tire matrix
    tire = np.array([[tl, tl, -tl, -tl, tl], [tw, -tw, -tw, tw, tw]]) 
    
    f1 = np.array([[0, fx1, 0, 0], [0, 0, 0, fy1]])
    f2 = np.array([[0, fx2, 0, 0], [0, 0, 0, fy2]])
    f3 = np.array([[0, fx3, 0, 0], [0, 0, 0, fy3]])
    f4 = np.array([[0, fx4, 0, 0], [0, 0, 0, fy4]])
    
    psi = psi + pi/2
    tread_f = d
    tread_r = d 
    fm = 1.5*tw  # force margin to plot
    #tread_r2 = tw+0.1 
    
    # cog
    gpos = [east, north]
    fpos = trans2d(np.array([[a],[0]]),[1,1], gpos, psi)
    flpos = trans2d(np.array([[a],[tread_f]]),[1,1], gpos, psi)
    frpos = trans2d(np.array([[a],[-tread_f]]),[1,1], gpos, psi)
    f1pos = trans2d(np.array([[a],[tread_f + fm]]),[1,1], gpos, psi)
    f2pos = trans2d(np.array([[a],[-(tread_f + fm)]]),[1,1], gpos, psi)
    
    rpos = trans2d(np.array([[-b],[0]]),[1,1], gpos, psi)
    rlpos = trans2d(np.array([[-b],[tread_r]]),[1,1], gpos, psi)
    rrpos = trans2d(np.array([[-b],[-tread_r]]),[1,1], gpos, psi)
    f3pos = trans2d(np.array([[-b],[tread_r + fm]]),[1,1], gpos, psi)
    f4pos = trans2d(np.array([[-b],[-(tread_r + fm)]]),[1,1], gpos, psi)
    
    tire1 = trans2d(tire,[1,1],flpos,psi+delta1)
    tire2 = trans2d(tire,[1,1],frpos,psi+delta2)
    tire3 = trans2d(tire,[1,1],rlpos,psi+delta3)
    tire4 = trans2d(tire,[1,1],rrpos,psi+delta4)
    
    force1 = trans2d(f1,[FORCE_ZOOM_X, FORCE_ZOOM_Y],f1pos,psi+delta1) 
    force2 = trans2d(f2,[FORCE_ZOOM_X, FORCE_ZOOM_Y],f2pos,psi+delta2) 
    force3 = trans2d(f3,[FORCE_ZOOM_X, FORCE_ZOOM_Y],f3pos,psi+delta3) 
    force4 = trans2d(f4,[FORCE_ZOOM_X, FORCE_ZOOM_Y],f4pos,psi+delta4) 

    # chassis center
    artists = []
    artists.extend(plt.plot([gpos[0],fpos[0]],[gpos[1],fpos[1]],'k',linewidth=1))
    artists.extend(plt.plot([gpos[0],rpos[0]],[gpos[1],rpos[1]],'k',linewidth=1))
    
    
    # chassis front
    artists.extend(plt.plot([flpos[0],frpos[0]],[flpos[1],frpos[1]],'k',linewidth=1))
    artists.extend(plt.plot([flpos[0],frpos[0]],[flpos[1],frpos[1]],'k',linewidth=1))
    
    # chassis rear
    artists.extend(plt.plot([rlpos[0],rrpos[0]],[rlpos[1],rrpos[1]],'k',linewidth=1))
    artists.extend(plt.plot([rlpos[0],rrpos[0]],[rlpos[1],rrpos[1]],'k',linewidth=1))
    
    # Front tire
    
    artists.extend(plt.plot(tire1[0,:],tire1[1,:],'k',linewidth=1))
    artists.extend(plt.fill(tire1[0,:],tire1[1,:],color=[.4,.4,.4]))
    artists.extend(plt.plot(tire2[0,:],tire2[1,:],'k',linewidth=1))
    artists.extend(plt.fill(tire2[0,:],tire2[1,:],color=[.4,.4,.4]))
    
    # Rear Tire
    artists.extend(plt.plot(tire3[0,:],tire3[1,:],'k',linewidth=1))
    artists.extend(plt.fill(tire3[0,:],tire3[1,:],color=[.4,.4,.4]))
    artists.extend(plt.plot(tire4[0,:],tire4[1,:],'k',linewidth=1))
    artists.extend(plt.fill(tire4[0,:],tire4[1,:],color=[.4,.4,.4]))
    
    # Force
    artists.extend(plt.plot(force1[0,0:2] ,force1[1,0:2],'r',linewidth=1))
    artists.extend(plt.plot(force1[0,2:4],force1[1,2:4],'g',linewidth=1))
    artists.extend(plt.plot(force2[0,0:2] ,force2[1,0:2],'r',linewidth=1))
    artists.extend(plt.plot(force2[0,2:4],force2[1,2:4],'g',linewidth=1))
    artists.extend(plt.plot(force3[0,0:2] ,force3[1,0:2],'r',linewidth=1))
    artists.extend(plt.plot(force3[0,2:4],force3[1,2:4],'g',linewidth=1))
    artists.extend(plt.plot(force4[0,0:2] ,force4[1,0:2],'r',linewidth=1))
    artists.extend(plt.plot(force4[0,2:4],force4[1,2:4],'g',linewidth=1))
                   
    # draw_cog
    artists.extend(drawCog(east,north,psi+pi/2,0.2))

    return artists
    
def plotVehicleAndCourse(east,north,psi,delta1,delta2,delta3,delta4,E,N):
    retlist = plt.plot(E,N,'b')
    arts = plotVehicle(east,north,psi,delta1,delta2,delta3,delta4)
    retlist.extend(arts)
    return retlist

def plotVehicleAndCourseWithForce(east,north,psi,delta1,delta2,delta3,delta4,fx1,fx2,fx3,fx4,fy1,fy2,fy3,fy4,E,N):
    retlist = plt.plot(E,N,'b')
    arts = plotVehicleWithForce(east,north,psi,delta1,delta2,delta3,delta4,fx1,fx2,fx3,fx4,fy1,fy2,fy3,fy4)
    retlist.extend(arts)
    return retlist

class Animator:
    """
    Animation Class
        Takes the result of the experiment and shows / saves the animation.
    """
    def __init__(self):
        self.artists = []

    def buildAnim(self, storage, fig):
        N = len(storage.E)         # size of storage
        frameInterval = 3;         # saves every 3 frame from whole storage
        animIntervalMillisec = 100 # animation interval time between frames

        for i in np.arange(0,N,frameInterval):
            east = storage.E[i];
            north = storage.N[i];
            psi = storage.Psi[i];
            delta1 = storage.delta[:,0][i];
            delta2 = storage.delta[:,1][i];
            delta3 = storage.delta[:,2][i];
            delta4 = storage.delta[:,3][i];
            fx1 = storage.Fx[:,0][i];
            fx2 = storage.Fx[:,1][i];
            fx3 = storage.Fx[:,2][i];
            fx4 = storage.Fx[:,3][i];
            fy1 = storage.Fy[:,0][i];
            fy2 = storage.Fy[:,1][i];
            fy3 = storage.Fy[:,2][i];
            fy4 = storage.Fy[:,3][i];
            print i
            plt.grid()
            plt.axis('equal')
            # self.artists.append(plotVehicleAndCourse(east,north,psi,delta1,delta2,delta3,delta4,storage.E[0:i:interval],storage.N[0:i:interval]))
            self.artists.append(plotVehicleAndCourseWithForce(
                east,north,psi,delta1,delta2,delta3,delta4,
                fx1,fx2,fx3,fx4,fy1,fy2,fy3,fy4,
                storage.E[0:i:frameInterval],storage.N[0:i:frameInterval]))
            
        # Set up formatting for the movie files
        Writer = animation.writers['ffmpeg']
        self.writer = Writer(fps=30, metadata=dict(artist='Me'), bitrate=1800)
        self.ani = animation.ArtistAnimation(fig,self.artists,interval=animIntervalMillisec,blit=True)

    def showAnim(self,storage):
        fig = plt.figure(figsize=(16,9))
        self.buildAnim(storage,fig)
        plt.show() # To show anim in window
        
    def saveAnim(self,storage,mp4FileName):
        fig = plt.figure(figsize=(16,9))
        self.buildAnim(storage,fig)
        self.ani.save(mp4FileName, writer=self.writer) # To save anim in mp4 file
